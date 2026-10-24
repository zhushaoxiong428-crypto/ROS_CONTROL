#include "microros_internal.h"

#include <errno.h>

namespace microros {

// Agent found by the last successful discovery (network byte order, 0 = none).
// Reconnects try it first and only sweep the subnet again when it stops answering.
static in_addr_t s_last_agent_addr = 0;

static constexpr int kMaxSendRetries = 5;

extern "C" bool transport_open_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    ctx->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (ctx->fd < 0) {
        return false;
    }

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(ctx->local_port);
    bind(ctx->fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));

    const int broadcast = 1;
    setsockopt(ctx->fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    return true;
}

extern "C" bool transport_close_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    if (ctx->fd >= 0) {
        close(ctx->fd);
        ctx->fd = -1;
    }
    return true;
}

// A burst of datagrams can exhaust the Wi-Fi TX buffers; wait a tick and retry.
static void send_to_host(int fd, const uint8_t *buf, size_t len, uint32_t host, uint16_t port) {
    sockaddr_in to = {};
    to.sin_family = AF_INET;
    to.sin_port = port;
    to.sin_addr.s_addr = htonl(host);
    for (int attempt = 0; attempt < kMaxSendRetries; ++attempt) {
        if (sendto(fd, buf, len, 0, reinterpret_cast<sockaddr *>(&to), sizeof(to)) >= 0 ||
            errno != ENOMEM) {
            return;
        }
        vTaskDelay(1);
    }
}

extern "C" size_t transport_write_udp(
    uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    if (ctx->discovering) {
        // Broadcast reaches the agent on most networks; the per-host sweep
        // covers access points that drop broadcast between clients.
        const uint16_t port = ctx->remote.sin_port;
        send_to_host(ctx->fd, buf, len, SubnetBroadcast(ctx->local_ip, ctx->netmask), port);
        ForEachSubnetHost(ctx->local_ip, ctx->netmask, [&](uint32_t host) {
            send_to_host(ctx->fd, buf, len, host, port);
        });
        return len;
    }

    const int sent = sendto(
        ctx->fd,
        buf,
        len,
        0,
        reinterpret_cast<sockaddr *>(&ctx->remote),
        sizeof(ctx->remote));
    return sent > 0 ? static_cast<size_t>(sent) : 0;
}

extern "C" size_t transport_read_udp(
    uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(ctx->fd, &readfds);

    timeval tv = {};
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
    const int ret = select(ctx->fd + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return 0;
    }

    sockaddr_in sender = {};
    socklen_t sender_len = sizeof(sender);
    const int received = recvfrom(
        ctx->fd, buf, len, 0, reinterpret_cast<sockaddr *>(&sender), &sender_len);
    if (received > 0) {
        ctx->last_sender = sender;
    }
    return received > 0 ? static_cast<size_t>(received) : 0;
}

static bool ensure_uart_driver_initialized(UartTransportContext *ctx) {
    if (ctx == nullptr) {
        return false;
    }

    if (s_uart_transport_initialized && uart_is_driver_installed(ctx->port)) {
        return true;
    }

    const uart_config_t uart_config = {
        .baud_rate = net_cfg::kMicroRosUartBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    const bool driver_already_installed = uart_is_driver_installed(ctx->port);
    esp_err_t err = ESP_OK;
    if (!driver_already_installed) {
        err = uart_driver_install(
            ctx->port,
            net_cfg::kMicroRosUartRxBufferSize,
            net_cfg::kMicroRosUartTxBufferSize,
            0,
            nullptr,
            0);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "failed to install UART%d driver: %s",
                     static_cast<int>(ctx->port), esp_err_to_name(err));
            return false;
        }
    }

    err = uart_param_config(ctx->port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "failed to configure UART%d: %s",
                 static_cast<int>(ctx->port), esp_err_to_name(err));
        if (!driver_already_installed) {
            uart_driver_delete(ctx->port);
        }
        return false;
    }

    err = uart_set_pin(
        ctx->port,
        net_cfg::kMicroRosUartTxPin,
        net_cfg::kMicroRosUartRxPin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "failed to set UART%d pins: %s",
                 static_cast<int>(ctx->port), esp_err_to_name(err));
        if (!driver_already_installed) {
            uart_driver_delete(ctx->port);
        }
        return false;
    }

    uart_flush_input(ctx->port);
    s_uart_transport_initialized = true;
    return true;
}

static bool transport_open_uart(uxrCustomTransport *transport) {
    if (transport == nullptr) {
        return false;
    }
    return ensure_uart_driver_initialized(
        static_cast<UartTransportContext *>(transport->args));
}

static bool transport_close_uart(uxrCustomTransport *transport) {
    return transport != nullptr && transport->args != nullptr;
}

static size_t transport_write_uart(
    uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *) {
    if (transport == nullptr || transport->args == nullptr || buf == nullptr || len == 0) {
        return 0;
    }

    auto *ctx = static_cast<UartTransportContext *>(transport->args);
    const int written = uart_write_bytes(
        ctx->port, reinterpret_cast<const char *>(buf), len);
    return written > 0 ? static_cast<size_t>(written) : 0;
}

static size_t transport_read_uart(
    uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *) {
    if (transport == nullptr || transport->args == nullptr || buf == nullptr || len == 0) {
        return 0;
    }

    auto *ctx = static_cast<UartTransportContext *>(transport->args);
    const int wait_ms = timeout <= 0
        ? 0
        : (timeout > net_cfg::kMicroRosUartReadMaxWaitMs ? net_cfg::kMicroRosUartReadMaxWaitMs : timeout);
    const int received = uart_read_bytes(ctx->port, buf, len, pdMS_TO_TICKS(wait_ms));
    taskYIELD();
    return received > 0 ? static_cast<size_t>(received) : 0;
}

bool setup_udp_transport() {
    memset(&s_udp_ctx, 0, sizeof(s_udp_ctx));
    s_udp_ctx.fd = -1;
    s_udp_ctx.local_port = static_cast<uint16_t>(atoi(net_cfg::kMicroRosLocalPort));
    s_udp_ctx.remote.sin_family = AF_INET;
    s_udp_ctx.remote.sin_port = htons(system_globals().runtime.microros_agent_port);

    if (IsAutoAgentIp(system_globals().runtime.microros_agent_ip)) {
        s_udp_ctx.remote.sin_addr.s_addr = s_last_agent_addr;
    } else if (inet_pton(AF_INET, system_globals().runtime.microros_agent_ip, &s_udp_ctx.remote.sin_addr.s_addr) != 1) {
        ESP_LOGE(kTag, "invalid micro-ROS agent IP: %s", system_globals().runtime.microros_agent_ip);
        return false;
    }

    rmw_uros_set_custom_transport(
        false,
        &s_udp_ctx,
        transport_open_udp,
        transport_close_udp,
        transport_write_udp,
        transport_read_udp);
    return true;
}

static bool read_sta_subnet(uint32_t *ip, uint32_t *netmask) {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t info = {};
    if (netif == nullptr || esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == 0) {
        return false;
    }
    *ip = ntohl(info.ip.addr);
    *netmask = ntohl(info.netmask.addr);
    return true;
}

// Ping the configured agent. With agent IP "auto" in UDP mode, try the last
// discovered agent and otherwise ping the whole subnet; whichever host answers
// with a valid XRCE-DDS reply becomes the agent.
bool ping_agent() {
    const bool auto_udp =
        system_globals().runtime.wifi_comm_mode == WifiCommMode::kMicroRos &&
        IsAutoAgentIp(system_globals().runtime.microros_agent_ip);
    if (!auto_udp || s_udp_ctx.remote.sin_addr.s_addr != 0) {
        if (rmw_uros_ping_agent(ros_cfg::kInitialAgentPingTimeoutMs, ros_cfg::kInitialAgentPingAttempts) ==
            RMW_RET_OK) {
            return true;
        }
        if (!auto_udp) {
            return false;
        }
    }

    if (!read_sta_subnet(&s_udp_ctx.local_ip, &s_udp_ctx.netmask)) {
        return false;
    }
    memset(&s_udp_ctx.last_sender, 0, sizeof(s_udp_ctx.last_sender));
    s_udp_ctx.discovering = true;
    const bool found =
        rmw_uros_ping_agent(ros_cfg::kAgentDiscoveryTimeoutMs, ros_cfg::kAgentDiscoveryAttempts) == RMW_RET_OK;
    s_udp_ctx.discovering = false;
    // The ping returns on the first valid reply, so the last sender is the agent.
    if (!found || s_udp_ctx.last_sender.sin_addr.s_addr == 0) {
        s_udp_ctx.remote.sin_addr.s_addr = 0;
        return false;
    }

    s_udp_ctx.remote.sin_addr = s_udp_ctx.last_sender.sin_addr;
    s_last_agent_addr = s_udp_ctx.remote.sin_addr.s_addr;
    char agent_ip[16] = {};
    inet_ntop(AF_INET, &s_udp_ctx.remote.sin_addr, agent_ip, sizeof(agent_ip));
    ESP_LOGI(kTag, "Discovered micro-ROS agent at %s:%u", agent_ip,
             static_cast<unsigned>(system_globals().runtime.microros_agent_port));
    return true;
}

bool setup_uart_transport() {
    memset(&s_uart_ctx, 0, sizeof(s_uart_ctx));
    s_uart_ctx.port = net_cfg::kMicroRosUartPort;
    rmw_uros_set_custom_transport(
        true,
        &s_uart_ctx,
        transport_open_uart,
        transport_close_uart,
        transport_write_uart,
        transport_read_uart);
    return true;
}

}  // namespace microros
