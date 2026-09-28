#pragma once

// 急停命令门控（纯逻辑，可在主机上单元测试）。
//
// 规则：
//   1. 急停锁存期间，所有运动命令都被丢弃。
//   2. 急停释放后，必须先收到一条全零速度命令，之后的命令才会放行；
//      避免释放瞬间执行释放前残留或正在持续发送的非零命令。
class EmergencyStopGate
{
public:
  // 每个控制周期开始时调用一次，传入当前急停标志。
  void Observe(bool estop_active)
  {
    active_ = estop_active;
    if (estop_active)
    {
      awaiting_zero_command_ = true;
    }
  }

  // 返回 true 表示这条命令可以交给后续的互锁和控制器处理。
  bool Admit(bool is_all_stop_command)
  {
    if (active_)
    {
      return false;
    }
    if (awaiting_zero_command_)
    {
      if (!is_all_stop_command)
      {
        return false;
      }
      awaiting_zero_command_ = false;
    }
    return true;
  }

  bool Active() const { return active_; }
  bool AwaitingZeroCommand() const { return awaiting_zero_command_; }

private:
  bool active_ = false;
  bool awaiting_zero_command_ = false;
};
