#!/usr/bin/env bash
# 克隆本仓库后、第一次构建前运行一次（CI 也会运行）。
#
# 1. dependencies.lock 以绝对路径记录本地组件 micro_ros_espidf_component。
#    换一台机器该路径不存在，组件管理器会改去组件库下载并失败，
#    因此把它改写为当前仓库的实际路径。
# 2. git 不保存文件修改时间。micro-ROS 组件在每次 CMake 配置时都会运行
#    libmicroros.mk，由 make 按时间戳判断是否要从源码重编 libmicroros.a；
#    克隆后时间戳顺序随机，可能触发需要 colcon 和联网的完整重编。
#    这里按 libmicroros.mk 的依赖顺序重设时间戳，直接使用已提交的预编译库。
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
component_dir="${repo_root}/managed_components/micro_ros_espidf_component"

sed -i "s|^\(\s*path: \).*/managed_components/micro_ros_espidf_component$|\1${component_dir}|" \
  "${repo_root}/dependencies.lock"

cd "${component_dir}"
touch -d '2020-01-01 00:00:00' esp32_toolchain.cmake.in
touch -d '2020-01-01 00:00:01' esp32_toolchain.cmake micro_ros_dev/install micro_ros_src/src
touch -d '2020-01-01 00:00:02' micro_ros_src/install
touch -d '2020-01-01 00:00:03' libmicroros.a

echo "prepared: dependencies.lock -> ${component_dir}, micro-ROS prebuilt timestamps fixed"
