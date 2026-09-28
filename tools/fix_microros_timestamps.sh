#!/usr/bin/env bash
# git 不保存文件修改时间。micro-ROS 组件在每次 CMake 配置时都会运行
# libmicroros.mk，由 make 按时间戳判断是否要从源码重编 libmicroros.a；
# 克隆后时间戳顺序随机，可能触发需要 colcon 和联网的完整重编。
# 本脚本按 libmicroros.mk 的依赖顺序重设时间戳，让已提交的预编译库被直接使用。
set -euo pipefail

cd "$(dirname "$0")/../managed_components/micro_ros_espidf_component"

touch -d '2020-01-01 00:00:00' esp32_toolchain.cmake.in
touch -d '2020-01-01 00:00:01' esp32_toolchain.cmake micro_ros_dev/install micro_ros_src/src
touch -d '2020-01-01 00:00:02' micro_ros_src/install
touch -d '2020-01-01 00:00:03' libmicroros.a

echo "micro-ROS prebuilt timestamps fixed"
