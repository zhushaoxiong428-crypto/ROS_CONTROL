#!/usr/bin/env bash
# Make the vendored micro-ROS prebuilt outputs look up to date.
#
# components/micro_ros_espidf_component/libmicroros.mk decides whether to
# rebuild libmicroros with make timestamps. A fresh git checkout gives every
# file an arbitrary mtime, which can make make rebuild micro-ROS from source
# (needs colcon and network access). Touch the outputs in dependency order so
# only the prebuilt library is used. Run once after clone or checkout.
set -euo pipefail

cd "$(dirname "$0")/../components/micro_ros_espidf_component"

for path in \
    esp32_toolchain.cmake.in \
    esp32_toolchain.cmake \
    micro_ros_dev/install \
    micro_ros_src/src \
    micro_ros_src/install \
    libmicroros.a; do
    touch "$path"
    sleep 0.01
done

echo "micro-ROS prebuilt timestamps prepared"
