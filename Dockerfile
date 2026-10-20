# syntax=docker/dockerfile:1
#
# docker build -t r3v-ros:jazzy .
# docker build -t r3v-ros:humble --build-arg ROS_DISTRO=humble .
#
# Initialize the submodules first: git submodule update --init --recursive

ARG ROS_DISTRO=jazzy

# Build dependencies only (also usable as a development image:
# docker build --target deps -t r3v-ros:deps .).
FROM ros:${ROS_DISTRO}-ros-base AS deps
ARG RUST_TOOLCHAIN=stable
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:${PATH}

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential ca-certificates cmake curl python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*
# Ubuntu's rustc is too old for the r3v crates (edition 2024).
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
    | sh -s -- -y --no-modify-path --profile minimal --default-toolchain "${RUST_TOOLCHAIN}"

WORKDIR /ws
COPY package.xml src/r3v_ros/package.xml
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
 && rm -rf /var/lib/apt/lists/*


FROM deps AS builder
ARG ROS_DISTRO
COPY . src/r3v_ros
# Cached cargo registry and target dir: rebuilds skip compiling the Rust
# crates unless they changed. One target cache per distro (different glibc).
RUN --mount=type=cache,target=/opt/rust/cargo/registry \
    --mount=type=cache,id=r3v-cargo-target-${ROS_DISTRO},target=/opt/r3v-cargo-target \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DR3V_CARGO_TARGET_DIR=/opt/r3v-cargo-target


FROM ros:${ROS_DISTRO}-ros-base AS runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility \
    XDG_RUNTIME_DIR=/tmp/runtime-root

COPY package.xml /tmp/r3v_ros/package.xml
# Vulkan loader + Mesa drivers (Intel, AMD, lavapipe CPU fallback); NVIDIA
# drivers come from the NVIDIA container toolkit.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libegl1 libvulkan1 mesa-vulkan-drivers \
      "ros-${ROS_DISTRO}-image-transport-plugins" \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths /tmp/r3v_ros --ignore-src -y -t exec --rosdistro "${ROS_DISTRO}" \
 && rm -rf /var/lib/apt/lists/* /tmp/r3v_ros

# wgpu probes EGL/Wayland too, which wants a runtime dir.
RUN mkdir -m 0700 -p "${XDG_RUNTIME_DIR}"

COPY --from=builder /ws/install /ws/install
COPY config/r3v.yaml /config/r3v.yaml
COPY docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "r3v_ros", "r3v.launch.py", "config:=/config/r3v.yaml"]
