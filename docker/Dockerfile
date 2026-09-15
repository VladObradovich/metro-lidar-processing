FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git gdb clang-format clang-tidy \
    ca-certificates curl sudo bash-completion pkg-config \
    libeigen3-dev libpcl-dev \
    python3-colcon-common-extensions python3-rosdep \
    python3-numpy python3-scipy python3-opencv python3-pytest \
    python3-flake8 python3-pip python3-venv \
    ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
    ros-humble-ament-flake8 ros-humble-ament-pep257 \
    ros-humble-rclcpp-components ros-humble-rclcpp-lifecycle \
    ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-vision-msgs ros-humble-diagnostic-updater \
    ros-humble-rosbag2 ros-humble-rosbag2-py \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rviz2 \
    ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
    mesa-utils xauth xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && mkdir -p /workspaces/metro-lidar-processing /data /results \
       "/home/${USERNAME}/ros_ws/src" \
    && ln -s /workspaces/metro-lidar-processing "/home/${USERNAME}/ros_ws/src/metro-lidar-processing" \
    && chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}/ros_ws" /results

COPY ros-dev-env.sh /etc/ros-dev-env.sh
COPY entrypoint.sh /usr/local/bin/metro-dev-entrypoint
RUN chmod 0755 /usr/local/bin/metro-dev-entrypoint \
    && echo 'source /etc/ros-dev-env.sh' >> "/home/${USERNAME}/.bashrc"

ENV ROS_DOMAIN_ID=42 \
    ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1

USER ${USERNAME}
WORKDIR /workspaces/metro-lidar-processing
RUN rosdep update --rosdistro humble

ENTRYPOINT ["/usr/local/bin/metro-dev-entrypoint"]
CMD ["sleep", "infinity"]
