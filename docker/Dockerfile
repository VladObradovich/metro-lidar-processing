FROM ros:humble-ros-base AS universal

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
    python3-numpy python3-scipy python3-matplotlib python3-opencv python3-pytest \
    python3-flake8 python3-pip python3-venv \
    ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
    ros-humble-ament-flake8 ros-humble-ament-pep257 \
    ros-humble-rclcpp-components ros-humble-rclcpp-lifecycle \
    ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-vision-msgs ros-humble-diagnostic-updater \
    ros-humble-rosbag2 ros-humble-rosbag2-py \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
    && rm -rf /var/lib/apt/lists/*

# Analysis tools are PyPI packages; keep the system NumPy/SciPy used by ROS.
# scikit-learn 1.3 supports Ubuntu 22.04's Python 3.10 and NumPy 1.21.
COPY analysis-requirements.txt /tmp/analysis-requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/analysis-requirements.txt \
    && rm /tmp/analysis-requirements.txt

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && mkdir -p /ws /data /results /tmp/wayland-runtime \
    && chown -R "${USER_UID}:${USER_GID}" /ws /results /tmp/wayland-runtime \
    && chmod 0700 /tmp/wayland-runtime

COPY gui-env.sh /etc/metro-gui-env.sh
COPY ros-dev-env.sh /etc/ros-dev-env.sh
COPY entrypoint.sh /usr/local/bin/metro-dev-entrypoint
RUN chmod 0755 /usr/local/bin/metro-dev-entrypoint \
    && echo 'source /etc/ros-dev-env.sh' >> "/home/${USERNAME}/.bashrc"

ENV ROS_DOMAIN_ID=42 \
    ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

USER ${USERNAME}
WORKDIR /ws
RUN rosdep update --rosdistro humble

ENTRYPOINT ["/usr/local/bin/metro-dev-entrypoint"]
CMD ["sleep", "infinity"]

FROM universal AS desktop

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && add-apt-repository -y ppa:kisak/turtle \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 qtwayland5 mesa-utils \
    libgl1-mesa-dri libglx-mesa0 libgl1-mesa-dev xvfb \
    && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}

FROM universal AS default
