FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /workspaces

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p -m 0700 /run/user/"${USER_UID}" \
    && mkdir -p -m 0700 /run/user/"${USER_UID}"/gdm \
    && chown user:user /run/user/"${USER_UID}" \
    && chown user:user /workspaces \
    && chown user:user /run/user/"${USER_UID}"/gdm \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-xacro \
    ros-humble-controller-interface \
    ros-humble-realtime-tools \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-control-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-gazebo-ros \
    ros-humble-control-toolbox \
    ros-humble-ackermann-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-ament-cmake-clang-tidy \
    libignition-gazebo6-dev \
    libpoco-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir ~/source_code
RUN cd ~/source_code && git clone https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git switch 0.13.2 \
    && git submodule init \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF  .. \
    && make franka -j$(nproc) \
    && cpack -G DEB \
    && sudo dpkg -i libfranka*.deb

RUN addgroup realtime
RUN usermod -a -G realtime $USERNAME

ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"

# set the default user to the newly created user
USER $USERNAME

RUN echo "source /ros_entrypoint.sh" >>~/.bashrc
RUN echo "export ROS_DOMAIN_ID=100" >>~/.bashrc
