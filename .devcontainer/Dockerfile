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
    ros-humble-gazebo-ros \
    ros-humble-control-toolbox \
    ros-humble-ackermann-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    ros-humble-moveit \
    ros-humble-joint-state-broadcaster \
    libignition-gazebo6-dev \
    ros-humble-joint-trajectory-controller \
    libpoco-dev \
    libeigen3-dev \
    ament-cmake-clang-tidy \
    && rm -rf /var/lib/apt/lists/*

ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"

# set the default user to the newly created user
USER $USERNAME

RUN echo "source /ros_entrypoint.sh" >>~/.bashrc
