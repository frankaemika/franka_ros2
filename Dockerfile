FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /tmp

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update -y && apt-get install -y --allow-unauthenticated \
    clang-14 \
    clang-format-14 \
    clang-tidy-14 \
    python3-pip \
    libpoco-dev \
    libeigen3-dev \
    ros-humble-control-msgs \
    ros-humble-xacro \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-clang-format \
    ros-humble-ament-flake8 \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-angles \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \    
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-controller-interface \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest

RUN mkdir ~/source_code    
RUN cd ~/source_code && git clone https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git checkout 0.13.2 \
    && git submodule init \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF  .. \
    && make franka -j$(nproc) \
    && cpack -G DEB \
    && sudo dpkg -i libfranka*.deb

# set the default user to the newly created user
USER $USERNAME
