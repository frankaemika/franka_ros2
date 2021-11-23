FROM ros:foxy
RUN apt-get update -y && apt-get install -y --allow-unauthenticated \
    clang-10 \
    clang-format-10 \
    clang-tidy-10 \
    python3-pip \
    libpoco-dev \
    libeigen3-dev \
    ros-foxy-control-msgs \
    ros-foxy-xacro \
    ros-foxy-ament-cmake-clang-format \
    ros-foxy-ament-clang-format \
    ros-foxy-ament-flake8 \
    ros-foxy-ament-cmake-clang-tidy \
    ros-foxy-angles \
    ros-foxy-ros2-control \
    ros-foxy-realtime-tools \
    ros-foxy-control-toolbox \
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
    && git submodule init \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF  .. \
    && make franka -j$(nproc) \
    && make install



