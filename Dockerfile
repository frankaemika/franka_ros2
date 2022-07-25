FROM ros:galactic
RUN apt-get update -y && apt-get install -y --allow-unauthenticated \
    clang-6.0 \
    clang-format-6.0 \
    clang-tidy-6.0 \
    python3-pip \
    libpoco-dev \
    libeigen3-dev \
    ros-galactic-control-msgs \
    ros-galactic-xacro \
    ros-galactic-ament-cmake-clang-format \
    ros-galactic-ament-clang-format \
    ros-galactic-ament-flake8 \
    ros-galactic-ament-cmake-clang-tidy \
    ros-galactic-angles \
    ros-galactic-ros2-control \
    ros-galactic-realtime-tools \
    ros-galactic-control-toolbox \
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
