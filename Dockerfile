FROM osrf/ros:noetic-desktop-full
LABEL maintainer="Lan Wu <Lan.Wu-2@uts.edu.au>"

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

# install zsh
RUN apt update && apt install -y wget git zsh tmux vim g++
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t robbyrussell \
    -p git \
    -p ssh-agent \
    -p https://github.com/agkozak/zsh-z \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting

# Install utilities
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python3 utils
RUN python3 -m pip install --no-cache  catkin-tools

# Install extra ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install OpenVDB dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    libblosc-dev \
    libboost-iostreams-dev \
    libboost-system-dev \
    libboost-system-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install OpenVDB from source, use -j$(nproc) if with enough ram and swap
RUN git clone --depth 1 https://github.com/nachovizzo/openvdb.git -b nacho/vdbfusion \
    && cd openvdb \
    && mkdir build && cd build \
    && cmake  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUSE_ZLIB=OFF .. \
    && make -j4 all install \
    && cd / \
    && rm -rf /openvdb

# vdb_gpdf_mapping
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
RUN echo "source /opt/ros/noetic/setup.bashrc" >> ~/.bashrc

# install glog
RUN mkdir -p /workspace/lib \
    && cd /workspace/lib \
    && git clone https://github.com/google/glog.git \
    && cd glog \
    && git fetch --all --tags \
    && git checkout tags/v0.4.0 -b v0.4.0 \
    && mkdir build && cd build \
    && cmake .. && make -j$(nproc) \
    && make install

# install gflag
RUN cd /workspace/lib \
    && git clone https://github.com/gflags/gflags.git \
    && cd gflags \
    && mkdir build && cd build \
    && cmake .. -DBUILD_SHARED_LIBS=ON && make \
    && make install

RUN cd ~ \
    && rm -rf /workspace/lib

RUN mkdir -p /workspace/vdb_gpdf_mapping_ws /workspace/data
WORKDIR /workspace/vdb_gpdf_mapping_ws