FROM osrf/ros:humble-desktop
LABEL maintainer="Lan Wu <Lan.Wu-2@uts.edu.au>"

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Base tools + zsh
# -----------------------------
RUN apt-get update && apt-get install -y \
    wget git zsh tmux vim g++ build-essential \
    cmake \
    python3-pip python3-vcstool \
    python3-colcon-common-extensions \
    locales \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y wget git && rm -rf /var/lib/apt/lists/*
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t robbyrussell \
    -p git \
    -p ssh-agent \
    -p https://github.com/agkozak/zsh-z \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting

SHELL ["/usr/bin/zsh", "-c"]

# -----------------------------
# System deps (不装 libopenvdb-dev，用你自己的 OpenVDB)
# -----------------------------
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libboost-all-dev \
    libtbb-dev \
    libblosc-dev \
    libpcl-dev \
    libceres-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# OpenVDB from source
# -----------------------------
RUN git clone --depth 1 https://github.com/nachovizzo/openvdb.git -b nacho/vdbfusion /tmp/openvdb \
    && cd /tmp/openvdb \
    && mkdir build && cd build \
    && cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUSE_ZLIB=OFF .. \
    && make -j4 all install \
    && cd / \
    && rm -rf /tmp/openvdb

# -----------------------------
# Workspace & your repo
# -----------------------------
RUN mkdir -p /workspace/vdb_gpdf_mapping_ws/src /workspace/data
WORKDIR /workspace/vdb_gpdf_mapping_ws

RUN git clone --recurse-submodules https://github.com/UTS-RI/VDB_GPDF.git src/VDB_GPDF && \
    cd src/VDB_GPDF && \
    git checkout ros2

# -----------------------------
# ROS2 PCL bridge (pcl_ros + pcl_conversions)
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Build with colcon
# -----------------------------
RUN source /opt/ros/humble/setup.zsh && \
    colcon build --symlink-install

# -----------------------------
# Shell environment
# -----------------------------
RUN echo 'source /opt/ros/humble/setup.zsh' >> /root/.zshrc && \
    echo 'source /workspace/vdb_gpdf_mapping_ws/install/setup.zsh' >> /root/.zshrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /root/.zshrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.zshrc

CMD ["zsh"]
