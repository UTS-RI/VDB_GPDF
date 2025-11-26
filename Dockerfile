# ROS2 Humble desktop image (includes RViz, SLAM tools, etc.)
FROM osrf/ros:humble-desktop

LABEL maintainer="Lan Wu <Lan.Wu-2@uts.edu.au>"

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Base tools + Zsh environment
# -----------------------------
RUN apt-get update && apt-get install -y \
    wget git zsh tmux vim g++ build-essential \
    cmake \
    python3-pip python3-vcstool \
    python3-colcon-common-extensions \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Zsh theme and plugins
RUN apt-get update && apt-get install -y wget git && rm -rf /var/lib/apt/lists/*

RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t robbyrussell \
    -p git \
    -p ssh-agent \
    -p https://github.com/agkozak/zsh-z \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting

# Default shell
SHELL ["/usr/bin/zsh", "-c"]

# -----------------------------
# ROS2 build dependencies
# -----------------------------
RUN apt-get update && apt-get install -y \
    # Math & core libraries
    libeigen3-dev \
    libboost-all-dev \
    # OpenVDB
    libopenvdb-dev \
    libtbb-dev \
    libblosc-dev \
    # PCL
    libpcl-dev \
    # Ceres / gflags / glog
    libceres-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    # Misc tools
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# ROS2 workspace setup
# -----------------------------
RUN mkdir -p /workspace/vdb_gpdf_mapping_ws/src /workspace/data
WORKDIR /workspace/vdb_gpdf_mapping_ws

# Clone the repository (ROS2 branch)
RUN git clone --recurse-submodules https://github.com/UTS-RI/VDB_GPDF.git src/VDB_GPDF && \
    cd src/VDB_GPDF && \
    git checkout ros2

# ROS2 environment defaults for Zsh
RUN echo 'source /opt/ros/humble/setup.zsh' >> /root/.zshrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /root/.zshrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.zshrc

# Default command
CMD ["zsh"]