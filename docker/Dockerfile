FROM osrf/ros:noetic-desktop

# Set environment variables
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

# Temporary workaround as GPG key is updated recently
RUN rm /etc/apt/sources.list.d/ros1-latest.list \
  && rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros-apt-source.deb \
    && rm -f /tmp/ros-apt-source.deb

# Install dependencies
RUN apt-get update && apt-get install -y \
      --allow-unauthenticated \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python3-catkin-tools \
      ros-noetic-rviz \
      ros-noetic-cv-bridge \
      ros-noetic-image-transport \
      ros-noetic-message-filters \
      ros-noetic-tf && \
      rm -rf /var/lib/apt/lists/*

# Install Ceres
RUN apt-get update && apt-get install -y \
      --allow-unauthenticated \
      git \
      build-essential \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev && \
      rm -rf /var/lib/apt/lists/*

WORKDIR /
RUN git clone --depth 1 --branch 1.12.0 https://ceres-solver.googlesource.com/ceres-solver
WORKDIR /ceres-solver
RUN mkdir build \
    && cd build \
    && cmake ../ \
    && make -j8 \
    && make test \
    && make install
WORKDIR /

# Set workspace
RUN mkdir -p /root/catkin_ws
WORKDIR /root/catkin_ws

# Install helper tools
RUN apt-get update && \
    apt-get install -y tmux && \
    rm -rf /var/lib/apt/lists/*

# Install format and lint tools
RUN apt-get update && \
    apt-get install -y \
      clang-format-18 \
      clang-tidy-18 && \
      rm -rf /var/lib/apt/lists/*

# Set clang-format and clang-tidy alternatives
RUN update-alternatives --install \
    /usr/bin/clang-format \
    clang-format \
    /usr/bin/clang-format-18 100

RUN update-alternatives --install \
    /usr/bin/clang-tidy \
    clang-tidy \
    /usr/bin/clang-tidy-18 100