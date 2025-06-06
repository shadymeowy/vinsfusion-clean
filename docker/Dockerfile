FROM ros:kinetic-perception

# Set environment variables
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

# Install dependencies
RUN apt-get update && apt-get install -y \
      --allow-unauthenticated \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      ros-kinetic-rviz \
      ros-kinetic-cv-bridge \
      ros-kinetic-image-transport \
      ros-kinetic-message-filters \
      ros-kinetic-tf && \
      rm -rf /var/lib/apt/lists/*

# Install Ceres
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