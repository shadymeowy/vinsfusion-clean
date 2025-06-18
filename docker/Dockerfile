FROM ros:noetic-perception

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
RUN git clone --depth 1 --branch 2.2.0 https://ceres-solver.googlesource.com/ceres-solver
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
      clang-tidy-18 \
      clangd-18 && \
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

RUN update-alternatives --install \
    /usr/bin/run-clang-tidy \
    run-clang-tidy \
    /usr/bin/run-clang-tidy-18 100

RUN update-alternatives --install \
    /usr/bin/clang-apply-replacements \
    clang-apply-replacements \
    /usr/bin/clang-apply-replacements-18 100

RUN update-alternatives --install \
    /usr/bin/clangd \
    clangd \
    /usr/bin/clangd-18 100

# Install msgpack
RUN apt-get update && \
    apt-get install -y \
    libmsgpack-dev && \
    rm -rf /var/lib/apt/lists/*

# Install uv and python from distroless image
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
RUN uv python install 3.12