FROM ros:jazzy-ros-core-noble

RUN apt-get update -y && \
  apt-get install -y --no-install-recommends \
  apt-transport-https \
  ca-certificates \
  cmake \
  g++ \
  gcc \
  gtk3-nocsd \
  libc6-dev \
  make \
  patch \
  pipewire-audio-client-libraries \
  python3 \
  python3.12-dev \
  python3.12-venv \
  software-properties-common \
  wget \
  && \
  apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*
