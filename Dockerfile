FROM industryalphainc/ia-amr-ros:base

# Fix ROS GPG key issue and install build dependencies
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 || true && \
    apt-get update --allow-insecure-repositories && \
    apt-get install -y --allow-unauthenticated \
    cmake \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy source code
COPY . /workspace
WORKDIR /workspace

# Build gmapping_json_server
RUN mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Set PATH to include the built binary
ENV PATH="/workspace/build:${PATH}"

WORKDIR /
