# Dockerfile for ZED Obstacle Detector ROS Node
# Based on ROS Noetic (CPU-only version)

# Use standard ROS Noetic base image
FROM ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV ROS_PYTHON_VERSION=3

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install additional system dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-eigen \
    ros-noetic-visualization-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-std-msgs \
    ros-noetic-message-runtime \
    ros-noetic-message-generation \
    && rm -rf /var/lib/apt/lists/*

# Install PCL development libraries
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    libpcl1.10 \
    libpcl-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Eigen3
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install additional development tools (optional)
RUN apt-get update && apt-get install -y \
    gdb \
    valgrind \
    cppcheck \
    clang-format \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create workspace directory
WORKDIR /catkin_ws

# Copy package files
COPY package.xml /catkin_ws/src/zed_obstacle_detector/
COPY CMakeLists.txt /catkin_ws/src/zed_obstacle_detector/
COPY launch/ /catkin_ws/src/zed_obstacle_detector/launch/
COPY include/ /catkin_ws/src/zed_obstacle_detector/include/
COPY src/ /catkin_ws/src/zed_obstacle_detector/src/

# Copy the main node file
COPY zed_obstacle_detector_node.cpp /catkin_ws/src/zed_obstacle_detector/

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python \
    matplotlib \
    scipy

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

# Source ROS in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /catkin_ws/devel/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]

# Expose ports for ROS communication
EXPOSE 11311

# Set working directory
WORKDIR /catkin_ws

# Add labels for documentation
LABEL maintainer="DopeBiscuit <abdulrahman.hany003@gmail.com>"
LABEL description="ZED Obstacle Detector ROS Node with PCL processing (CPU-only)"
LABEL version="0.0.1"
LABEL ros_distro="noetic" 