# Dockerfile for eYs3D ROS Melodic Development Environment
# Base: Ubuntu 18.04 with ROS Melodic Desktop Full
# Supports: USB camera access, X11 GUI (rviz), x86_64 architecture

FROM osrf/ros:melodic-desktop-full-bionic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic
ENV CATKIN_WS=/catkin_ws

# Install system dependencies for eYs3D camera
RUN apt-get update && apt-get install -y \
    # Build tools
    cmake \
    git \
    pkg-config \
    # OpenCV and image processing dependencies
    libgtk2.0-dev \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libjpeg9 \
    libpng-dev \
    libtiff-dev \
    libdc1394-22-dev \
    # USB support
    libusb-dev \
    libusb-1.0-0-dev \
    usbutils \
    # Python dependencies
    python-dev \
    python-numpy \
    # ROS additional tools
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    # Utilities
    vim \
    nano \
    wget \
    curl \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update

# Create catkin workspace
RUN mkdir -p ${CATKIN_WS}/src

# Set working directory
WORKDIR ${CATKIN_WS}

# Source ROS setup automatically in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ${CATKIN_WS}/devel/setup.bash ]; then" >> ~/.bashrc && \
    echo "  source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc

# Add udev rules for USB devices (will need to be mounted at runtime)
RUN mkdir -p /etc/udev/rules.d

# Set up environment for GUI applications
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS environment\n\
source /opt/ros/'${ROS_DISTRO}'/setup.bash\n\
\n\
# Source workspace if built\n\
if [ -f '${CATKIN_WS}'/devel/setup.bash ]; then\n\
  source '${CATKIN_WS}'/devel/setup.bash\n\
fi\n\
\n\
# Execute command\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
