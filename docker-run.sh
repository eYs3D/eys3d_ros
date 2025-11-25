#!/bin/bash
# Run script for eYs3D ROS Melodic with NVIDIA GPU support
# Requires nvidia-container-toolkit to be installed first

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
IMAGE_NAME="eys3d_ros_melodic"
IMAGE_TAG="latest"
CONTAINER_NAME="eys3d_ros_dev"

# Check if nvidia-container-toolkit is installed
if ! docker info 2>/dev/null | grep -q "nvidia"; then
    echo "Warning: NVIDIA runtime not detected in Docker."
    echo "Please run: sudo ./install-nvidia-docker.sh"
    echo ""
fi

# Check if image exists
if ! docker image inspect ${IMAGE_NAME}:${IMAGE_TAG} >/dev/null 2>&1; then
    echo "Error: Docker image '${IMAGE_NAME}:${IMAGE_TAG}' not found!"
    echo "Please run './docker-build.sh' first to build the image."
    exit 1
fi

# Remove existing container if exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container '${CONTAINER_NAME}'..."
    docker rm -f ${CONTAINER_NAME}
fi

# Allow X11 connections
echo "Configuring X11 access..."
xhost +local:docker > /dev/null 2>&1 || echo "Warning: Could not configure xhost"

# Get DISPLAY
DISPLAY_VAR="${DISPLAY:-:0}"

echo "Starting Docker container with NVIDIA GPU support..."
echo "Container: ${CONTAINER_NAME}"
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Display: ${DISPLAY_VAR}"
echo ""

# Run with NVIDIA GPU support
docker run -it \
    --name ${CONTAINER_NAME} \
    --gpus all \
    --privileged \
    --net=host \
    --env="DISPLAY=${DISPLAY_VAR}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${SCRIPT_DIR}/dm_preview:/catkin_ws/src/dm_preview:rw" \
    --volume="/dev:/dev:rw" \
    --device-cgroup-rule='c *:* rmw' \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash

echo ""
echo "Container stopped."
