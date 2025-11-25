#!/bin/bash
# Build script for eYs3D ROS Melodic Docker image

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
IMAGE_NAME="eys3d_ros_melodic"
IMAGE_TAG="latest"

echo "Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Build context: ${SCRIPT_DIR}"
echo ""

# Build the Docker image
docker build \
    -t ${IMAGE_NAME}:${IMAGE_TAG} \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo ""
echo "Build complete!"
echo "Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo ""
echo "Next steps:"
echo "1. Run './docker-run.sh' to start the container"
echo "2. Inside the container, run 'catkin_make' to build the workspace"
echo "3. Launch the camera with 'roslaunch dm_preview G100Plus_1.launch'"
