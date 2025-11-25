# Docker Environment for eYs3D ROS Melodic

This Docker setup provides a complete ROS Melodic environment on Ubuntu 18.04 for running the eYs3D depth camera on newer Ubuntu host systems (e.g., Ubuntu 20.04/22.04).

## Features

- ROS Melodic Desktop Full (Ubuntu 18.04 Bionic)
- NVIDIA GPU support for OpenGL/rviz visualization
- USB device access for eYs3D cameras
- X11 GUI support for rviz and rqt tools
- Pre-installed dependencies (OpenCV, libusb, libgtk, etc.)
- Workspace volume mounting for easy development

## Prerequisites

### 1. Docker

Install Docker on your host system:

```bash
sudo apt-get update
sudo apt-get install docker.io
sudo systemctl start docker
sudo systemctl enable docker
```

Add your user to docker group (to run without sudo):

```bash
sudo usermod -aG docker $USER
newgrp docker  # Or log out and back in
```

### 2. NVIDIA Container Toolkit (Required for rviz/GUI)

For proper OpenGL rendering in rviz, you need NVIDIA GPU support. Run the installation script:

```bash
sudo ./install-nvidia-docker.sh
```

This installs `nvidia-container-toolkit` and configures Docker for GPU access.

Verify the installation:

```bash
docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi
```

## Quick Start

### 1. Build the Docker Image

```bash
chmod +x docker-build.sh docker-run.sh install-nvidia-docker.sh
./docker-build.sh
```

This creates a Docker image named `eys3d_ros_melodic:latest`.

### 2. Run the Container

```bash
./docker-run.sh
```

This starts an interactive container with:
- NVIDIA GPU access for OpenGL rendering
- USB device access for cameras
- X11 forwarding for GUI applications
- Your dm_preview workspace mounted at `/catkin_ws/src/dm_preview`

### 3. Build the ROS Package

Inside the container:

```bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Test the Camera

Connect your eYs3D camera and run:

```bash
# Check if camera is detected
lsusb

# Launch the camera node (example for G53)
roslaunch dm_preview G53_1.launch
```

### 5. Visualize with rviz

Open a new terminal on your host and execute into the running container:

```bash
docker exec -it eys3d_ros_dev bash
source /catkin_ws/devel/setup.bash
rosrun rviz rviz
```

Or use dynamic reconfigure:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Available Camera Launch Files

The package supports multiple camera models:

| Camera | Launch File | Video Modes |
|--------|-------------|-------------|
| G100Plus | `G100Plus_1.launch` | 1-19 |
| G100 | `G100_1.launch` | 1-11 |
| G62 | `G62_1.launch` | 1-9 |
| G53 | `G53_1.launch` | 1-9 |
| G50 | `G50_1.launch` | 1-11 |
| R50 | `R50_1.launch` | 1-5 |
| BMVM0S30A | `BMVM0S30A.launch` | - |

Refer to `camera_name_video_mode_list.txt` for detailed mode specifications.

## Published ROS Topics

After launching, the following topics will be available:

```
/dm_preview/depth/camera_info          # Depth camera info
/dm_preview/depth/image_raw            # Raw depth image
/dm_preview/left/camera_info           # Left camera info
/dm_preview/left/image_color           # Left color image
/dm_preview/right/camera_info          # Right camera info
/dm_preview/right/image_color          # Right color image
/dm_preview/points/data_raw            # Point cloud data
/dm_preview/imu/data_raw               # IMU data (if supported)
/dm_preview/imu/data_raw_processed     # Processed IMU data
```

## Docker Container Management

### Stop the container:
```bash
docker stop eys3d_ros_dev
```

### Restart the container:
```bash
docker start -i eys3d_ros_dev
```

### Execute into running container:
```bash
docker exec -it eys3d_ros_dev bash
```

### Remove the container:
```bash
docker rm -f eys3d_ros_dev
```

### Remove the image:
```bash
docker rmi eys3d_ros_melodic:latest
```

## Troubleshooting

### Camera not detected

```bash
# Inside container, check USB devices
lsusb

# Check USB permissions
ls -l /dev/bus/usb/*/*
```

### GUI/rviz not working (OpenGL errors)

Ensure NVIDIA Container Toolkit is installed:

```bash
# On host
sudo ./install-nvidia-docker.sh

# Verify GPU access inside container
nvidia-smi
glxinfo | grep "OpenGL renderer"
```

The output should show your NVIDIA GPU, not software rendering.

### X11 display errors

```bash
# On host, allow X11 connections
xhost +local:docker

# Check DISPLAY variable inside container
echo $DISPLAY
```

### Build errors

```bash
# Clean and rebuild
cd /catkin_ws
rm -rf build/ devel/
catkin_make
```

## Architecture Support

The Docker environment supports **x86_64** architecture with NVIDIA GPU. The dm_preview package includes platform-specific libraries:

- `libeys3d.X86_64.so` - for x86_64 systems
- `libeys3d.NVIDIA_64.so` - for NVIDIA Jetson (aarch64)

The CMakeLists.txt automatically selects the correct library based on the host processor.

## Development Workflow

1. Edit source files on your host system (they're mounted in the container)
2. Build inside the container: `catkin_make`
3. Test with the camera
4. Changes persist on the host filesystem

## Files

| File | Description |
|------|-------------|
| `Dockerfile` | Docker image definition with ROS Melodic and dependencies |
| `docker-build.sh` | Build the Docker image |
| `docker-run.sh` | Run the container with GPU and USB support |
| `install-nvidia-docker.sh` | Install NVIDIA Container Toolkit (run with sudo) |

## Additional Resources

- [ROS Melodic Documentation](http://wiki.ros.org/melodic)
- [Docker ROS Tutorial](http://wiki.ros.org/docker/Tutorials)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)
