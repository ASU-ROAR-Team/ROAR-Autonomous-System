# ZED Obstacle Detector - Docker Setup

This directory contains Docker configuration files for running the ZED Obstacle Detector ROS node in a containerized environment.

## Prerequisites

### System Requirements
- Ubuntu 20.04 or later
- Docker Engine 20.10 or later
- Docker Compose 2.0 or later
- NVIDIA Docker runtime (for GPU acceleration)
- NVIDIA drivers compatible with CUDA 11.4

### Hardware Requirements
- ZED camera (ZED2i recommended)
- NVIDIA GPU (optional, for CUDA acceleration)
- USB 3.0 port for ZED camera

## Installation

### 1. Install Docker and NVIDIA Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install NVIDIA Docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

### 2. Clone and Setup

```bash
# Navigate to the zed_obstacle_detector directory
cd Perception/zed_obstacle_detector

# Create necessary directories
mkdir -p config logs rviz_config
```

## Building the Docker Image

### Option 1: Using Docker Compose (Recommended)

```bash
# Build the image
docker-compose build

# Or build with no cache (if you encounter issues)
docker-compose build --no-cache
```

### Option 2: Using Docker directly

```bash
# Build the image
docker build -t zed_obstacle_detector:latest .

# Tag the image (optional)
docker tag zed_obstacle_detector:latest your-registry/zed_obstacle_detector:latest
```

## Running the Container

### Option 1: Using Docker Compose (Recommended)

#### Basic Usage
```bash
# Start the obstacle detector
docker-compose up

# Run in background
docker-compose up -d

# View logs
docker-compose logs -f zed_obstacle_detector
```

#### With ROS Master
```bash
# Start ROS master and obstacle detector
docker-compose --profile ros-master up

# Or start them separately
docker-compose up roscore -d
docker-compose up zed_obstacle_detector
```

#### With Visualization
```bash
# Start with RViz for visualization
docker-compose --profile visualization up
```

### Option 2: Using Docker directly

```bash
# Run the container
docker run --rm -it \
  --runtime=nvidia \
  --network=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/bus/usb:/dev/bus/usb:ro \
  -v $(pwd)/config:/catkin_ws/src/zed_obstacle_detector/config:ro \
  -v $(pwd)/logs:/catkin_ws/logs:rw \
  zed_obstacle_detector:latest

# Or run with specific command
docker run --rm -it \
  --runtime=nvidia \
  --network=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/bus/usb:/dev/bus/usb:ro \
  zed_obstacle_detector:latest \
  roslaunch zed_obstacle_detector pc.launch
```

## Configuration

### Environment Variables

You can customize the behavior by setting environment variables:

```bash
# In docker-compose.yml or docker run command
environment:
  - ROS_MASTER_URI=http://your-ros-master:11311
  - ROS_HOSTNAME=your-hostname
  - DISPLAY=:0
```

### Volume Mounts

The container mounts several volumes for configuration and data:

- `./config/` → `/catkin_ws/src/zed_obstacle_detector/config/` (read-only)
- `./logs/` → `/catkin_ws/logs/` (read-write)
- `/dev/bus/usb/` → `/dev/bus/usb/` (for ZED camera access)

### ROS Parameters

You can pass ROS parameters when launching:

```bash
# Using docker-compose
command: >
  bash -c "
    roslaunch zed_obstacle_detector pc.launch
    camera_name:=zed2i
    enable_ground_filtering:=true
  "

# Using docker run
docker run ... zed_obstacle_detector:latest \
  roslaunch zed_obstacle_detector pc.launch camera_name:=zed2i
```

## Troubleshooting

### Common Issues

#### 1. ZED Camera Not Detected
```bash
# Check if ZED camera is connected
lsusb | grep ZED

# Check USB permissions
sudo chmod 666 /dev/bus/usb/*/*

# Run container with USB device access
docker run --privileged -v /dev/bus/usb:/dev/bus/usb:ro ...
```

#### 2. Display Issues
```bash
# Allow X11 connections
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Run with X11 forwarding
docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ...
```

#### 3. CUDA/GPU Issues
```bash
# Check NVIDIA Docker installation
docker run --rm --gpus all nvidia/cuda:11.4-base-ubuntu20.04 nvidia-smi

# Check GPU availability in container
docker run --rm --gpus all zed_obstacle_detector:latest nvidia-smi
```

#### 4. Build Failures
```bash
# Clean build
docker-compose build --no-cache

# Check available disk space
df -h

# Increase Docker memory limit
# Edit /etc/docker/daemon.json
{
  "default-shm-size": "2G"
}
```

### Debugging

#### Enter the Container
```bash
# Interactive shell
docker-compose exec zed_obstacle_detector bash

# Or with docker run
docker run -it zed_obstacle_detector:latest bash
```

#### Check ROS Topics
```bash
# Inside the container
rostopic list
rostopic echo /zed_obstacle/obstacle_array
```

#### View Logs
```bash
# Docker logs
docker-compose logs -f zed_obstacle_detector

# ROS logs
docker-compose exec zed_obstacle_detector tail -f /catkin_ws/logs/zed_obstacle_detector.log
```

## Performance Optimization

### Resource Limits

Adjust resource limits in `docker-compose.yml`:

```yaml
deploy:
  resources:
    limits:
      memory: 4G
      cpus: '2.0'
    reservations:
      memory: 2G
      cpus: '1.0'
```

### GPU Memory

For better GPU performance:

```bash
# Set GPU memory fraction
export CUDA_VISIBLE_DEVICES=0
export CUDA_MEMORY_FRACTION=0.8
```

## Development

### Building for Development

```bash
# Build with development tools
docker build --target development -t zed_obstacle_detector:dev .

# Run development container
docker run -it --rm \
  --runtime=nvidia \
  --network=host \
  --privileged \
  -v $(pwd):/workspace \
  zed_obstacle_detector:dev
```

### Testing

```bash
# Run tests
docker-compose exec zed_obstacle_detector \
  bash -c "cd /catkin_ws && catkin_make run_tests"

# Check test results
docker-compose exec zed_obstacle_detector \
  bash -c "cd /catkin_ws && catkin_test_results"
```

## Monitoring

### Health Checks

The container includes health checks:

```bash
# Check container health
docker-compose ps

# View health check logs
docker inspect zed_obstacle_detector | grep -A 10 Health
```

### Metrics

Monitor resource usage:

```bash
# Container stats
docker stats zed_obstacle_detector

# GPU usage
nvidia-smi
```

## Support

For issues and questions:

1. Check the troubleshooting section above
2. Review ROS and ZED camera documentation
3. Check container logs: `docker-compose logs zed_obstacle_detector`
4. Contact maintainer: abdulrahman.hany003@gmail.com

## License

This Docker setup is part of the ROAR Autonomous System project and follows the same license as the main project. 