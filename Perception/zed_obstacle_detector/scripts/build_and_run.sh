#!/bin/bash

# ZED Obstacle Detector - Docker Build and Run Script
# This script helps you build and run the ZED obstacle detector in Docker

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check Docker installation
check_docker() {
    if ! command_exists docker; then
        print_error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker is not running or you don't have permissions. Please start Docker and ensure you're in the docker group."
        exit 1
    fi
    
    print_success "Docker is installed and running"
}

# Function to check NVIDIA Docker
check_nvidia_docker() {
    if ! command_exists nvidia-docker; then
        print_warning "NVIDIA Docker not found. GPU acceleration will not be available."
        return 1
    fi
    
    if ! docker run --rm --gpus all nvidia/cuda:11.4-base-ubuntu20.04 nvidia-smi >/dev/null 2>&1; then
        print_warning "NVIDIA Docker test failed. GPU acceleration may not work."
        return 1
    fi
    
    print_success "NVIDIA Docker is working"
    return 0
}

# Function to create necessary directories
create_directories() {
    print_status "Creating necessary directories..."
    mkdir -p config logs rviz_config
    print_success "Directories created"
}

# Function to build the Docker image
build_image() {
    print_status "Building Docker image..."
    
    if [ "$1" = "--no-cache" ]; then
        print_status "Building without cache..."
        docker-compose build --no-cache
    else
        docker-compose build
    fi
    
    if [ $? -eq 0 ]; then
        print_success "Docker image built successfully"
    else
        print_error "Failed to build Docker image"
        exit 1
    fi
}

# Function to run the container
run_container() {
    local mode=$1
    
    print_status "Starting ZED Obstacle Detector..."
    
    case $mode in
        "basic")
            docker-compose up
            ;;
        "background")
            docker-compose up -d
            print_success "Container started in background"
            print_status "View logs with: docker-compose logs -f zed_obstacle_detector"
            ;;
        "ros-master")
            docker-compose --profile ros-master up
            ;;
        "visualization")
            docker-compose --profile visualization up
            ;;
        *)
            print_error "Unknown mode: $mode"
            print_status "Available modes: basic, background, ros-master, visualization"
            exit 1
            ;;
    esac
}

# Function to stop the container
stop_container() {
    print_status "Stopping containers..."
    docker-compose down
    print_success "Containers stopped"
}

# Function to clean up
cleanup() {
    print_status "Cleaning up Docker resources..."
    docker-compose down --rmi all --volumes --remove-orphans
    print_success "Cleanup completed"
}

# Function to show logs
show_logs() {
    print_status "Showing logs..."
    docker-compose logs -f zed_obstacle_detector
}

# Function to enter container
enter_container() {
    print_status "Entering container..."
    docker-compose exec zed_obstacle_detector bash
}

# Function to show help
show_help() {
    echo "ZED Obstacle Detector - Docker Build and Run Script"
    echo ""
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  build [--no-cache]    Build the Docker image"
    echo "  run [MODE]            Run the container"
    echo "  stop                  Stop the container"
    echo "  logs                  Show container logs"
    echo "  shell                 Enter the container shell"
    echo "  cleanup               Clean up Docker resources"
    echo "  check                 Check system requirements"
    echo "  help                  Show this help message"
    echo ""
    echo "Run modes:"
    echo "  basic                 Run in foreground (default)"
    echo "  background            Run in background"
    echo "  ros-master            Run with ROS master"
    echo "  visualization         Run with RViz visualization"
    echo ""
    echo "Examples:"
    echo "  $0 build              Build the image"
    echo "  $0 build --no-cache   Build without cache"
    echo "  $0 run basic          Run in foreground"
    echo "  $0 run background     Run in background"
    echo "  $0 logs               Show logs"
    echo "  $0 shell              Enter container"
}

# Main script logic
main() {
    local command=$1
    local option=$2
    
    case $command in
        "build")
            check_docker
            create_directories
            build_image $option
            ;;
        "run")
            check_docker
            create_directories
            run_container ${option:-basic}
            ;;
        "stop")
            stop_container
            ;;
        "logs")
            show_logs
            ;;
        "shell")
            enter_container
            ;;
        "cleanup")
            cleanup
            ;;
        "check")
            check_docker
            check_nvidia_docker
            create_directories
            print_success "System check completed"
            ;;
        "help"|"--help"|"-h")
            show_help
            ;;
        "")
            print_error "No command specified"
            show_help
            exit 1
            ;;
        *)
            print_error "Unknown command: $command"
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@" 