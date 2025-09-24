#!/bin/bash

# Default values
IMAGE_NAME="qemu-aarch64"
IMAGE_TAG="latest"
QEMU_VERSION="9.2.2"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --tag)
      IMAGE_TAG="$2"
      shift 2
      ;;
    --version)
      QEMU_VERSION="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Function to build for a specific platform
build_for_platform() {
    local platform=$1
    local platform_suffix=$(echo $platform | sed 's/\//-/g')
    local build_args="--build-arg QEMU_VERSION=${QEMU_VERSION}"

    echo "Building for platform: ${platform}"
    docker buildx build \
        --platform ${platform} \
        ${build_args} \
        -t "${IMAGE_NAME}:${IMAGE_TAG}-${platform_suffix}" \
        --load \
        -f docker/Dockerfile \
        .
}

docker buildx create --name qemu-builder --use

# Build for both architectures
# build_for_platform "linux/amd64"
build_for_platform "linux/arm64"

# Create a multi-arch manifest
docker buildx imagetools create \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    "${IMAGE_NAME}:${IMAGE_TAG}-linux-amd64" \
    "${IMAGE_NAME}:${IMAGE_TAG}-linux-arm64"

echo "Build complete! You can now use:"
echo "docker pull ${IMAGE_NAME}:${IMAGE_TAG}"