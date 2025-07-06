#!/usr/bin/env bash

set -e

IMAGE_NAME="ghcr.io/$(echo "${GITHUB_REPOSITORY:-mowglifrenchtouch/open_mower_jeremy}" | tr '[:upper:]' '[:lower:]')"
DOCKERFILE="docker/Dockerfile"
VERSION="v1.0.$(date +%Y%m%d)"
PLATFORMS="linux/amd64,linux/arm64"
BUILD_DIR="."
USE_CACHE=true

# Optional: override with env
TAG_BUN="bun"

echo "🔧 Building image: $IMAGE_NAME"
echo "📦 Dockerfile: $DOCKERFILE"
echo "🏷️ Tags: latest, $VERSION, $TAG_BUN"
echo "📁 Context: $BUILD_DIR"
echo "📐 Platforms: $PLATFORMS"
echo "🧠 Cache: ${USE_CACHE}"

# Ensure buildx builder exists
docker buildx inspect multiarch-builder >/dev/null 2>&1 || docker buildx create --name multiarch-builder --use --bootstrap
docker buildx use multiarch-builder

# Login to GHCR (if using in CI)
if [ -n "$GHCR_TOKEN" ]; then
    echo "$GHCR_TOKEN" | docker login ghcr.io -u "$GHCR_USER" --password-stdin
fi

docker buildx build \
    --platform "$PLATFORMS" \
    --file "$DOCKERFILE" \
    --tag "$IMAGE_NAME:latest" \
    --tag "$IMAGE_NAME:$VERSION" \
    --tag "$IMAGE_NAME:$TAG_BUN" \
    --push \
    --build-arg BUILDKIT_INLINE_CACHE=1 \
    --cache-from type=gha,scope=openmower \
    --cache-to type=gha,scope=openmower,mode=max \
    "$BUILD_DIR"
