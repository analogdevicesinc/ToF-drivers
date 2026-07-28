#!/bin/bash

# Build Docker image for ToF-drivers tests
# Usage: ./build_docker.sh [image-name] [options]
#
# Options:
#   --ci                    Pre-build tests in image (CI/CD mode)
#   --git-repo <url>        Clone from git repository
#   --git-branch <branch>   Git branch to clone (default: main)
#   --build-jobs <n>        Number of parallel build jobs (default: auto)
#
# Build Modes:
#   1. Local source (default):     ./build_docker.sh
#   2. CI/CD mode:                 ./build_docker.sh --ci
#   3. Git clone:                  ./build_docker.sh --git-repo <url> --git-branch <branch>
#   4. Git clone + CI/CD:          ./build_docker.sh --ci --git-repo <url>
#
# Examples:
#   ./build_docker.sh
#   ./build_docker.sh my-image --ci
#   ./build_docker.sh --git-repo https://github.com/analogdevicesinc/ToF.git
#   ./build_docker.sh --git-repo <url> --git-branch feature/test --ci

set -e

IMAGE_NAME="tof-driver-tests"
PREBUILD_TESTS="OFF"
GIT_REPO=""
GIT_BRANCH="main"
BUILD_JOBS=$(nproc 2>/dev/null || echo 4)
BUILD_TYPE="Local Development"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --ci)
            PREBUILD_TESTS="ON"
            shift
            ;;
        --git-repo)
            GIT_REPO="$2"
            shift 2
            ;;
        --git-branch)
            GIT_BRANCH="$2"
            shift 2
            ;;
        --build-jobs)
            BUILD_JOBS="$2"
            shift 2
            ;;
        -*)
            echo "Unknown option: $1"
            grep "^#" "$0" | grep -v "^#!/" | sed 's/^# //'
            exit 1
            ;;
        *)
            IMAGE_NAME="$1"
            shift
            ;;
    esac
done

# Determine build type
if [[ -n "${GIT_REPO}" ]]; then
    if [[ "${PREBUILD_TESTS}" == "ON" ]]; then
        BUILD_TYPE="Git Clone + CI/CD (pre-built)"
    else
        BUILD_TYPE="Git Clone (tests built at runtime)"
    fi
else
    if [[ "${PREBUILD_TESTS}" == "ON" ]]; then
        BUILD_TYPE="CI/CD (pre-built)"
    else
        BUILD_TYPE="Local Development"
    fi
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../" && pwd)"

echo "============================================"
echo "Building ToF-drivers Test Docker Image"
echo "============================================"
echo "Build type: ${BUILD_TYPE}"
echo "Image name: ${IMAGE_NAME}"
echo "Prebuild tests: ${PREBUILD_TESTS}"
echo "Build jobs: ${BUILD_JOBS}"
if [[ -n "${GIT_REPO}" ]]; then
    echo "Git repository: ${GIT_REPO}"
    echo "Git branch: ${GIT_BRANCH}"
fi
echo "Project root: ${PROJECT_ROOT}"
echo ""

# Build Docker image from project root (for COPY context)
cd "${PROJECT_ROOT}"

# Build command with dynamic args
BUILD_ARGS=(
    --build-arg PREBUILD_TESTS="${PREBUILD_TESTS}"
    --build-arg BUILD_JOBS="${BUILD_JOBS}"
)

if [[ -n "${GIT_REPO}" ]]; then
    BUILD_ARGS+=(
        --build-arg GIT_REPO="${GIT_REPO}"
        --build-arg GIT_BRANCH="${GIT_BRANCH}"
    )
fi

docker build \
    "${BUILD_ARGS[@]}" \
    -t "${IMAGE_NAME}:latest" \
    -f ToF-drivers/tests/docker/Dockerfile.local \
    .

echo ""
echo "============================================"
echo "Build complete!"
echo "Image: ${IMAGE_NAME}:latest"
echo "Type: ${BUILD_TYPE}"
echo ""
if [[ -n "${GIT_REPO}" ]]; then
    echo "Source cloned from: ${GIT_REPO} (${GIT_BRANCH})"
elif [[ "${PREBUILD_TESTS}" == "OFF" ]]; then
    echo "Local development mode: source will be mounted at runtime"
else
    echo "CI/CD mode: tests pre-built in image"
fi
echo ""
echo "Run tests with:"
if [[ -n "${GIT_REPO}" ]] || [[ "${PREBUILD_TESTS}" == "ON" ]]; then
    echo "  docker run --rm ${IMAGE_NAME}:latest"
else
    echo "  ./run_docker.sh"
    echo "  ./run_csv_tests.sh"
    echo "  ./run_all.sh"
fi
echo "============================================"
