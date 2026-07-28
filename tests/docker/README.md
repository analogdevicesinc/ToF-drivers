# ToF-drivers Docker Test Environment

Docker containerization for ToF-drivers system tests enables consistent testing across environments and CI/CD integration.

## Quick Start

### Build Docker Image

**Local Development** (default - source mounted at runtime):
```bash
cd ToF-drivers/tests/docker
./build_docker.sh
```

**CI/CD Build** (pre-builds tests in image):
```bash
./build_docker.sh tof-driver-tests --ci
```

**Git Clone Mode** (clone from repository):
```bash
./build_docker.sh --git-repo https://github.com/analogdevicesinc/ToF.git --git-branch main
```

Single unified **Dockerfile.local** with build arguments:
- `PREBUILD_TESTS=OFF` (default): Local development mode
- `PREBUILD_TESTS=ON` (--ci flag): CI/CD pre-built mode
- `GIT_REPO` (optional): Clone source from git repository
- `GIT_BRANCH` (default: main): Branch to clone

📖 **See [GIT_CLONE_SUPPORT.md](GIT_CLONE_SUPPORT.md) for detailed git integration guide**

Image includes:
- Ubuntu 20.04 base (Jetson Orin Nano compatible)
- GoogleTest framework
- V4L2 and I2C utilities
- Development tools (gdb, strace, vim, htop)
- Python tools (pytest, pandas, matplotlib)

### Run Tests

**Local Development** (builds tests in container from mounted source):
```bash
# Build and run tests
./run_docker.sh --build

# Interactive development shell
./run_docker.sh --interactive

# Specify custom devices
./run_docker.sh --build --device /dev/video2 --i2c /dev/i2c-8
```

**CI/CD Mode** (uses pre-built binaries):
```bash
# Run pre-built tests (no source mount)
./run_docker.sh --no-mount

# Run specific test filter
./run_docker.sh --no-mount --filter "*Capability*"

# Repeat tests
./run_docker.sh --no-mount --repeat 5
```

### Docker Compose

**Local Development** (builds tests from mounted source):
```bash
# Build and run all tests
docker-compose up

# Run specific test suite
docker-compose up v4l2-tests
docker-compose up i2c-tests

# Interactive shell
docker-compose run --rm tof-driver-tests /bin/bash

# Clean up
docker-compose down
```

## Image Details

### Unified Dockerfile.local with Build Arguments

Single Dockerfile.local handles both use cases via `--build-arg PREBUILD_TESTS=ON|OFF`:

**Local Development Mode** (`PREBUILD_TESTS=OFF`, default):
- Source code mounted from host at runtime
- Tests built on-demand in container
- Includes development tools (gdb, strace, vim, htop)
- Interactive bash shell by default
- Fast iteration with live code changes

**CI/CD Pre-built Mode** (`PREBUILD_TESTS=ON`, --ci flag):
- Tests pre-built during image creation
- Self-contained image (no source dependencies)
- Faster test execution
- Smaller runtime footprint
- Runs tests by default

### Base Image
- **nvcr.io/nvidia/l4t-base:r35.2.0** - NVIDIA Jetson Linux for Tegra
- Compatible with Jetson Orin Nano (JetPack 5.1+)

### Installed Packages
- **Build tools**: cmake, g++, make
- **GoogleTest**: Testing framework
- **Kernel headers**: V4L2 and I2C definitions
- **Debug tools**: v4l-utils, i2c-tools, gdb, strace, ltrace
- **Python**: pytest, pandas, matplotlib
- **Utilities**: vim, nano, htop, tree

### Directory Structure
```
/workspace/
├── ToF-drivers/tests/
│   ├── build/          # Build directory (local or pre-built)
│   ├── system/         # Test source
│   └── utilities/      # Test utilities
├── test-results/       # JSON test results (mounted)
└── test-logs/          # Test logs (mounted)
```

## Build Options

### Custom Image Name

```bash
# Local development image
./build_docker.sh my-custom-name

# CI/CD pre-built image
./build_docker.sh my-custom-name --ci
```

### Manual Build

**Local development**:
```bash
# From project root (ADCAM/)
docker build \
  --build-arg PREBUILD_TESTS=OFF \
  -t tof-driver-tests:latest \
  -f ToF-drivers/tests/docker/Dockerfile.local .
```

**CI/CD pre-built**:
```bash
# From project root (ADCAM/)
docker build \
  --build-arg PREBUILD_TESTS=ON \
  --build-arg BUILD_JOBS=8 \
  -t tof-driver-tests:latest \
  -f ToF-drivers/tests/docker/Dockerfile.local .
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `PREBUILD_TESTS` | `OFF` | `ON`: Pre-build tests (CI/CD)<br>`OFF`: Build at runtime (local dev) |
| `BUILD_JOBS` | `4` | Number of parallel build jobs |

## Running Tests

### Local Development Mode

**Build and run tests** (source mounted from host):
```bash
./run_docker.sh --build

# Or manually:
docker run --rm --privileged \
  --device=/dev/video0 \
  --device=/dev/i2c-1 \
  -v $(pwd)/../:/workspace/ToF-drivers/tests \
  tof-driver-tests:latest \
  /bin/bash -c "cd /workspace/ToF-drivers/tests && mkdir -p build && cd build && cmake .. && make -j && ./driver-v4l2_test"
```

### CI/CD Mode (Pre-built)

```bash
./run_docker.sh --no-mount

# Or manually:
docker run --rm --privileged \
  --device=/dev/video0 \
  --device=/dev/i2c-1 \
  tof-driver-tests:latest
```

### Custom Devices

```bash
docker run --rm --privileged \
  --device=/dev/video2 \
  --device=/dev/i2c-8 \
  tof-driver-tests:latest \
  /bin/bash -c "cd /workspace/ToF-drivers/tests/build && \
    ./driver-v4l2_test --device=/dev/video2 && \
    ./driver-i2c_test --device=/dev/i2c-8"
```

### GoogleTest Options

```bash
# Run with filter
docker run --rm --privileged \
  --device=/dev/video0 \
  --device=/dev/i2c-1 \
  tof-driver-tests:latest \
  /bin/bash -c "cd /workspace/ToF-drivers/tests/build && ./driver-v4l2_test --gtest_filter=*Enumeration*"

# Repeat tests
docker run --rm --privileged \
  --device=/dev/video0 \
  --device=/dev/i2c-1 \
  tof-driver-tests:latest \
  /bin/bash -c "cd /workspace/ToF-drivers/tests/build && ./driver-v4l2_test --gtest_repeat=10"
```

### Save Test Results

```bash
# Mount volume for results
docker run --rm --privileged \
  --device=/dev/video0 \
  --device=/dev/i2c-1 \
  -v $(pwd)/test-results:/workspace/test-results \
  -e GTEST_OUTPUT=json:/workspace/test-results/results.json \
  tof-driver-tests:latest
```

### Interactive Shell

```bash
# Debug inside container
docker run --rm --privileged -it \
  --device=/dev/video0 \
  --device=/dev/i2c-1 \
  tof-driver-tests:latest \
  /bin/bash

# Inside container:
cd /workspace/ToF-drivers/tests/build
./driver-v4l2_test --gtest_list_tests
v4l2-ctl --list-devices
i2cdetect -l
```

## Docker Compose Usage

### Configuration

Edit `docker-compose.yml` to customize:

```yaml
services:
  tof-driver-tests:
    build:
      args:
        PREBUILD_TESTS: "OFF"  # Local dev (default)
        BUILD_JOBS: 8
    devices:
      - /dev/video2:/dev/video2  # Change video device
      - /dev/i2c-8:/dev/i2c-8    # Change I2C device
    
    environment:
      - GTEST_REPEAT=3            # Add repeat count
```

### Running Services

```bash
# All tests
docker-compose up

# Specific suite
docker-compose up v4l2-tests

# Background mode
docker-compose up -d

# View logs
docker-compose logs -f

# Stop
docker-compose down
```

### Development Mode (Default)

Source is automatically mounted by default in `docker-compose.yml`:

```yaml
volumes:
  - ../:/workspace/ToF-drivers/tests  # Already configured
```

Interactive development workflow:

```bash
# Start interactive shell
docker-compose run --rm tof-driver-tests /bin/bash

# Inside container:
cd /workspace/ToF-drivers/tests
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./driver-v4l2_test --gtest_filter=*Enumeration*

# Edit code on host, then rebuild in container:
make -j$(nproc)
./driver-v4l2_test
```

**Workflow**:
1. Edit source files on host (with your IDE)
2. Build in container (with correct dependencies)
3. Run tests in container (with hardware access)
4. Iterate quickly

## CI/CD Integration

### Azure Pipelines

**Use CI/CD mode with build arg**:

```yaml
- task: Docker@2
  displayName: 'Build test image'
  inputs:
    command: build
    dockerfile: ToF-drivers/tests/docker/Dockerfile.local
    arguments: '--build-arg PREBUILD_TESTS=ON --build-arg BUILD_JOBS=8'
    tags: tof-driver-tests:$(Build.BuildId)

- task: Bash@3
  displayName: 'Run tests'
  inputs:
    targetType: 'inline'
    script: |
      docker run --rm --privileged \
        --device=/dev/video0 \
        --device=/dev/i2c-1 \
        -v $(Build.ArtifactStagingDirectory):/workspace/test-results \
        tof-driver-tests:$(Build.BuildId)

- task: PublishTestResults@2
  inputs:
    testResultsFormat: 'JUnit'
    testResultsFiles: '$(Build.ArtifactStagingDirectory)/*.json'
```

### GitHub Actions

**Use CI/CD mode with build arg**:

```yaml
- name: Build test image
  run: |
    docker build \
      --build-arg PREBUILD_TESTS=ON \
      --build-arg BUILD_JOBS=8 \
      -t tof-driver-tests:latest \
      -f ToF-drivers/tests/docker/Dockerfile.local \
      .

- name: Run tests
  run: |
    docker run --rm --privileged \
      --device=/dev/video0 \
      --device=/dev/i2c-1 \
      -v ${{ github.workspace }}/test-results:/workspace/test-results \
      tof-driver-tests:latest

- name: Upload results
  uses: actions/upload-artifact@v3
  with:
    name: test-results
    path: test-results/
```

## Troubleshooting

### "Cannot open device" Errors

**Symptom**: Tests fail with permission denied or device not found

**Solutions**:
1. Verify devices exist on host: `ls -l /dev/video* /dev/i2c-*`
2. Ensure `--privileged` flag is used
3. Check device paths in docker run command match host devices
4. On some systems, add `--device-cgroup-rule='c 81:* rmw'` for V4L2

### Tests Skip with "Hardware not present"

**Expected behavior** - Tests detect missing hardware and skip gracefully.

Run with `--interactive` to debug:

```bash
./run_docker.sh --interactive

# Inside container:
v4l2-ctl --list-devices
i2cdetect -y 1
```

### Build Failures

**Symptom**: GoogleTest not found

**Solution**: Ensure GoogleTest builds in Dockerfile:
```dockerfile
RUN cd /usr/src/googletest/googletest && \
    cmake CMakeLists.txt && \
    make && \
    cp lib/*.a /usr/lib/
```

**Symptom**: Cannot find ToF-drivers directory

**Solution**: Build from project root (ADCAM/), not from docker/ subdirectory.

### Performance Issues

**Symptom**: Tests run slowly in container

**Solutions**:
1. Use `--network host` for faster I/O
2. Increase Docker resources (CPU/memory)
3. Run on bare metal for hardware-intensive tests

## Reference

- [Dockerfile.local](Dockerfile.local) - Unified image (local dev + CI/CD)
- [build_docker.sh](build_docker.sh) - Build script with --ci flag
- [run_docker.sh](run_docker.sh) - Run script (auto-detects mode)
- [docker-compose.yml](docker-compose.yml) - Compose configuration
- [Parent README](../README.md) - Main test documentation

## Best Practices

### Local Development
1. **Use default mode** - `./build_docker.sh` (no --ci flag)
2. **Interactive shell** - `./run_docker.sh --interactive` for debugging
3. **Build in container** - `./run_docker.sh --build` for clean builds
4. **Edit on host, build in container** - Best of both worlds

### CI/CD
1. **Use --ci flag** - `./build_docker.sh tof-tests --ci` for pre-built tests
2. **Or use build-arg** - `--build-arg PREBUILD_TESTS=ON`
3. **No source mount** - Self-contained image
4. **Cache images** - Reuse built images across pipeline runs
5. **Parallel execution** - Run multiple test suites simultaneously

### General
1. **Single Dockerfile.local** - Unified image with build arguments
2. **Use run_docker.sh** - Handles devices and volumes automatically
3. **Mount results** - Always save test results to host volume
4. **Check devices first** - Verify hardware availability before running
5. **Docker isolation** - Ensures consistent test environment across platforms
