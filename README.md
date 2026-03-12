This code is responsible for servo control. It is designed to work with ST3215 and ST3025 bus servos.

## Cloning

This repository contains a Git submodule (`docs-theme/doxygen-awesome-css`).
Clone with submodules in one step:

```bash
git clone --recurse-submodules https://github.com/BETA-KNH/beta-servo-sensei.git
```

If you already cloned without `--recurse-submodules`, initialise it afterwards:

```bash
git submodule update --init
```

## Building

### Dependencies

**Required:**
- CMake 3.20 or newer
- C++17 compiler (GCC, Clang)

**Optional:**
- YARP 3.x (for YARP motor control plugin)
- Doxygen (for documentation generation)

### Basic Build

```bash
cd beta-servo-sensei
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

This builds:
- `scan_servos` - Scan and detect servos on the bus
- `servo_cli` - Command-line interface for servo control
- `servo_tests` - Unit tests (no hardware required)

### Building with YARP Plugin

If YARP is installed on your system:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
cmake --build build
```

This additionally builds the `st_servo` YARP device plugin for integration with YARP robot control frameworks.

### Running Tests

```bash
cd build
ctest --output-on-failure
```
or
```bash
./build/servo_tests
```

### Installation

```bash
sudo cmake --install build
```

Writes:
- Binaries to `/usr/local/bin`
- YARP plugin (if built) to `/usr/local/lib/yarp`
- Configuration samples to `/usr/local/etc/servo_sensei`

### Building Documentation

```bash
cmake --build build --target docs
```
