# senior-design

Repository for Group D9's Senior Design Lidar Project

## Build Instructions

This repository uses CMake with the [Point Cloud Library](https://pointclouds.org/).

Detailed build instructions are located in [docs/BUILD.md](./docs/BUILD.md).

### Quickstart

> Make sure you have [Git](https://git-scm.com), [CMake](https://cmake.org), and [vcpkg](https://vcpkg.io) installed.

```sh
git clone https://github.com/BobbyChiu/senior-design.git sd-2d-lidar-scanner
cd sd-2d-lidar-scanner
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg-root>/scripts/buildsystems/vcpkg.cmake
cmake --build build
```
