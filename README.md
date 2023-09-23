# senior-design
Repository for Group D9's Senior Design Lidar Project

# Quickstart

Have Git, CMake, and a C++ compiler.

## Cloning

```sh
# If you have not cloned the repo yet:
git clone --recurse-submodules https://github.com/BobbyChiu/senior-design.git
cd senior-design

# If you have cloned the repo without submodules:
git submodule update --init --recursive
```

## Building

```sh
# Configure and generate
cmake -S . -B build

# Build
cmake --build build
```
