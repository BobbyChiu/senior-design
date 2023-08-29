# Build instructions for this repository

## Prerequisites

Please make sure you have the following installed:

- [Git](https://git-scm.com): should be accessible by terminal
- [CMake](https://cmake.org): should be accessible by terminal
- [any generator supported by CMake](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html): see below for information
  - For Windows: MinGW, Visual Studio, or Unix make under WSL are solid choices.
- [vcpkg](https://vcpkg.io): cloned to a known directory and bootstrap script ran (steps 1 and 2 in https://vcpkg.io/en/getting-started.html)

In addition, building requires a fair amount of time and disk space.

## 1. Clone the Repo

> **Make sure that the full path of the local directory does not contain any whitespace.** \
> libiconv is known to fail when trying to build in a path with whitespaces.

Type the following into your terminal:

```sh
git clone https://github.com/BobbyChiu/senior-design.git
cd senior-design
```

> Note: You can specify the local directory if you wish.

### 1.1 Determine your generator for CMake

The configuration options are different between a single configuration generator and a multi-configuration generator.

Type the following into your terminal:

```sh
cmake --help
```

There should be a star (*) that appears next to your default generator. However, you can manually choose your generator with `-G` in the configure and generate step.

> **Note which type of generator you are using.** \
> Makefile generators (like MinGW) tend to be single configuration.
> IDE generators (like Visual Studio) tend to be multi-configuration.

After you figured out your generator type, go to the correct section, either [single](#single-2-configure-and-generate) or [multi](#multi-2-configure-and-generate).

## Single: 2. Configure and Generate

Type the following into your terminal, replacing `<path-to-vcpkg-root>` accordingly:

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg-root>/scripts/buildsystems/vcpkg.cmake
# Or `cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg-root>/scripts/buildsystems/vcpkg.cmake`
```

> Note: You can specify the build directory and CMAKE_BUILD_TYPE (which is typically default to DEBUG).

## Single: 3. Build

Type the following into your terminal:

```sh
cmake --build build
# Or `cmake --build build --target <target>`
```

> Note: You can specify the target (which is typically default to all).

Alternatively, you can use your build tool manually, such as `make`.

## Multi: 2. Configure and Generate

Type the following into your terminal, replacing `<path-to-vcpkg-root>` accordingly:

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<path-to-vcpkg-root>/scripts/buildsystems/vcpkg.cmake
```

> Note: You can specify the build directory.

## Multi: 3. Build

Open your solution file from inside the build directory and use your IDE to manually build your targets.

Alternatively, you can type the following into your terminal:

```sh
cmake --build build
# Or `cmake --build build --target <target> --config Release`
```

> Note: You can specify the target (which is typically default to all) and the build config (which is typically default to Debug).

# A Note about CMake

CMake is a build system generator. \
Step 2 generates the build system, meaning that you only need to run the command(s) the first time and after you change any CMakeLists.txt. \
Step 3 actually builds the target(s), meaning that you run the command(s) after you change source code.
