# Development

[Back to README](../README.md)

## Building without Docker

You need:

- ROS 2 Humble or Jazzy,
- Rust from [rustup](https://rustup.rs) (the crates use edition 2024, Ubuntu's
  rustc is too old),
- the dependencies from `package.xml`.

```bash
mkdir -p ~/r3v_ws/src && cd ~/r3v_ws/src
git clone --recurse-submodules https://github.com/igorosky/r3v-ros.git r3v_ros
cd ~/r3v_ws
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The Rust crates are built from the `r3v-voxelizer` and `r3v-renderer`
submodules by cargo during the CMake build (`cmake/r3v_rust.cmake`, with
`--locked`). `-DR3V_CARGO_TARGET_DIR=<dir>` keeps the cargo target directory
between clean builds.

## Code style and tests

C++ code is checked with cpplint ([CPPLINT.cfg](../CPPLINT.cfg)) and
uncrustify ([uncrustify.cfg](../uncrustify.cfg)), and the voxel stream payload
has unit tests ([test](../test), no GPU needed). All of them run as tests:

```bash
colcon test --packages-select r3v_ros && colcon test-result --verbose
```

Or the linters directly, from the repository root:

```bash
ament_cpplint src include test
ament_uncrustify -c uncrustify.cfg src include test  # --reformat to fix
```

The code style is the one of Humble's uncrustify (0.72). Newer versions format
some constructs differently.

Our targets are compiled with `-Wall -Wextra -Wpedantic -Wconversion
-Wsign-conversion -Werror`, so narrowing conversions need an explicit
`static_cast`. The code has to build on both Humble and Jazzy.

## CI

GitHub Actions, [.github/workflows/ci.yml](../.github/workflows/ci.yml):

| Job | What it does |
|---|---|
| `lint` | cpplint and uncrustify on Humble, without building |
| `build` | Build and `colcon test` on Humble and Jazzy, warnings are errors. The uncrustify test runs on Humble only. |
| `docker` | Builds the Docker image for Humble and Jazzy (not pushed) |
