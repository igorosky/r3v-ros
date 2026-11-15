# Docker

[Back to README](../README.md)

## Building the image

The submodules have to be checked out before building:

```bash
git submodule update --init --recursive
docker build -t r3v-ros:jazzy .
docker build -t r3v-ros:humble --build-arg ROS_DISTRO=humble .
```

| Build argument / target | Default | Meaning |
|---|---|---|
| `ROS_DISTRO` | `jazzy` | ROS 2 distribution, `humble` or `jazzy` |
| `RUST_TOOLCHAIN` | `stable` | Rust toolchain used for the r3v crates |
| `--target deps` | | Builds only the build dependencies, handy as a development image |

The first build takes several minutes, mostly compiling the Rust crates
(wgpu). Their cargo target directory lives in a BuildKit cache (one per
distro), so later builds only recompile what changed.

## Running

Both nodes in one container (`r3v.launch.py`) with your own configuration:

```bash
docker run --rm -it --net=host --ipc=host --device /dev/dri \
  -v $PWD/my_config.yaml:/config/r3v.yaml:ro \
  r3v-ros:jazzy
```

Without the `-v` option the image uses its copy of
[config/r3v.yaml](../r3v_ros/config/r3v.yaml).

### Compose

[docker/compose.yaml](../docker/compose.yaml) has a service per way of
running:

```bash
docker compose -f docker/compose.yaml up --build    # both nodes
docker compose -f docker/compose.yaml up voxelizer  # voxelizer only (robot)
docker compose -f docker/compose.yaml up renderer   # renderer only (base station)
```

`voxelizer` and `renderer` have profiles (`robot`, `station`) only to keep
them out of a plain `up`. Start them by name, since `--profile robot up` would
start the combined `r3v` service too.

## GPU access

| Hardware | What to pass |
|---|---|
| Intel / AMD | `--device /dev/dri` (Mesa Vulkan drivers are in the image) |
| NVIDIA | Install the NVIDIA container toolkit and pass `--gpus all` (the image sets `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility`) |
| No GPU | Nothing. The image falls back to lavapipe (software Vulkan). Works, but slow for big frames. |

The renderer always needs the GPU. The voxelizer needs it only with
`voxelizer.backend: wgpu` (the default in `r3v_ros/config/r3v.yaml`).

## Robot meshes

Meshes referenced as `package://<pkg>/...` must be found in the container.
Either install the description package into a derived image, or mount the
directory holding it and list it in `robot_description.package_paths`:

```bash
docker run ... -v /path/to/src:/robot:ro r3v-ros:jazzy
# config: robot_description.package_paths: ["/robot"]  ->  /robot/<pkg>/meshes/...
```
