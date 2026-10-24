# r3v_ros

ROS 2 node (Humble and Jazzy) that renders a robot and a voxel map built from
point clouds, and publishes the result as a video stream.

- Robot: URDF from `/robot_description` (or a file), posed from `/tf` and
  `/tf_static`. Meshes in any format assimp reads (STL, DAE, OBJ, glTF, ...)
  plus box, cylinder and sphere primitives.
- Point clouds: any number (0..N) of `sensor_msgs/PointCloud2` topics, ray cast
  into an occupancy voxel map by [r3v-voxelizer](r3v-voxelizer).
- Output: `sensor_msgs/Image` through `image_transport`, so `<topic>/compressed`
  (and every other installed transport) is published too.
- Rendering: [r3v-renderer](r3v-renderer), headless on the GPU through
  Vulkan (or on the CPU with lavapipe).

Everything is set in a ROS parameter file: see [config/r3v.yaml](config/r3v.yaml),
which documents every parameter.

## Docker

The submodules have to be checked out before building:

```bash
git submodule update --init --recursive
docker build -t r3v-ros:jazzy .
docker build -t r3v-ros:humble --build-arg ROS_DISTRO=humble .
```

`RUST_TOOLCHAIN` (default `stable`) pins the Rust toolchain used for the
r3v crates.

The first build takes several minutes, mostly compiling the Rust crates (wgpu).
Their cargo target directory lives in a BuildKit cache (one per distro), so
later builds only recompile what changed. `--target deps` builds just the
build dependencies, handy as a development image.

Run with your own configuration:

```bash
docker run --rm -it --net=host --ipc=host --device /dev/dri \
  -v $PWD/my_config.yaml:/config/r3v.yaml:ro \
  r3v-ros:jazzy
```

or with compose: `docker compose -f docker/compose.yaml up --build`.

GPU access:

- Intel / AMD: pass `--device /dev/dri` (Mesa Vulkan drivers are in the image).
- NVIDIA: install the NVIDIA container toolkit and pass `--gpus all`
  (the image sets `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility`).
- No GPU: the image falls back to lavapipe (software Vulkan). Works, but slow
  for big frames.

Meshes referenced as `package://<pkg>/...` must be found in the container.
Either install the description package into a derived image, or mount the
directory holding it and list it in `robot_description.package_paths`:

```bash
docker run ... -v /path/to/src:/robot:ro r3v-ros:jazzy
# config: robot_description.package_paths: ["/robot"]  ->  /robot/<pkg>/meshes/...
```

## Topics

| Topic | Type | Direction |
|---|---|---|
| `tf_topic` (`/tf`), `tf_static_topic` (`/tf_static`) | `tf2_msgs/TFMessage` | in |
| `robot_description.topic` (`/robot_description`) | `std_msgs/String` (transient local) | in |
| `point_clouds.<name>.topic` | `sensor_msgs/PointCloud2` | in |
| `camera.pose_topic` (`~/camera_pose`) | `geometry_msgs/PoseStamped` | in |
| `~/hidden_links` | `std_msgs/String` | in |
| `~/voxels_visible` | `std_msgs/Bool` | in |
| `~/clear_voxels` | `std_msgs/Empty` | in |
| `output.topic` (`/r3v/image`) + transports | `sensor_msgs/Image` | out |

The camera is placed by `camera.position` / `camera.target`, given in
`camera.frame` (the fixed frame by default; a robot frame makes it follow the
robot). A pose on `camera.pose_topic` replaces them: the camera sits at the
pose and looks along its +X axis with +Z up.

Three topics change the scene at runtime. Each message replaces the previous
state rather than adding to it, so publishing the same one again is harmless:

- `~/hidden_links`: comma or space separated link names. Every listed link
  is hidden together with all links below it in the URDF tree (the arm's base
  link hides the whole arm); an empty string shows everything again. Starts
  from `robot_description.hidden_links` and survives a new robot description.
- `~/voxels_visible`: hides or shows the voxel map. Point clouds keep
  building it while it is hidden. Starts from `voxelizer.visible`.
- `~/clear_voxels`: forgets the whole voxel map.

With `output.lazy` (default) nothing is rendered while nobody subscribes.
Frames are stamped with the time their scene was sampled. The renderer reads
frames back one call late, so images lag one period (`1 / rate`).

## Notes

- The node needs a multi-threaded executor (point clouds wait for TF). The
  `r3v_node` executable uses one; as a component (`r3v_ros::R3vNode`) load it
  into `component_container_mt`.
- Robot visuals are drawn without URDF colors (the renderer shades meshes
  white); voxels are colored by distance from the camera.
- Voxels live in the fixed frame and accumulate over time; rays clear voxels
  they pass through.
- A renderer failure shuts the node down so a supervisor (e.g. compose
  `restart: unless-stopped`) can restart it.

## Building without Docker

Needs a ROS 2 install, Rust (via rustup, edition 2024 capable) and the
dependencies from `package.xml` (`rosdep install --from-paths . --ignore-src`).
Clone into a colcon workspace and run `colcon build`.
