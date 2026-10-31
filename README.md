# r3v_ros

ROS 2 nodes (Humble and Jazzy) that build a voxel map from point clouds and
render it together with a robot, publishing the result as a video stream.

The repository holds two packages:

- [r3v_ros](r3v_ros): the nodes below, with the Rust crates they are built on
  as git submodules ([r3v-voxelizer](r3v_ros/r3v-voxelizer),
  [r3v-renderer](r3v_ros/r3v-renderer)).
- [r3v_interfaces](r3v_interfaces): the voxel stream messages only, so that
  whatever only passes the stream on (rosbridge, bridges between machines) or
  renders it elsewhere (e.g. r3v-renderer's JS bindings in a browser) builds
  without Rust.

- `r3v_voxelizer` (`r3v_voxelizer_node`, component `r3v_ros::VoxelizerNode`):
  any number (0..N) of `sensor_msgs/PointCloud2` topics, ray cast into an
  occupancy voxel map by [r3v-voxelizer](r3v_ros/r3v-voxelizer). Publishes the map on
  the voxel stream.
- `r3v_renderer` (`r3v_renderer_node`, component `r3v_ros::RendererNode`):
  follows the voxel stream and renders the map with the robot, URDF from
  `/robot_description` (or a file) posed from `/tf` and `/tf_static`. Meshes
  in any format assimp reads (STL, DAE, OBJ, glTF, ...) plus box, cylinder and
  sphere primitives. Renders headless on the GPU through Vulkan (or on the CPU
  with lavapipe) with [r3v-renderer](r3v_ros/r3v-renderer).
- Output: `sensor_msgs/Image` through `image_transport`, so `<topic>/compressed`
  (and every other installed transport) is published too.

Both nodes are set in one ROS parameter file: see [config/r3v.yaml](r3v_ros/config/r3v.yaml),
which documents every parameter. It has a section read by both nodes and one
per node, so the same file serves every way of running them.

## Running

Everything in one process, from point clouds and TF to images:

```bash
ros2 launch r3v_ros r3v.launch.py config:=/path/to/r3v.yaml
```

It loads both nodes into one `component_container_mt`, where the voxel stream
goes from one to the other without the middleware (and uncompressed;
`compression:=zstd` compresses it for renderers elsewhere following it too).

Split, e.g. the voxelizer on the robot and the renderer on a base station
(same parameter file on both):

```bash
ros2 launch r3v_ros voxelizer.launch.py config:=/path/to/r3v.yaml   # robot
ros2 launch r3v_ros renderer.launch.py config:=/path/to/r3v.yaml    # base station
```

The renderer needs the voxel stream, TF and `robot_description` from the
robot; the voxelizer needs only the point clouds and TF. Any number of
renderers can follow one voxelizer. All launch files take `config`,
`namespace` and `use_sim_time`. The parameter file sections are keyed by the
node names (`r3v_voxelizer`, `r3v_renderer`).

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

Run with your own configuration (both nodes, `r3v.launch.py`):

```bash
docker run --rm -it --net=host --ipc=host --device /dev/dri \
  -v $PWD/my_config.yaml:/config/r3v.yaml:ro \
  r3v-ros:jazzy
```

or with compose ([docker/compose.yaml](docker/compose.yaml)):

```bash
docker compose -f docker/compose.yaml up --build    # both nodes
docker compose -f docker/compose.yaml up voxelizer  # voxelizer only (robot)
docker compose -f docker/compose.yaml up renderer   # renderer only (base station)
```

`voxelizer` and `renderer` have profiles (`robot`, `station`) only to keep
them out of a plain `up`; start them by name, since `--profile robot up` would
start the combined `r3v` service too.

GPU access:

- Intel / AMD: pass `--device /dev/dri` (Mesa Vulkan drivers are in the image).
- NVIDIA: install the NVIDIA container toolkit and pass `--gpus all`
  (the image sets `NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility`).
- No GPU: the image falls back to lavapipe (software Vulkan). Works, but slow
  for big frames.

The renderer needs the GPU, the voxelizer only with `voxelizer.backend: wgpu`
(the default in `r3v_ros/config/r3v.yaml`).

Meshes referenced as `package://<pkg>/...` must be found in the container.
Either install the description package into a derived image, or mount the
directory holding it and list it in `robot_description.package_paths`:

```bash
docker run ... -v /path/to/src:/robot:ro r3v-ros:jazzy
# config: robot_description.package_paths: ["/robot"]  ->  /robot/<pkg>/meshes/...
```

## Topics

`r3v_voxelizer`:

| Topic | Type | Direction |
|---|---|---|
| `tf_topic` (`/tf`), `tf_static_topic` (`/tf_static`) | `tf2_msgs/TFMessage` | in |
| `point_clouds.<name>.topic` | `sensor_msgs/PointCloud2` | in |
| `~/request_full_sync` | `r3v_interfaces/FullSyncRequest` | in |
| `~/clear_voxels` | `std_msgs/Empty` | in |
| `~/set_resolution` | `std_msgs/Float64` | in |
| `~/set_destroy_threshold`, `set_construct_threshold`, `set_max_value` | `std_msgs/UInt32` | in |
| `~/point_clouds/<name>/set_intensity_multiplier` | `std_msgs/Float64` | in |
| `~/point_clouds/<name>/set_hit_multiplier`, `set_target_multiplier` | `std_msgs/UInt32` | in |
| `voxel_stream.topic` (`/r3v/voxels`) | `r3v_interfaces/VoxelMapUpdate` (reliable) | out |

`r3v_renderer`:

| Topic | Type | Direction |
|---|---|---|
| `tf_topic` (`/tf`), `tf_static_topic` (`/tf_static`) | `tf2_msgs/TFMessage` | in |
| `robot_description.topic` (`/robot_description`) | `std_msgs/String` (transient local) | in |
| `voxel_stream.topic` (`/r3v/voxels`) | `r3v_interfaces/VoxelMapUpdate` (reliable) | in |
| `~/set_enabled` | `std_msgs/Bool` | in |
| `camera.pose_topic` (`~/camera_pose`) | `geometry_msgs/PoseStamped` | in |
| `~/hidden_links` | `std_msgs/String` | in |
| `~/voxels_visible` | `std_msgs/Bool` | in |
| `~/marching_cubes/set_enabled` | `std_msgs/Bool` | in |
| `~/background/set_color` | `std_msgs/ColorRGBA` | in |
| `~/colormap/set_depth_range`, `set_color_offset` | `std_msgs/Float64` | in |
| `~/colormap/set_mode` | `std_msgs/String` | in |
| `~/colormap/set_height_relative` | `std_msgs/Bool` | in |
| `~/colormap/set_height_offset`, `set_height_scale` | `std_msgs/Float64` | in |
| `~/colormap/set_height_bands` | `std_msgs/UInt32` | in |
| `voxel_stream.request_topic` (`/r3v_voxelizer/request_full_sync`) | `r3v_interfaces/FullSyncRequest` | out |
| `output.topic` (`/r3v/image`) + transports | `sensor_msgs/Image` (best effort, `output.qos`) | out |

The camera is placed by `camera.position` / `camera.target`, given in
`camera.frame` (the fixed frame by default; a robot frame makes it follow the
robot). A pose on `camera.pose_topic` replaces them: the camera sits at the
pose and looks along its +X axis with +Z up.

These topics change the scene at runtime. Each message replaces the previous
state rather than adding to it, so publishing the same one again is harmless.

`r3v_voxelizer`:

- `~/clear_voxels`: forgets the whole voxel map.
- `~/set_resolution` (`std_msgs/Float64`): voxel edge length in meters,
  positive. Starts from `voxelizer.resolution`.
- `~/set_destroy_threshold`, `~/set_construct_threshold`, `~/set_max_value`
  (`std_msgs/UInt32`): start from the `voxelizer.*` parameters of the same
  name.
- `~/point_clouds/<name>/set_intensity_multiplier` (`std_msgs/Float64`,
  finite), `~/point_clouds/<name>/set_hit_multiplier` and
  `~/point_clouds/<name>/set_target_multiplier` (`std_msgs/UInt32`): the
  voxelizer multipliers of point cloud `<name>` (one of `point_clouds.names`,
  which therefore have to be valid in topic names). Start from
  `point_clouds.<name>.*_multiplier`.

The voxelizer and point cloud settings change how the map is built, so every
change (even of a single point cloud's multiplier, all clouds share the
voxelizer) clears the whole voxel map and starts it over from nothing.
Changes arriving within one `voxel_stream.rate` period clear it once.

`r3v_renderer`:

- `~/set_enabled` (`std_msgs/Bool`): switches the renderer, see below.
  Starts from `enabled`.
- `~/hidden_links`: comma or space separated link names. Every listed link
  is hidden together with all links below it in the URDF tree (the arm's base
  link hides the whole arm); an empty string shows everything again. Starts
  from `robot_description.hidden_links` and survives a new robot description.
- `~/voxels_visible`: hides or shows the voxel map. The voxelizer keeps
  building it while it is hidden. Starts from `voxels.visible`.
- `~/marching_cubes/set_enabled` (`std_msgs/Bool`): switches marching cubes
  smoothing of the voxel surface. Starts from `voxels.marching_cubes`.
- `~/background/set_color` (`std_msgs/ColorRGBA`): background color, sRGB
  components between 0 and 1, alpha ignored. Starts from
  `output.background_color`.
- `~/colormap/set_depth_range` (`std_msgs/Float64`): meters over which the
  voxel colormap runs once, positive. Starts from `camera.color_depth_range`.
- `~/colormap/set_color_offset` (`std_msgs/Float64`): meters added to the
  distance voxels are colored by, finite. Starts from `camera.color_offset`.
- `~/colormap/set_mode` (`std_msgs/String`): `distance` colors voxels by
  their distance from the camera (the two settings above), `height` by their
  height above a neutral height (the settings below). Starts from
  `colormap.mode`.
- `~/colormap/set_height_relative` (`std_msgs/Bool`): the neutral height is
  the height of `colormap.height_frame` (true, following it as it moves) or
  z 0 of the fixed frame (false). Starts from `colormap.height_relative`.
- `~/colormap/set_height_offset` (`std_msgs/Float64`): meters added to the
  neutral height, finite. Starts from `colormap.height_offset`.
- `~/colormap/set_height_scale` (`std_msgs/Float64`): meters from the neutral
  height where colors start changing slower, positive. Starts from
  `colormap.height_scale`.
- `~/colormap/set_height_bands` (`std_msgs/UInt32`): 0 for a continuous
  height colormap, otherwise that many flat color bands, at most 256. Starts
  from `colormap.height_bands`.

Invalid values are logged and ignored. Every setting with a `set_*` topic is
published on the matching `get_*` topic of its node (`r3v_renderer`:
`~/get_enabled`, `~/marching_cubes/get_enabled`, `~/background/get_color`,
`~/colormap/get_depth_range`, `~/colormap/get_mode`, ...; `r3v_voxelizer`: `~/get_resolution`,
`~/point_clouds/<name>/get_hit_multiplier`, ...) with a transient local QoS,
so a late subscriber (with a transient local, reliable QoS too) gets the
current state.

With `output.lazy` (default) nothing is rendered while nobody subscribes.
Frames are stamped with the time their scene was sampled. The renderer reads
frames back one call late, so images lag one period (`1 / rate`).

### Enabling and disabling the renderer

A disabled renderer drops its subscriptions to TF, the voxel stream and
`robot_description`, forgets its voxel map and renders nothing; its control
topics keep working. Its GPU resources and the loaded robot stay, so enabling
it again is quick: it subscribes again and gets the whole map from the
voxelizer (see below). The parameter `enabled` sets the state it starts in.

## Voxel stream

The voxelizer publishes the map on `voxel_stream.topic` as
[r3v_interfaces/msg/VoxelMapUpdate](r3v_interfaces/msg/VoxelMapUpdate.msg), reliable and in order
(keep last `voxel_stream.depth`, volatile): deltas and the whole map on the
same topic, so a renderer never has to line the two up.

- Every message carries the voxelizer's `session` (random per start, below
  2^53 so that JavaScript numbers hold it exactly) and a
  `sequence` growing by one per message, the voxel `resolution` and the fixed
  frame (`header.frame_id`, renderers ignore a stream in a frame other than
  their `fixed_frame`).
- `DELTA`: the 4x4x2 voxel blocks (r3v-voxelizer `PointI`, 16 bytes) changed
  since the previous message, at most `voxel_stream.rate` messages a second.
  Every block carries its whole occupancy mask.
- `FULL`: the whole map as 64x64x64 voxel groups (r3v-voxelizer
  `VoxelGroup`, 32784 bytes), as `get_all_points_compressed` returns them and
  the tesselator's `replace_input_geometry` takes them. It replaces whatever a
  renderer holds. The voxelizer sends one after the map was cleared (also by
  a settings change: an empty map at the new resolution), when it gets a
  subscriber while it had none, and when a renderer asks for one.
- `data` is little endian and compressed as `compression` says
  (`voxel_stream.compression`, zstd level `voxel_stream.compression_level` by
  default; none within the combined launch file). Zero runs make up most of a
  FULL message, so it compresses to about the size of its occupied pages.
- After `voxel_stream.heartbeat_period` without a message an empty DELTA goes
  out, so renderers notice a voxelizer restart and always have a sequence to
  ask from. Without subscribers nothing is published.

A renderer follows the messages of one session in a row from some sequence
`since` on (the first message it got, or the first after a gap or a new
session). Until a FULL message comes it drops deltas and asks for one on
`voxel_stream.request_topic` ([r3v_interfaces/msg/FullSyncRequest](r3v_interfaces/msg/FullSyncRequest.msg):
`session`, `since`), again every `voxel_stream.sync_retry` seconds. The
voxelizer sends a FULL message unless it sent one at or after `since`
already: that one is on its way, in order, so renderers asking together get a
single FULL message. A lost message shows up as a gap in the sequence and
starts this over. A voxel stream subscriber has to be reliable, and a stream
topic takes one voxelizer: renderers take every other session for a restart
and would synchronize again and again between two.

## Notes

- Both nodes want a multi-threaded executor: point clouds wait for TF, which
  arrives on another callback group, and the renderer decodes the voxel stream
  while it renders. The executables use one; as components load them into
  `component_container_mt`.
- Robot visuals are drawn without URDF colors (the renderer shades meshes
  white); voxels are colored by distance from the camera (turbo over
  0..`camera.color_depth_range`, 50 m by default, shifted by
  `camera.color_offset`) or, with `colormap.mode: height`, by height through
  a hypsometric colormap (blue below the neutral height, then green, yellow,
  brown and white above it).
- Marching cubes (`voxels.marching_cubes`) smooths the voxel surface, which
  also closes the gaps a thin, steep surface (ground seen by a lidar) leaves
  between voxels that only touch at an edge.
- Voxels live in the fixed frame and accumulate over time; rays clear voxels
  they pass through.
- Robot links (and a camera frame that is a robot link) are posed from two
  latest lookups, fixed frame to URDF root and root to link, rather than one
  through both: robot pose and joint states may be stamped by different
  clocks (a simulator's lagging one and wall time), and their latest common
  time can fall out of the TF cache.
- A renderer failure shuts the renderer down (and with it the combined
  process) so a supervisor (e.g. compose `restart: unless-stopped`) can
  restart it.

### Migrating from `r3v_node`

The single `r3v_node` is gone. `ros2 launch r3v_ros r3v.launch.py` runs the
same pipeline; parameter files need the per-node sections of
[config/r3v.yaml](r3v_ros/config/r3v.yaml) (`voxelizer.visible` and
`voxelizer.marching_cubes` became `voxels.visible` and `voxels.marching_cubes`
of `r3v_renderer`), and the control topics moved:

- `/r3v_node/voxelizer/set_<setting>` and `get_<setting>`:
  `/r3v_voxelizer/set_<setting>` and `get_<setting>`.
- `/r3v_node/point_clouds/...` and `/r3v_node/clear_voxels`:
  `/r3v_voxelizer/...`.
- Every other `/r3v_node/...` topic: `/r3v_renderer/...`.

## Building without Docker

Needs a ROS 2 install, Rust (via rustup, edition 2024 capable) and the
dependencies of both packages (`rosdep install --from-paths . --ignore-src`).
Clone into a colcon workspace and run `colcon build`; `colcon build
--packages-select r3v_interfaces` builds the messages alone, without Rust.

## Development

C++ code is checked with cpplint ([CPPLINT.cfg](r3v_ros/CPPLINT.cfg)) and uncrustify
([uncrustify.cfg](r3v_ros/uncrustify.cfg)), and the voxel stream payload has unit
tests ([test](r3v_ros/test), no GPU needed). All of them run as tests:

```bash
colcon test --packages-select r3v_ros && colcon test-result --verbose
```

or the linters directly, from the `r3v_ros` directory:

```bash
ament_cpplint src include test
ament_uncrustify -c uncrustify.cfg src include test  # --reformat to fix
```

The code style is the one of Humble's uncrustify (0.72); newer versions
format some constructs differently.

CI (GitHub Actions, [.github/workflows/ci.yml](.github/workflows/ci.yml)):

- `lint`: cpplint and uncrustify on Humble, without building.
- `build`: build and `colcon test` on Humble and Jazzy, warnings are errors
  (`-Werror`). The uncrustify test runs on Humble only.
- `docker`: builds the Docker image for Humble and Jazzy (not pushed).
