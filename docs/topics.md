# Topics

[Back to README](../README.md)

- [r3v_voxelizer](#r3v_voxelizer)
- [r3v_renderer](#r3v_renderer)
- [Camera](#camera)
- [Runtime control](#runtime-control)
- [Reading the current settings](#reading-the-current-settings)
- [Output](#output)
- [Enabling and disabling the renderer](#enabling-and-disabling-the-renderer)

Topic names in the tables are parameters of [config/r3v.yaml](../config/r3v.yaml)
with their default in parentheses, or fixed names relative to the node (`~/`).

## r3v_voxelizer

| Topic | Type | Direction |
|---|---|---|
| `tf_topic` (`/tf`), `tf_static_topic` (`/tf_static`) | `tf2_msgs/TFMessage` | in |
| `point_clouds.<name>.topic` | `sensor_msgs/PointCloud2` | in |
| `~/request_full_sync` | `r3v_ros/FullSyncRequest` | in |
| `~/clear_voxels` | `std_msgs/Empty` | in |
| `~/set_resolution` | `std_msgs/Float64` | in |
| `~/set_destroy_threshold`, `set_construct_threshold`, `set_max_value` | `std_msgs/UInt32` | in |
| `~/point_clouds/<name>/set_intensity_multiplier` | `std_msgs/Float64` | in |
| `~/point_clouds/<name>/set_hit_multiplier`, `set_target_multiplier` | `std_msgs/UInt32` | in |
| `voxel_stream.topic` (`/r3v/voxels`) | `r3v_ros/VoxelMapUpdate` (reliable) | out |

## r3v_renderer

| Topic | Type | Direction |
|---|---|---|
| `tf_topic` (`/tf`), `tf_static_topic` (`/tf_static`) | `tf2_msgs/TFMessage` | in |
| `robot_description.topic` (`/robot_description`) | `std_msgs/String` (transient local) | in |
| `voxel_stream.topic` (`/r3v/voxels`) | `r3v_ros/VoxelMapUpdate` (reliable) | in |
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
| `voxel_stream.request_topic` (`/r3v_voxelizer/request_full_sync`) | `r3v_ros/FullSyncRequest` | out |
| `output.topic` (`/r3v/image`) + transports | `sensor_msgs/Image` (best effort, `output.qos`) | out |

The voxel stream topics are described in [voxel-stream.md](voxel-stream.md).

## Camera

The camera is placed by `camera.position` and `camera.target`, given in
`camera.frame` (the fixed frame by default; a robot frame makes the camera
follow the robot).

A pose on `camera.pose_topic` replaces them: the camera sits at the pose and
looks along its +X axis with +Z up.

## Runtime control

These topics change the scene at runtime. Each message replaces the previous
state rather than adding to it, so publishing the same one again is harmless.
Invalid values are logged and ignored.

Every setting starts from a parameter of [config/r3v.yaml](../config/r3v.yaml),
listed in the "Starts from" column.

### r3v_voxelizer

| Topic | Type | Starts from | Value |
|---|---|---|---|
| `~/clear_voxels` | `std_msgs/Empty` | | Forgets the whole voxel map |
| `~/set_resolution` | `std_msgs/Float64` | `voxelizer.resolution` | Voxel edge length in meters, positive |
| `~/set_destroy_threshold` | `std_msgs/UInt32` | `voxelizer.destroy_threshold` | |
| `~/set_construct_threshold` | `std_msgs/UInt32` | `voxelizer.construct_threshold` | |
| `~/set_max_value` | `std_msgs/UInt32` | `voxelizer.max_value` | |
| `~/point_clouds/<name>/set_intensity_multiplier` | `std_msgs/Float64` | `point_clouds.<name>.intensity_multiplier` | Finite |
| `~/point_clouds/<name>/set_hit_multiplier` | `std_msgs/UInt32` | `point_clouds.<name>.hit_multiplier` | |
| `~/point_clouds/<name>/set_target_multiplier` | `std_msgs/UInt32` | `point_clouds.<name>.target_multiplier` | |

`<name>` is one of `point_clouds.names`, which therefore have to be valid in
topic names. The meaning of the thresholds and multipliers is described in
r3v-voxelizer's
[parameters](https://github.com/igorosky/r3v-voxelizer/blob/master/docs/parameters.md).

These settings change how the map is built, so every change (even of a single
point cloud's multiplier, all clouds share the voxelizer) clears the whole
voxel map and starts it over from nothing. Changes arriving within one
`voxel_stream.rate` period clear it once.

### r3v_renderer

| Topic | Type | Starts from | Value |
|---|---|---|---|
| `~/set_enabled` | `std_msgs/Bool` | `enabled` | Switches the renderer, see [below](#enabling-and-disabling-the-renderer) |
| `~/hidden_links` | `std_msgs/String` | `robot_description.hidden_links` | Link names to hide, see below |
| `~/voxels_visible` | `std_msgs/Bool` | `voxels.visible` | Hides or shows the voxel map. The voxelizer keeps building it while hidden. |
| `~/marching_cubes/set_enabled` | `std_msgs/Bool` | `voxels.marching_cubes` | Marching cubes smoothing of the voxel surface |
| `~/background/set_color` | `std_msgs/ColorRGBA` | `output.background_color` | sRGB components between 0 and 1, alpha ignored |
| `~/colormap/set_mode` | `std_msgs/String` | `colormap.mode` | `distance` or `height`, see below |
| `~/colormap/set_depth_range` | `std_msgs/Float64` | `camera.color_depth_range` | Meters over which the colormap runs once, positive |
| `~/colormap/set_color_offset` | `std_msgs/Float64` | `camera.color_offset` | Meters added to the distance voxels are colored by, finite |
| `~/colormap/set_height_relative` | `std_msgs/Bool` | `colormap.height_relative` | Neutral height follows `colormap.height_frame` (true) or is z 0 of the fixed frame (false) |
| `~/colormap/set_height_offset` | `std_msgs/Float64` | `colormap.height_offset` | Meters added to the neutral height, finite |
| `~/colormap/set_height_scale` | `std_msgs/Float64` | `colormap.height_scale` | Meters from the neutral height where colors start changing slower, positive |
| `~/colormap/set_height_bands` | `std_msgs/UInt32` | `colormap.height_bands` | 0 for a continuous colormap, otherwise that many flat bands, at most 256 |

**Hidden links.** `~/hidden_links` takes comma or space separated link names.
Every listed link is hidden together with all links below it in the URDF tree
(the arm's base link hides the whole arm). An empty string shows everything
again. The setting survives a new robot description.

**Colormap modes.** `distance` colors voxels by their distance from the camera
(`set_depth_range`, `set_color_offset`). `height` colors them by their height
above a neutral height (the `set_height_*` topics).

## Reading the current settings

Every setting with a `set_*` topic is published on the matching `get_*` topic
of its node, for example:

- `r3v_renderer`: `~/get_enabled`, `~/marching_cubes/get_enabled`,
  `~/background/get_color`, `~/colormap/get_depth_range`,
  `~/colormap/get_mode`, ...
- `r3v_voxelizer`: `~/get_resolution`,
  `~/point_clouds/<name>/get_hit_multiplier`, ...

They use a transient local QoS, so a late subscriber (with a transient local,
reliable QoS too) gets the current state.

## Output

- With `output.lazy` (default) nothing is rendered while nobody subscribes.
- Frames are stamped with the time their scene was sampled.
- The renderer reads frames back one call late, so images lag one period
  (`1 / rate`).

## Enabling and disabling the renderer

A disabled renderer drops its subscriptions to TF, the voxel stream and
`robot_description`, forgets its voxel map and renders nothing. Its control
topics keep working.

Its GPU resources and the loaded robot stay, so enabling it again is quick: it
subscribes again and gets the whole map from the voxelizer (see
[voxel-stream.md](voxel-stream.md)). The parameter `enabled` sets the state it
starts in.
