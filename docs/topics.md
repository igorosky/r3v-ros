# Topics

[Back to README](../README.md)

- [r3v_voxelizer](#r3v_voxelizer)
- [r3v_renderer](#r3v_renderer)
- [Views](#views)
- [Camera](#camera)
- [Runtime control](#runtime-control)
- [Reading the current settings](#reading-the-current-settings)
- [Output](#output)
- [Enabling and disabling the renderer](#enabling-and-disabling-the-renderer)

Topic names in the tables are parameters of [config/r3v.yaml](../r3v_ros/config/r3v.yaml)
with their default in parentheses, or fixed names relative to the node (`~/`).

## r3v_voxelizer

| Topic | Type | Direction |
|---|---|---|
| `tf_topic` (`/tf`), `tf_static_topic` (`/tf_static`) | `tf2_msgs/TFMessage` | in |
| `point_clouds.<name>.topic` | `sensor_msgs/PointCloud2` | in |
| `~/request_full_sync`, `~/downsampled/request_full_sync` | `r3v_interfaces/FullSyncRequest` | in |
| `~/clear_voxels` | `std_msgs/Empty` | in |
| `~/set_resolution` | `std_msgs/Float64` | in |
| `~/set_destroy_threshold`, `set_construct_threshold`, `set_max_value` | `std_msgs/UInt32` | in |
| `~/point_clouds/<name>/set_intensity_multiplier` | `std_msgs/Float64` | in |
| `~/point_clouds/<name>/set_hit_multiplier`, `set_target_multiplier` | `std_msgs/UInt32` | in |
| `~/downsampling/set_enabled` | `std_msgs/Bool` | in |
| `~/downsampling/set_voxel_size`, `set_threshold` | `std_msgs/UInt32` | in |
| `voxel_stream.topic` (`/r3v/voxels`) | `r3v_interfaces/VoxelMapUpdate` (reliable) | out |
| `voxel_stream.downsampled_topic` (`/r3v/voxels_downsampled`) | `r3v_interfaces/VoxelMapUpdate` (reliable) | out |

## r3v_renderer

| Topic | Type | Direction |
|---|---|---|
| `tf_topic` (`/tf`), `tf_static_topic` (`/tf_static`) | `tf2_msgs/TFMessage` | in |
| `robot_description.topic` (`/robot_description`) | `std_msgs/String` (transient local) | in |
| `voxel_stream.topic` (`/r3v/voxels`) | `r3v_interfaces/VoxelMapUpdate` (reliable) | in |
| `voxel_stream.downsampled_topic` (`/r3v/voxels_downsampled`) | `r3v_interfaces/VoxelMapUpdate` (reliable) | in |
| `~/set_enabled` | `std_msgs/Bool` | in |
| `~/set_mode` | `std_msgs/String` | in |
| `camera.pose_topic` (`~/camera_pose`) | `geometry_msgs/PoseStamped` | in |
| `~/hidden_links` | `std_msgs/String` | in |
| `~/voxels_visible`, `~/downsampled/voxels_visible` | `std_msgs/Bool` | in |
| `~/marching_cubes/set_enabled` | `std_msgs/Bool` | in |
| `~/background/set_color` | `std_msgs/ColorRGBA` | in |
| `~/colormap/set_depth_range`, `set_color_offset` | `std_msgs/Float64` | in |
| `~/colormap/set_mode` | `std_msgs/String` | in |
| `~/colormap/set_height_relative` | `std_msgs/Bool` | in |
| `~/colormap/set_height_offset`, `set_height_scale` | `std_msgs/Float64` | in |
| `~/colormap/set_height_bands` | `std_msgs/UInt32` | in |
| `~/grids/set` | `r3v_interfaces/GridArray` | in |
| `~/grids/get` | `r3v_interfaces/GridArray` (transient local) | out |
| `voxel_stream.request_topic` (`/r3v_voxelizer/request_full_sync`) | `r3v_interfaces/FullSyncRequest` | out |
| `voxel_stream.downsampled_request_topic` (`/r3v_voxelizer/downsampled/request_full_sync`) | `r3v_interfaces/FullSyncRequest` | out |
| `output.topic` (`/r3v/image`) + transports | `sensor_msgs/Image` (best effort, `output.qos`) | out |
| `output.downsampled_topic` (`/r3v/image_downsampled`) + transports | `sensor_msgs/Image` (best effort, `output.qos`) | out |

The voxel stream topics are described in [voxel-stream.md](voxel-stream.md).
Only the streams and image topics of the [views](#views) in use are
subscribed and published.

## Views

The renderer has two views: the full voxel map (`voxel_stream.topic` to
`output.topic`) and the downsampled one (`voxel_stream.downsampled_topic` to
`output.downsampled_topic`). `mode` (`none`, `full`, `downsampled` or `both`)
says which of them run. Both show the same camera, robot and colormap.

A view that runs follows its voxel stream and, with `output.lazy`, renders
only while its image topic has subscribers. Each view has its own GPU
renderer, made the first time the view runs and kept until exit, so switching
back is quick. A view that stops forgets its voxel map and gets it again from
the voxelizer when it runs again.

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

Every setting starts from a parameter of [config/r3v.yaml](../r3v_ros/config/r3v.yaml),
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
| `~/downsampling/set_enabled` | `std_msgs/Bool` | `downsampling.enabled` | Disabled, the downsampled stream carries an empty map |
| `~/downsampling/set_voxel_size` | `std_msgs/UInt32` | `downsampling.voxel_size` | Edge of a downsampled voxel in voxels, a power of two from 2 to 128 |
| `~/downsampling/set_threshold` | `std_msgs/UInt32` | `downsampling.threshold` | Voxels of the cube that make a downsampled voxel, from 1 to `voxel_size`^3 |

`<name>` is one of `point_clouds.names`, which therefore have to be valid in
topic names. The meaning of the thresholds, multipliers and downsampling is
described in r3v-voxelizer's
[parameters](https://github.com/igorosky/r3v-voxelizer/blob/master/docs/parameters.md).

The resolution, the thresholds and `max_value` change how the map is built, so
a change clears the whole voxel map and starts it over from nothing. Changes
arriving within one `voxel_stream.rate` period clear it once. `~/clear_voxels`
empties the map in place.

Multipliers only apply to the points that follow, the map stays.

Downsampling changes only send the downsampled map again. Voxel size and
threshold are checked together: set the threshold first when it would not fit
the new size, or send both within one period.

### r3v_renderer

| Topic | Type | Starts from | Value |
|---|---|---|---|
| `~/set_enabled` | `std_msgs/Bool` | `enabled` | Switches the renderer, see [below](#enabling-and-disabling-the-renderer) |
| `~/set_mode` | `std_msgs/String` | `mode` | `none`, `full`, `downsampled` or `both`, see [views](#views) |
| `~/hidden_links` | `std_msgs/String` | `robot_description.hidden_links` | Link names to hide, see below |
| `~/voxels_visible` | `std_msgs/Bool` | `voxels.visible` | Hides or shows the voxel map. The voxelizer keeps building it while hidden. |
| `~/downsampled/voxels_visible` | `std_msgs/Bool` | `voxels.downsampled_visible` | The same for the downsampled view |
| `~/marching_cubes/set_enabled` | `std_msgs/Bool` | `voxels.marching_cubes` | Marching cubes smoothing of the voxel surface |
| `~/background/set_color` | `std_msgs/ColorRGBA` | `output.background_color` | sRGB components between 0 and 1, alpha ignored |
| `~/colormap/set_mode` | `std_msgs/String` | `colormap.mode` | `distance` or `height`, see below |
| `~/colormap/set_depth_range` | `std_msgs/Float64` | `camera.color_depth_range` | Meters over which the colormap runs once, positive |
| `~/colormap/set_color_offset` | `std_msgs/Float64` | `camera.color_offset` | Meters added to the distance voxels are colored by, finite |
| `~/colormap/set_height_relative` | `std_msgs/Bool` | `colormap.height_relative` | Neutral height follows `colormap.height_frame` (true) or is z 0 of the fixed frame (false) |
| `~/colormap/set_height_offset` | `std_msgs/Float64` | `colormap.height_offset` | Meters added to the neutral height, finite |
| `~/colormap/set_height_scale` | `std_msgs/Float64` | `colormap.height_scale` | Meters from the neutral height where colors start changing slower, positive |
| `~/colormap/set_height_bands` | `std_msgs/UInt32` | `colormap.height_bands` | 0 for a continuous colormap, otherwise that many flat bands, at most 256 |
| `~/grids/set` | `r3v_interfaces/GridArray` | none (no grids) | Every grid to draw, replacing the previous ones, see [below](#grids) |

**Hidden links.** `~/hidden_links` takes comma or space separated link names.
Every listed link is hidden together with all links below it in the URDF tree
(the arm's base link hides the whole arm). An empty string shows everything
again. The setting survives a new robot description.

**Colormap modes.** `distance` colors voxels by their distance from the camera
(`set_depth_range`, `set_color_offset`). `height` colors them by their height
above a neutral height (the `set_height_*` topics).

### Grids

`~/grids/set` replaces every grid the renderer draws (in every view) with the
ones of the `r3v_interfaces/GridArray`; an empty array removes them all. A
grid (`r3v_interfaces/Grid`) is lines `resolution` apart along the x, y and z
axes of its `pose`, spanning `size` centred on the pose's origin; a size of 0
on an axis leaves no lines along it, so `size` (10, 10, 0) is a 10 x 10 m plane
in the pose's xy plane. Lines go through the origin and along the boundary, so
a size that is not a multiple of the resolution leaves narrower cells at the
edges. Lines are opaque, one pixel wide and hidden behind voxels and the robot.

- `frame` is the TF frame the grid moves with, empty for `fixed_frame`. Robot
  links are posed exactly like the robot's visuals (latest URDF root pose
  composed with the latest link transform). A grid shows once its frame has a
  transform to `fixed_frame` and keeps its last pose while TF is missing.
- An array is taken whole or not at all: more than 64 grids, a non-finite or
  zero quaternion, a negative or non-finite size, a resolution that is not
  positive, color components outside 0..1, or a grid of more than 2^20 lines
  (r3v-renderer's limit) drops it with a warning, and `~/grids/get` stays as it
  was. The color's alpha is ignored.
- `~/grids/get` holds the array drawn, exactly as it was sent (an empty frame
  stays empty). It starts empty, so a client that keeps grids elsewhere (the
  web app keeps them in key_value_storage) sees the node (re)start and sends
  them again. Sending the array that is drawn already changes nothing and
  publishes nothing.
- There is no parameter for grids: the node starts without any.

## Reading the current settings

Every setting with a `set_*` topic is published on the matching `get_*` topic
of its node, for example:

- `r3v_renderer`: `~/get_enabled`, `~/get_mode`, `~/marching_cubes/get_enabled`,
  `~/background/get_color`, `~/colormap/get_depth_range`,
  `~/colormap/get_mode`, `~/grids/get`, ...
- `r3v_voxelizer`: `~/get_resolution`,
  `~/point_clouds/<name>/get_hit_multiplier`, `~/downsampling/get_voxel_size`,
  ...

They use a transient local QoS, so a late subscriber (with a transient local,
reliable QoS too) gets the current state.

## Output

- With `output.lazy` (default) a view renders nothing while nobody subscribes
  to its image topic.
- Frames are stamped with the time their scene was sampled.
- The renderer reads frames back one call late, so images lag one period
  (`1 / rate`).

## Enabling and disabling the renderer

A disabled renderer drops its subscriptions to TF, the voxel streams and
`robot_description`, forgets its voxel maps and renders nothing. Its control
topics keep working. Mode `none` does the same.

Its GPU resources and the loaded robot stay, so enabling it again is quick: it
subscribes again and gets the whole map from the voxelizer (see
[voxel-stream.md](voxel-stream.md)). The parameter `enabled` sets the state it
starts in.
