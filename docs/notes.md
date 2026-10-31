# Notes

[Back to README](../README.md)

## Behavior

- **Executor.** Both nodes need a multi-threaded executor: point clouds wait
  for TF, which arrives on another callback group, and the renderer decodes
  the voxel stream while it renders. The executables use one. As components,
  load them into `component_container_mt`.
- **Colors.** Robot visuals are drawn without URDF colors (the renderer shades
  meshes white). Voxels are colored by distance from the camera (turbo over
  0..`camera.color_depth_range`, 50 m by default, shifted by
  `camera.color_offset`) or, with `colormap.mode: height`, by height through a
  hypsometric colormap (blue below the neutral height, then green, yellow,
  brown and white above it).
- **Marching cubes** (`voxels.marching_cubes`) smooths the voxel surface. It
  also closes the gaps a thin, steep surface (ground seen by a lidar) leaves
  between voxels that only touch at an edge.
- **Map.** Voxels live in the fixed frame and accumulate over time. Rays clear
  the voxels they pass through.
- **Clearing.** `~/clear_voxels` empties the map in place. A change of the
  resolution, a threshold or `max_value` makes a new voxelizer, so it clears
  the map too. Multiplier changes keep the map and apply to the points that
  follow.
- **Downsampling.** The downsampled stream costs the voxelizer nothing while
  nobody follows it. Rendering both views takes two GPU renderers, each with
  its own copy of the robot meshes.
- **Robot pose.** Robot links (and a camera frame that is a robot link) are
  posed from two latest TF lookups, fixed frame to URDF root and root to link,
  rather than one through both. Robot pose and joint states may be stamped by
  different clocks (a simulator's lagging one and wall time), and their latest
  common time can fall out of the TF cache.
- **Failures.** A renderer failure shuts the renderer down (and with it the
  combined process), so a supervisor (for example compose
  `restart: unless-stopped`) can restart it.

## Migrating from `r3v_node`

The single `r3v_node` is gone. `ros2 launch r3v_ros r3v.launch.py` runs the
same pipeline.

Parameter files need the per-node sections of
[config/r3v.yaml](../r3v_ros/config/r3v.yaml). `voxelizer.visible` and
`voxelizer.marching_cubes` became `voxels.visible` and `voxels.marching_cubes`
of `r3v_renderer`.

The control topics moved:

| Old | New |
|---|---|
| `/r3v_node/voxelizer/set_<setting>`, `get_<setting>` | `/r3v_voxelizer/set_<setting>`, `get_<setting>` |
| `/r3v_node/point_clouds/...`, `/r3v_node/clear_voxels` | `/r3v_voxelizer/...` |
| Every other `/r3v_node/...` topic | `/r3v_renderer/...` |
