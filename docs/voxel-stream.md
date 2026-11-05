# Voxel stream

[Back to README](../README.md)

The voxelizer publishes the map on `voxel_stream.topic` (`/r3v/voxels`) as
[r3v_ros/msg/VoxelMapUpdate](../msg/VoxelMapUpdate.msg), reliable and in
order (keep last `voxel_stream.depth`, volatile). Changes and the whole map go
on the same topic, so a renderer never has to line the two up.

## Messages

Every message carries:

- the voxelizer's `session` (random per start),
- a `sequence` growing by one per message,
- the voxel `resolution`,
- the fixed frame in `header.frame_id`. Renderers ignore a stream in a frame
  other than their `fixed_frame`.

There are two kinds:

| Kind | Content | When |
|---|---|---|
| `DELTA` | The 4x4x2 voxel blocks (r3v-voxelizer `PointI`, 16 bytes) changed since the previous message. Every block carries its whole occupancy mask. | At most `voxel_stream.rate` messages a second |
| `FULL` | The whole map as 64x64x64 voxel groups (r3v-voxelizer `VoxelGroup`, 32784 bytes), as `get_all_points_compressed` returns them and the tesselator's `replace_input_geometry` takes them. Replaces whatever a renderer holds. | After the map was cleared (also by a settings change: an empty map at the new resolution), when the voxelizer gets a subscriber while it had none, and when a renderer asks for one |

`data` is little endian and compressed as the message's `compression` field
says. The voxelizer sets it from `voxel_stream.compression` (zstd at level
`voxel_stream.compression_level` by default; none within the combined launch
file). Zero runs make up most of a FULL message, so it compresses to about the
size of its occupied pages.

After `voxel_stream.heartbeat_period` without a message an empty DELTA goes
out, so renderers notice a voxelizer restart and always have a sequence to
ask from. Without subscribers nothing is published.

## Synchronization

A renderer follows the messages of one session in a row, from some sequence
`since` on (the first message it got, or the first after a gap or a new
session):

1. Until a FULL message comes, it drops deltas and asks for one on
   `voxel_stream.request_topic` with a
   [r3v_ros/msg/FullSyncRequest](../msg/FullSyncRequest.msg) (`session`,
   `since`). It asks again every `voxel_stream.sync_retry` seconds.
2. The voxelizer sends a FULL message, unless it already sent one at or after
   `since`. That one is on its way, in order, so renderers asking together get
   a single FULL message.
3. After the FULL message the renderer applies every DELTA in order.
4. A lost message shows up as a gap in the sequence and starts this over.

Rules for subscribers:

- A voxel stream subscriber has to be reliable.
- A stream topic takes one voxelizer. Renderers take every other session for
  a restart and would synchronize again and again between two.
