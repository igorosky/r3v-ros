# Voxel stream

[Back to README](../README.md)

The voxelizer publishes the map on `voxel_stream.topic` (`/r3v/voxels`) as
[r3v_interfaces/msg/VoxelMapUpdate](../r3v_interfaces/msg/VoxelMapUpdate.msg), reliable and in
order (keep last `voxel_stream.depth`, volatile). Changes and the whole map go
on the same topic, so a renderer never has to line the two up.

A second stream on `voxel_stream.downsampled_topic` carries a coarser copy of
the map, see [Downsampled stream](#downsampled-stream). Both work the same way.

## Messages

Every message carries:

- the voxelizer's `session` (random per start, below 2^53 so that
  JavaScript numbers, e.g. through rosbridge, hold it exactly and can send it
  back),
- a `sequence` growing by one per message,
- the voxel `resolution`,
- the fixed frame in `header.frame_id`. Renderers ignore a stream in a frame
  other than their `fixed_frame`.

There are two kinds:

| Kind | Content | When |
|---|---|---|
| `DELTA` | The 4x4x2 voxel blocks (r3v-voxelizer `PointI`, 16 bytes) changed since the previous message. Every block carries its whole occupancy mask. | At most `voxel_stream.rate` messages a second |
| `FULL` | The whole map as 64x64x64 voxel groups (r3v-voxelizer `VoxelGroup`, 32784 bytes), as `get_all_points_compressed` returns them and the tesselator's `replace_input_geometry` takes them. Replaces whatever a renderer holds. | After the map was cleared (also by a change of the resolution, a threshold or `max_value`: an empty map at the new resolution), when the stream gets a subscriber while it had none, and when a renderer asks for one |

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
   [r3v_interfaces/msg/FullSyncRequest](../r3v_interfaces/msg/FullSyncRequest.msg) (`session`,
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

## Downsampled stream

The voxelizer also publishes the map downsampled (r3v-voxelizer's
[downsampling](https://github.com/igorosky/r3v-voxelizer/blob/master/docs/parameters.md#downsampling))
on `voxel_stream.downsampled_topic` (`/r3v/voxels_downsampled`). One
downsampled voxel stands for a cube of `downsampling.voxel_size`^3 voxels and
is there while at least `downsampling.threshold` of them are.

- Same messages and synchronization as above, with its own `sequence` and
  request topic (`~/downsampled/request_full_sync`). Both streams share the
  session.
- Coordinates are in downsampled voxels and `resolution` is the edge of a
  downsampled voxel (`voxelizer.resolution * downsampling.voxel_size`), so a
  renderer draws it like any map.
- DELTA blocks come from `get_downsampled`, FULL groups from
  `get_all_downsampled`.
- The voxelizer only does the downsampling work while the stream has
  subscribers.
- A change of the downsampling settings sends a FULL message on this stream
  only. Disabled, it carries an empty map.

A client can follow either stream or both.
