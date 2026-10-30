# One message of the voxel stream: a change of the voxel map built by
# r3v_voxelizer_node, or the whole map. See README.md for the protocol.

# Blocks that changed since the previous message of the session.
uint8 DELTA=0
# The whole map: replaces everything a renderer holds.
uint8 FULL=1

uint8 COMPRESSION_NONE=0
uint8 COMPRESSION_ZSTD=1

# stamp: when the message was published, frame_id: the voxelizer's fixed frame.
std_msgs/Header header
# Random for every voxelizer start.
uint64 session
# Starts at 0 and grows by 1 with every published message of the session.
uint64 sequence
uint8 kind
# How data is compressed.
uint8 compression
# Voxel edge length the map is built with, in meters.
float64 resolution
# Little endian, voxel coordinates in the renderer world (Y-up). DELTA:
# blocks of 4x4x2 voxels, int32 x, y, z (origin) and uint32 mask (16 bytes
# each). FULL: groups of 64x64x64 voxels, a 16 byte key block (voxel
# coordinates shifted right by 6, mask 0) and 4096 uint64 page words (32784
# bytes each), as r3v-voxelizer's get_all_points_compressed returns them.
uint8[] data
