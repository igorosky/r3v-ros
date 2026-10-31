# Asks r3v_voxelizer_node for a FULL message of the voxel stream, unless one
# was published at or after `since` in `session`.

# Session of the voxel stream the requester follows.
uint64 session
# Sequence of the first message the requester received in a row (since its
# last gap, session change or start).
uint64 since
