# One grid of lines r3v_renderer draws (see r3v_ros docs/topics.md, "Grids"):
# lines `resolution` apart along the x, y and z axes of `pose`, spanning `size`
# centred on its origin. Lines go through the origin and along the boundary,
# so a size that is not a multiple of the resolution leaves narrower cells at
# the edges.

# TF frame the grid is placed in and moves with; empty for the renderer's fixed
# frame. The grid is drawn only while the renderer has a transform from this
# frame to its fixed frame.
string frame
# Pose of the grid's centre in `frame`.
geometry_msgs/Pose pose
# Extent along the grid's x, y and z in meters, each finite and at least 0. An
# axis of size 0 has no lines along it: (10, 10, 0) is a 10 x 10 m plane in the
# grid's xy plane.
geometry_msgs/Vector3 size
# Distance between neighbouring lines (the cell size) along x, y and z in
# meters, each positive. Unused along an axis of size 0.
geometry_msgs/Vector3 resolution
# sRGB, r, g and b between 0 and 1. a is ignored: lines are opaque.
std_msgs/ColorRGBA color
