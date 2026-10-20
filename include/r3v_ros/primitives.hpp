#pragma once

#include <vector>

#include <Eigen/Core>

#include "r3v_ros/r3v_handles.hpp"

namespace r3v_ros
{

// Triangle soup, counter-clockwise seen from the front (what the renderer
// culls by).
using Triangles = std::vector<rr::R3vTriangle>;

void addTriangle(
  Triangles & out, const Eigen::Vector3d & a, const Eigen::Vector3d & b,
  const Eigen::Vector3d & c);

// URDF primitives, centered on the origin. The cylinder axis is Z.
Triangles makeBox(const Eigen::Vector3d & size);
Triangles makeCylinder(double radius, double length, int segments = 32);
Triangles makeSphere(double radius, int stacks = 16, int slices = 32);

// Scales every vertex; a mirroring scale also flips the winding so faces
// keep pointing outwards.
void scaleTriangles(Triangles & triangles, const Eigen::Vector3d & scale);

// Appends a reversed copy of every triangle, for meshes with broken winding.
void makeDoubleSided(Triangles & triangles);

}  // namespace r3v_ros
