#include "r3v_ros/primitives.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace r3v_ros {

namespace {

void addQuad(
  Triangles& out, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
  const Eigen::Vector3d& c, const Eigen::Vector3d& d) {
  addTriangle(out, a, b, c);
  addTriangle(out, a, c, d);
}

void setVertex(float (& out)[3], const Eigen::Vector3d& v) {
  out[0] = static_cast<float>(v.x());
  out[1] = static_cast<float>(v.y());
  out[2] = static_cast<float>(v.z());
}

}  // namespace

void addTriangle(
  Triangles& out, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
  const Eigen::Vector3d& c) {
  rr::R3vTriangle triangle;
  setVertex(triangle.v0, a);
  setVertex(triangle.v1, b);
  setVertex(triangle.v2, c);
  out.push_back(triangle);
}

Triangles makeBox(const Eigen::Vector3d& size) {
  const Eigen::Vector3d half = size / 2.0;
  Triangles out;
  out.reserve(12);
  for (int axis = 0; axis < 3; ++axis) {
    // u x v = +axis
    const int u_axis = (axis + 1) % 3;
    const int v_axis = (axis + 2) % 3;
    for (const double sign : { 1.0, -1.0 }) {
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      center[axis] = sign * half[axis];
      Eigen::Vector3d u = Eigen::Vector3d::Zero();
      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      u[u_axis] = half[u_axis];
      v[v_axis] = half[v_axis];
      if (sign < 0.0) {
        std::swap(u, v);
      }
      addQuad(out, center - u - v, center + u - v, center + u + v, center - u + v);
    }
  }
  return out;
}

Triangles makeCylinder(double radius, double length, int segments) {
  const double h = length / 2.0;
  const Eigen::Vector3d top(0.0, 0.0, h);
  const Eigen::Vector3d bottom(0.0, 0.0, -h);
  Triangles out;
  out.reserve(static_cast<size_t>(segments) * 4);
  for (int i = 0; i < segments; ++i) {
    const double a0 = 2.0 * M_PI * i / segments;
    const double a1 = 2.0 * M_PI * (i + 1) / segments;
    const Eigen::Vector3d r0(radius * std::cos(a0), radius * std::sin(a0), 0.0);
    const Eigen::Vector3d r1(radius * std::cos(a1), radius * std::sin(a1), 0.0);
    addQuad(out, bottom + r0, bottom + r1, top + r1, top + r0);
    addTriangle(out, top, top + r0, top + r1);
    addTriangle(out, bottom, bottom + r1, bottom + r0);
  }
  return out;
}

Triangles makeSphere(double radius, int stacks, int slices) {
  const auto point = [&](int stack, int slice) {
      const double theta = M_PI * stack / stacks;
      const double phi = 2.0 * M_PI * slice / slices;
      return Eigen::Vector3d(
        radius * std::sin(theta) * std::cos(phi),
        radius * std::sin(theta) * std::sin(phi),
        radius * std::cos(theta));
    };
  Triangles out;
  out.reserve(static_cast<size_t>(stacks) * static_cast<size_t>(slices) * 2);
  for (int i = 0; i < stacks; ++i) {
    for (int j = 0; j < slices; ++j) {
      const Eigen::Vector3d a = point(i, j);
      const Eigen::Vector3d b = point(i + 1, j);
      const Eigen::Vector3d c = point(i + 1, j + 1);
      const Eigen::Vector3d d = point(i, j + 1);
      // The first and last stacks meet in a pole, where one triangle of the
      // quad collapses.
      if (i != stacks - 1) {
        addTriangle(out, a, b, c);
      }
      if (i != 0) {
        addTriangle(out, a, c, d);
      }
    }
  }
  return out;
}

void scaleTriangles(Triangles& triangles, const Eigen::Vector3d& scale) {
  const bool mirrored = scale.x() * scale.y() * scale.z() < 0.0;
  for (auto& triangle : triangles) {
    for (float* vertex : { triangle.v0, triangle.v1, triangle.v2 }) {
      for (int k = 0; k < 3; ++k) {
        vertex[k] = static_cast<float>(vertex[k] * scale[k]);
      }
    }
    if (mirrored) {
      std::swap_ranges(triangle.v1, triangle.v1 + 3, triangle.v2);
    }
  }
}

void makeDoubleSided(Triangles& triangles) {
  const size_t count = triangles.size();
  triangles.reserve(count * 2);
  for (size_t i = 0; i < count; ++i) {
    rr::R3vTriangle reversed = triangles[i];
    std::swap_ranges(reversed.v1, reversed.v1 + 3, reversed.v2);
    triangles.push_back(reversed);
  }
}

}  // namespace r3v_ros
