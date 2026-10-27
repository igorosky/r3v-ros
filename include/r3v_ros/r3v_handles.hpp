#pragma once

#include <memory>
#include <string>

#include "r3v-renderer.hpp"
#include "r3v-voxelizer.hpp"

namespace r3v_ros {

namespace rr = r3v::renderer;
namespace rv = r3v::voxelizer;

static_assert(sizeof(rr::R3vPointI) == sizeof(rv::PointI),
  "r3v-renderer R3vPointI and r3v-voxelizer PointI must share a layout");

struct RendererDeleter {
  void operator()(rr::R3vWgpuRenderer* p) const { rr::r3v_wgpu_renderer_drop(p); }
};
struct TesselatorDeleter {
  void operator()(rr::R3vCpuTesselator* p) const { rr::r3v_cpu_tesselator_drop(p); }
};
struct MeshDeleter {
  void operator()(rr::R3vStlMesh* p) const { rr::r3v_stl_mesh_drop(p); }
};
struct GeometryDeltaDeleter {
  void operator()(rr::R3vGeometryDelta* p) const { rr::r3v_geometry_delta_drop(p); }
};
struct VoxelizerDeleter {
  void operator()(void* p) const { rv::drop_voxelizer(p); }
};

using RendererPtr = std::unique_ptr<rr::R3vWgpuRenderer, RendererDeleter>;
using TesselatorPtr = std::unique_ptr<rr::R3vCpuTesselator, TesselatorDeleter>;
using MeshPtr = std::unique_ptr<rr::R3vStlMesh, MeshDeleter>;
using GeometryDeltaPtr = std::unique_ptr<rr::R3vGeometryDelta, GeometryDeltaDeleter>;
using VoxelizerPtr = std::unique_ptr<void, VoxelizerDeleter>;

// Message of the last r3v-renderer failure on this thread.
std::string lastRendererError();

// Throws std::runtime_error naming `what` and the library's error unless
// `status` is R3V_OK.
void checkStatus(rr::R3vStatus status, const std::string& what);

}  // namespace r3v_ros
