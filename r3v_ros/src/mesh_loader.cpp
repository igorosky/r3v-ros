#include "r3v_ros/mesh_loader.hpp"

#include <filesystem>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "assimp/Importer.hpp"
#include "assimp/config.h"
#include "assimp/postprocess.h"
#include "assimp/scene.h"

namespace r3v_ros {

namespace {

bool startsWith(const std::string& s, const std::string& prefix) {
  return s.compare(0, prefix.size(), prefix) == 0;
}

bool isFile(const std::filesystem::path& path) {
  std::error_code error;
  return std::filesystem::is_regular_file(path, error);
}

Eigen::Vector3d toEigen(const aiVector3D& v) {
  return { v.x, v.y, v.z };
}

}  // namespace

MeshLoader::MeshLoader(std::vector<std::string> package_paths)
: package_paths_(std::move(package_paths)) {
}

std::string MeshLoader::resolve(const std::string& uri) const {
  static const std::string kFile = "file://";
  static const std::string kPackage = "package://";
  if (startsWith(uri, kFile)) {
    const std::string path = uri.substr(kFile.size());
    return isFile(path) ? path : "";
  }
  if (!startsWith(uri, kPackage)) {
    return isFile(uri) ? uri : "";
  }

  const std::string rest = uri.substr(kPackage.size());
  const size_t slash = rest.find('/');
  if (slash == std::string::npos) {
    return "";
  }
  const std::string package = rest.substr(0, slash);
  const std::string relative = rest.substr(slash + 1);
  for (const auto& dir : package_paths_) {
    const auto candidate = std::filesystem::path(dir) / package / relative;
    if (isFile(candidate)) {
      return candidate.string();
    }
  }
  try {
    const auto candidate =
      std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) / relative;
    if (isFile(candidate)) {
      return candidate.string();
    }
  } catch (const ament_index_cpp::PackageNotFoundError&) {
  }
  return "";
}

Triangles MeshLoader::load(const std::string& path) const {
  Assimp::Importer importer;
  // ROS meshes are Z-up whatever their Collada <up_axis> says; this matches
  // rviz, which drops the root transform assimp would rotate with. The unit
  // scale is still applied.
  importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
  importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
  const aiScene* scene = importer.ReadFile(
    path, aiProcess_Triangulate | aiProcess_PreTransformVertices | aiProcess_SortByPType);
  if (scene == nullptr) {
    throw std::runtime_error("assimp: " + std::string(importer.GetErrorString()));
  }

  Triangles out;
  for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
    const aiMesh* mesh = scene->mMeshes[m];
    for (unsigned f = 0; f < mesh->mNumFaces; ++f) {
      const aiFace& face = mesh->mFaces[f];
      if (face.mNumIndices != 3) {
        continue;
      }
      addTriangle(
        out, toEigen(mesh->mVertices[face.mIndices[0]]),
        toEigen(mesh->mVertices[face.mIndices[1]]),
        toEigen(mesh->mVertices[face.mIndices[2]]));
    }
  }
  if (out.empty()) {
    throw std::runtime_error("no triangles");
  }
  return out;
}

}  // namespace r3v_ros
