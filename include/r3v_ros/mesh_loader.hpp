#pragma once

#include <string>
#include <vector>

#include "r3v_ros/primitives.hpp"

namespace r3v_ros
{

class MeshLoader
{
public:
  // `package_paths` are searched for package://<pkg>/<path> as
  // <dir>/<pkg>/<path> before the ament index, so meshes can be mounted
  // into a container without installing their package.
  explicit MeshLoader(std::vector<std::string> package_paths);

  // Local path of a URDF mesh filename (package://, file:// or a plain
  // path). Empty when it cannot be found.
  std::string resolve(const std::string & uri) const;

  // Loads every triangle of a mesh file in any format assimp reads (STL,
  // DAE, OBJ, glTF, ...). Throws std::runtime_error on failure.
  Triangles load(const std::string & path) const;

private:
  std::vector<std::string> package_paths_;
};

}  // namespace r3v_ros
