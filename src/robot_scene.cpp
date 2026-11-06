#include "r3v_ros/robot_scene.hpp"

#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#if __has_include("tf2_eigen/tf2_eigen.hpp")
#include "tf2_eigen/tf2_eigen.hpp"
#else
#include "tf2_eigen/tf2_eigen.h"
#endif
#if __has_include("urdf/model.hpp")
#include "urdf/model.hpp"
#else
#include "urdf/model.h"
#endif

#include "r3v_ros/frames.hpp"

namespace r3v_ros {

namespace {

Eigen::Isometry3d toEigen(const urdf::Pose& pose) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  out.linear() =
    Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z)
    .normalized().toRotationMatrix();
  return out;
}

rr::R3vTwist toTwist(const Eigen::Isometry3d& pose) {
  const Eigen::Quaterniond q(pose.linear());
  const Eigen::Vector3d t = pose.translation();
  return {
    { static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()) },
    { static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
      static_cast<float>(q.w()) } };
}

std::string vectorKey(const urdf::Vector3& v) {
  std::ostringstream out;
  out << v.x << ',' << v.y << ',' << v.z;
  return out.str();
}

}  // namespace

RobotScene::RobotScene(
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock, MeshLoader loader, bool double_sided)
: logger_(std::move(logger)), clock_(std::move(clock)), loader_(std::move(loader)),
  double_sided_(double_sided) {
}

std::string RobotScene::geometryKey(const urdf::Geometry& geometry) const {
  switch (geometry.type) {
    case urdf::Geometry::BOX:
      return "box:" + vectorKey(static_cast<const urdf::Box&>(geometry).dim);
    case urdf::Geometry::CYLINDER: {
        const auto& cylinder = static_cast<const urdf::Cylinder&>(geometry);
        return "cylinder:" + std::to_string(cylinder.radius) + ',' +
               std::to_string(cylinder.length);
      }
    case urdf::Geometry::SPHERE:
      return "sphere:" + std::to_string(static_cast<const urdf::Sphere&>(geometry).radius);
    case urdf::Geometry::MESH: {
        const auto& mesh = static_cast<const urdf::Mesh&>(geometry);
        return "mesh:" + mesh.filename + ':' + vectorKey(mesh.scale);
      }
    default:
      return "unknown:" + std::to_string(geometry.type);
  }
}

const Triangles* RobotScene::geometryTriangles(
  const urdf::Geometry& geometry, const std::string& key) {
  if (const auto it = cache_.find(key); it != cache_.end()) {
    return &it->second;
  }
  if (failed_.count(key) != 0) {
    return nullptr;
  }

  Triangles out;
  switch (geometry.type) {
    case urdf::Geometry::BOX: {
        const auto& dim = static_cast<const urdf::Box&>(geometry).dim;
        out = makeBox({ dim.x, dim.y, dim.z });
        break;
      }
    case urdf::Geometry::CYLINDER: {
        const auto& cylinder = static_cast<const urdf::Cylinder&>(geometry);
        out = makeCylinder(cylinder.radius, cylinder.length);
        break;
      }
    case urdf::Geometry::SPHERE:
      out = makeSphere(static_cast<const urdf::Sphere&>(geometry).radius);
      break;
    case urdf::Geometry::MESH: {
        const auto& mesh = static_cast<const urdf::Mesh&>(geometry);
        const std::string path = loader_.resolve(mesh.filename);
        if (path.empty()) {
          RCLCPP_WARN(
            logger_, "Cannot find mesh '%s'; add the directory holding its package to "
            "robot_description.package_paths", mesh.filename.c_str());
          failed_.insert(key);
          return nullptr;
        }
        try {
          out = loader_.load(path);
        } catch (const std::exception& e) {
          RCLCPP_WARN(logger_, "Cannot load mesh '%s': %s", path.c_str(), e.what());
          failed_.insert(key);
          return nullptr;
        }
        scaleTriangles(out, { mesh.scale.x, mesh.scale.y, mesh.scale.z });
        break;
      }
    default:
      RCLCPP_WARN(logger_, "Skipping visuals of unsupported geometry type %d", geometry.type);
      failed_.insert(key);
      return nullptr;
  }
  if (double_sided_) {
    makeDoubleSided(out);
  }
  return &cache_.emplace(key, std::move(out)).first->second;
}

void RobotScene::addRenderer(rr::R3vWgpuRenderer* renderer) {
  targets_.push_back({ renderer, { } });
  upload(targets_.back());
}

void RobotScene::load(const std::string& urdf_xml) {
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("cannot parse robot description");
  }

  for (auto& target : targets_) {
    for (const auto& drawn : target.drawn) {
      if (drawn.visible) {
        checkStatus(
          rr::r3v_wgpu_renderer_set_asset_visibility(target.renderer, drawn.id, false),
          "hiding a previous robot visual");
      }
    }
    target.drawn.clear();
  }
  instances_.clear();
  children_.clear();
  root_ = model.getRoot() ? model.getRoot()->name : "";

  // Visuals sharing a geometry become instances of one uploaded asset.
  std::map<std::string, std::vector<Instance>> by_geometry;
  size_t visual_count = 0;
  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);
  for (const auto& link : links) {
    auto& children = children_[link->name];
    for (const auto& child : link->child_links) {
      children.push_back(child->name);
    }
    for (const auto& visual : link->visual_array) {
      if (!visual || !visual->geometry) {
        continue;
      }
      ++visual_count;
      const std::string key = geometryKey(*visual->geometry);
      if (geometryTriangles(*visual->geometry, key) == nullptr) {
        continue;
      }
      by_geometry[key].push_back({ link->name, toEigen(visual->origin), key });
    }
  }
  for (auto& [key, instances] : by_geometry) {
    for (auto& instance : instances) {
      instances_.push_back(std::move(instance));
    }
  }
  for (auto& target : targets_) {
    upload(target);
  }
  RCLCPP_INFO(
    logger_, "Loaded robot '%s': %zu of %zu visuals, %zu distinct meshes",
    model.getName().c_str(), instances_.size(), visual_count, by_geometry.size());
  unknown_hidden_.clear();
  updateHidden();
}

void RobotScene::upload(Target& target) {
  target.drawn.assign(instances_.size(), Drawn{ 0, false });
  size_t begin = 0;
  while (begin < instances_.size()) {
    const std::string& key = instances_[begin].geometry;
    size_t end = begin + 1;
    while (end < instances_.size() && instances_[end].geometry == key) {
      ++end;
    }
    const Triangles& triangles = cache_.at(key);
    MeshPtr mesh(rr::r3v_mesh_from_triangles(triangles.data(), triangles.size()));
    if (!mesh) {
      throw std::runtime_error("creating mesh '" + key + "' failed: " + lastRendererError());
    }
    std::vector<uint64_t> ids(end - begin);
    checkStatus(
      rr::r3v_wgpu_renderer_upload_asset(target.renderer, mesh.get(), ids.size(), ids.data()),
      "uploading mesh '" + key + "'");
    for (size_t i = 0; i < ids.size(); ++i) {
      target.drawn[begin + i].id = ids[i];
      checkStatus(
        rr::r3v_wgpu_renderer_set_asset_visibility(target.renderer, ids[i], false),
        "hiding a new robot visual");
    }
    begin = end;
  }
}

void RobotScene::setHiddenLinks(const std::vector<std::string>& links) {
  hidden_roots_ = links;
  unknown_hidden_.clear();
  updateHidden();
}

void RobotScene::updateHidden() {
  hidden_.clear();
  if (children_.empty()) {
    return;  // no robot yet, load() expands them
  }
  std::vector<std::string> stack;
  for (const auto& root : hidden_roots_) {
    if (children_.count(root) == 0) {
      if (unknown_hidden_.insert(root).second) {
        RCLCPP_WARN(logger_, "Cannot hide link '%s': the robot has no such link", root.c_str());
      }
      continue;
    }
    stack.push_back(root);
  }
  while (!stack.empty()) {
    std::string link = std::move(stack.back());
    stack.pop_back();
    if (!hidden_.insert(link).second) {
      continue;
    }
    const auto& children = children_.at(link);
    stack.insert(stack.end(), children.begin(), children.end());
  }
}

void RobotScene::update(const tf2_ros::Buffer& tf, const std::string& fixed_frame) {
  if (targets_.empty()) {
    return;
  }
  // Shows or hides instance `index` in every renderer.
  const auto set_visible = [this](size_t index, bool visible) {
      for (auto& target : targets_) {
        Drawn& drawn = target.drawn[index];
        if (drawn.visible != visible) {
          checkStatus(
            rr::r3v_wgpu_renderer_set_asset_visibility(target.renderer, drawn.id, visible),
            visible ? "showing a robot visual" : "hiding a robot visual");
          drawn.visible = visible;
        }
      }
    };
  std::string missing;
  for (size_t index = 0; index < instances_.size(); ++index) {
    const Instance& instance = instances_[index];
    if (hidden_.count(instance.link) != 0) {
      set_visible(index, false);
      continue;
    }
    const auto link_to_fixed = lookupLatest(tf, fixed_frame, instance.link);
    if (!link_to_fixed) {
      missing = instance.link;
      continue;
    }
    const rr::R3vTwist pose = toTwist(toRender(*link_to_fixed * instance.origin));
    for (const auto& target : targets_) {
      checkStatus(
        rr::r3v_wgpu_renderer_set_asset_transform(target.renderer, target.drawn[index].id, pose),
        "posing a robot visual");
    }
    set_visible(index, true);
  }
  if (!missing.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "No transform from '%s' to '%s' yet; its visuals keep their "
      "last pose", missing.c_str(), fixed_frame.c_str());
  }
}

std::optional<Eigen::Isometry3d> RobotScene::lookupLatest(
  const tf2_ros::Buffer& tf, const std::string& fixed_frame, const std::string& frame,
  std::string* error) const {
  const auto latest = [&tf](const std::string& target, const std::string& source) {
      return target == source ? Eigen::Isometry3d::Identity() :
             tf2::transformToEigen(tf.lookupTransform(target, source, tf2::TimePointZero));
    };
  if (frame.empty() || fixed_frame.empty() || frame == fixed_frame) {
    return Eigen::Isometry3d::Identity();
  }
  try {
    if (root_.empty() || children_.count(frame) == 0) {
      return latest(fixed_frame, frame);  // not a link of the robot
    }
    return latest(fixed_frame, root_) * latest(root_, frame);
  } catch (const tf2::TransformException& e) {
    if (error != nullptr) {
      *error = e.what();
    }
    return std::nullopt;
  }
}

}  // namespace r3v_ros
