#include "r3v_ros/cloud_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "sensor_msgs/msg/point_field.hpp"

#include "r3v_ros/frames.hpp"

namespace r3v_ros {

namespace {

using sensor_msgs::msg::PointField;

size_t datatypeSize(uint8_t datatype) {
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

template<typename T>
double readAs(const uint8_t* p) {
  T value;
  std::memcpy(&value, p, sizeof(T));
  return static_cast<double>(value);
}

// One numeric field of a PointCloud2 point, of any datatype.
class FieldReader
{
public:
  FieldReader(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
    const auto it = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [&](const PointField& field) { return field.name == name; });
    if (it == cloud.fields.end()) {
      throw std::runtime_error("cloud has no field '" + name + "'");
    }
    const size_t size = datatypeSize(it->datatype);
    if (size == 0 || it->offset + size > cloud.point_step) {
      throw std::runtime_error("field '" + name + "' has an invalid datatype or offset");
    }
    offset_ = it->offset;
    datatype_ = it->datatype;
  }

  double read(const uint8_t* point) const {
    const uint8_t* p = point + offset_;
    switch (datatype_) {
      case PointField::INT8: return readAs<int8_t>(p);
      case PointField::UINT8: return readAs<uint8_t>(p);
      case PointField::INT16: return readAs<int16_t>(p);
      case PointField::UINT16: return readAs<uint16_t>(p);
      case PointField::INT32: return readAs<int32_t>(p);
      case PointField::UINT32: return readAs<uint32_t>(p);
      case PointField::FLOAT32: return readAs<float>(p);
      default: return readAs<double>(p);
    }
  }

private:
  uint32_t offset_ = 0;
  uint8_t datatype_ = PointField::FLOAT32;
};

}  // namespace

void extractPoints(
  const sensor_msgs::msg::PointCloud2& cloud, const CloudConfig& config,
  const Eigen::Isometry3d& cloud_to_fixed, const Eigen::Vector3d& sensor_origin,
  std::vector<rv::PointF>& out) {
  out.clear();
  if (cloud.is_bigendian) {
    throw std::runtime_error("big endian clouds are not supported");
  }
  const size_t width = cloud.width;
  const size_t height = cloud.height;
  if (width * height == 0) {
    return;
  }
  if (static_cast<size_t>(cloud.point_step) * width > cloud.row_step ||
    static_cast<size_t>(cloud.row_step) * height > cloud.data.size()) {
    throw std::runtime_error("cloud data is smaller than its width, height and steps");
  }

  const FieldReader x(cloud, "x");
  const FieldReader y(cloud, "y");
  const FieldReader z(cloud, "z");
  const bool has_intensity = !config.intensity_field.empty();
  const FieldReader intensity(cloud, has_intensity ? config.intensity_field : "x");

  const double min_range_sq = config.min_range * config.min_range;
  const double max_range_sq = config.max_range * config.max_range;
  const size_t step = static_cast<size_t>(std::max(1, config.decimation));
  out.reserve(width * height / step);

  for (size_t index = 0; index < width * height; index += step) {
    const uint8_t* point =
      cloud.data.data() + (index / width) * cloud.row_step + (index % width) * cloud.point_step;
    const Eigen::Vector3d local(x.read(point), y.read(point), z.read(point));
    if (!local.allFinite()) {
      // Non-finite points would make the voxelizer's rays never end.
      continue;
    }
    const Eigen::Vector3d fixed = cloud_to_fixed * local;
    const double range_sq = (fixed - sensor_origin).squaredNorm();
    if (range_sq < min_range_sq || range_sq > max_range_sq) {
      continue;
    }
    double value = config.intensity_scale;
    if (has_intensity) {
      value *= intensity.read(point);
    }
    if (!std::isfinite(value) || value <= 0.0) {
      continue;
    }
    // The voxelizer multiplies intensities as u32; keep products far from
    // overflowing.
    value = std::min(value, 65535.0);
    const Eigen::Vector3d world = toRender(fixed);
    out.push_back(
      { static_cast<float>(world.x()), static_cast<float>(world.y()),
        static_cast<float>(world.z()), static_cast<float>(value) });
  }
}

}  // namespace r3v_ros
