#include "r3v_ros/voxelizer_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#if __has_include("tf2_eigen/tf2_eigen.hpp")
#include "tf2_eigen/tf2_eigen.hpp"
#else
#include "tf2_eigen/tf2_eigen.h"
#endif

#include "r3v_ros/frames.hpp"

namespace r3v_ros {

namespace {

using msg::VoxelMapUpdate;

// The stream carries what the voxelizer returns as it is.
static_assert(
  sizeof(rv::PointI) == sizeof(WireBlock) && offsetof(rv::PointI, i) == offsetof(WireBlock, mask),
  "r3v-voxelizer PointI must have the layout of WireBlock");
static_assert(
  sizeof(rv::VoxelGroup) == sizeof(WireGroup) &&
  offsetof(rv::VoxelGroup, values) == offsetof(WireGroup, pages) &&
  rv::POINTS_IN_VOXEL_GROUP == kGroupPages,
  "r3v-voxelizer VoxelGroup must have the layout of WireGroup");

bool sameMultipliers(const rv::Multipliers& a, const rv::Multipliers& b) {
  return a.intensity == b.intensity && a.hit == b.hit && a.target == b.target;
}

uint64_t randomSession() {
  std::random_device device;
  std::seed_seq seed{ device(), device(), device(), device() };
  std::mt19937_64 engine(seed);
  return engine();
}

}  // namespace

VoxelizerNode::VoxelizerNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("r3v_voxelizer", options), session_(randomSession()) {
  declareParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // TF is fed from its own callback group, so lookups may wait for it.
  tf_buffer_->setUsingDedicatedThread(true);

  voxelizer_ = makeVoxelizer();
  createInterfaces();
  RCLCPP_INFO(
    get_logger(), "Voxelizing %zu point clouds in '%s' (%s backend), publishing to '%s' at "
    "%.1f Hz, session %016" PRIx64, cloud_configs_.size(), fixed_frame_.c_str(),
    voxelizer_backend_.c_str(), stream_pub_->get_topic_name(), stream_rate_, session_);
}

void VoxelizerNode::declareParameters() {
  fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
  tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
  tf_static_topic_ = declare_parameter<std::string>("tf_static_topic", "/tf_static");
  tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
  require(!fixed_frame_.empty(), "fixed_frame must not be empty");
  require(tf_timeout_ >= 0.0, "tf_timeout must not be negative");

  stream_topic_ = declare_parameter<std::string>("voxel_stream.topic", "/r3v/voxels");
  const int64_t depth = declare_parameter<int64_t>("voxel_stream.depth", 100);
  stream_rate_ = declare_parameter<double>("voxel_stream.rate", 15.0);
  const std::string compression =
    declare_parameter<std::string>("voxel_stream.compression", "zstd");
  const int64_t level = declare_parameter<int64_t>("voxel_stream.compression_level", 1);
  heartbeat_period_ = declare_parameter<double>("voxel_stream.heartbeat_period", 1.0);
  require(!stream_topic_.empty(), "voxel_stream.topic must not be empty");
  require(depth >= 1 && depth <= 100000, "voxel_stream.depth must be between 1 and 100000");
  require(stream_rate_ > 0.0, "voxel_stream.rate must be positive");
  require(
    compression == "zstd" || compression == "none",
    "voxel_stream.compression must be zstd or none");
  require(
    level >= std::numeric_limits<int>::min() && level <= std::numeric_limits<int>::max() &&
    validCompressionLevel(static_cast<int>(level)),
    "voxel_stream.compression_level is out of the range zstd takes");
  require(heartbeat_period_ > 0.0, "voxel_stream.heartbeat_period must be positive");
  stream_depth_ = static_cast<size_t>(depth);
  compression_ = compression == "zstd" ?
    VoxelMapUpdate::COMPRESSION_ZSTD : VoxelMapUpdate::COMPRESSION_NONE;
  compression_level_ = static_cast<int>(level);

  settings_.resolution = declare_parameter<double>("voxelizer.resolution", 0.1);
  require(
    validPositiveFloat(settings_.resolution),
    "voxelizer.resolution must be positive and fit in a float");
  multipliers_.intensity =
    static_cast<float>(declare_parameter<double>("voxelizer.intensity_multiplier", 1.0));
  multipliers_.hit = declareU32(*this, "voxelizer.hit_multiplier", 1);
  multipliers_.target = declareU32(*this, "voxelizer.target_multiplier", 2);
  voxelizer_backend_ = declare_parameter<std::string>("voxelizer.backend", "multi_threaded");
  voxelizer_threads_ = declare_parameter<int64_t>("voxelizer.threads", 0);
  voxelizer_buckets_ = declare_parameter<int64_t>("voxelizer.wgpu_bucket_count", 131072);
  settings_.destroy_threshold = declareU32(*this, "voxelizer.destroy_threshold", 0);
  settings_.construct_threshold = declareU32(*this, "voxelizer.construct_threshold", 2);
  settings_.max_value = declareU32(*this, "voxelizer.max_value", 10);
  require(voxelizer_threads_ >= 0, "voxelizer.threads must not be negative");
  require(
    voxelizer_buckets_ > 0 && voxelizer_buckets_ <= std::numeric_limits<uint32_t>::max(),
    "voxelizer.wgpu_bucket_count must be positive and fit in 32 bits");
  require(
    voxelizer_backend_ == "single_threaded" || voxelizer_backend_ == "multi_threaded" ||
    voxelizer_backend_ == "wgpu",
    "voxelizer.backend must be single_threaded, multi_threaded or wgpu");

  const auto names = declare_parameter<std::vector<std::string>>(
    "point_clouds.names", std::vector<std::string>{ });
  std::set<std::string> seen;
  for (const auto& name : names) {
    if (name.empty()) {
      continue;
    }
    require(seen.insert(name).second, "point_clouds.names lists '" + name + "' twice");
    const std::string prefix = "point_clouds." + name + ".";
    CloudConfig config;
    config.name = name;
    config.topic = declare_parameter<std::string>(prefix + "topic", "");
    config.qos = declare_parameter<std::string>(prefix + "qos", "sensor_data");
    config.sensor_frame = declare_parameter<std::string>(prefix + "sensor_frame", "");
    config.min_range = declare_parameter<double>(prefix + "min_range", 0.0);
    config.max_range = declare_parameter<double>(prefix + "max_range", 30.0);
    config.decimation = static_cast<int>(declare_parameter<int64_t>(prefix + "decimation", 1));
    config.intensity_field = declare_parameter<std::string>(prefix + "intensity_field", "");
    config.intensity_scale = declare_parameter<double>(prefix + "intensity_scale", 1.0);
    rv::Multipliers multipliers;
    multipliers.intensity = static_cast<float>(
      declare_parameter<double>(prefix + "intensity_multiplier", multipliers_.intensity));
    multipliers.hit = declareU32(*this, prefix + "hit_multiplier", multipliers_.hit);
    multipliers.target = declareU32(*this, prefix + "target_multiplier", multipliers_.target);
    require(!config.topic.empty(), prefix + "topic must be set");
    require(
      config.qos == "sensor_data" || config.qos == "reliable",
      prefix + "qos must be sensor_data or reliable");
    require(
      config.min_range >= 0.0 && config.max_range > config.min_range,
      prefix + "needs 0 <= min_range < max_range");
    require(config.decimation >= 1, prefix + "decimation must be at least 1");
    cloud_configs_.push_back(std::move(config));
    settings_.multipliers.push_back(multipliers);
  }
}

VoxelizerPtr VoxelizerNode::makeVoxelizer() const {
  const auto resolution = static_cast<float>(settings_.resolution);
  // Every cloud sets its own before adding points.
  const rv::Multipliers& m = multipliers_;
  const uint32_t destroy = settings_.destroy_threshold;
  const uint32_t construct = settings_.construct_threshold;
  const uint32_t max_value = settings_.max_value;
  // compress = true: blocks of 4x4x2 voxels, what the tesselator takes.
  void* voxelizer = nullptr;
  if (voxelizer_backend_ == "single_threaded") {
    voxelizer = rv::create_single_threaded_voxelizer(
      resolution, m.intensity, m.hit, m.target, destroy, construct, max_value, true);
  } else if (voxelizer_backend_ == "multi_threaded") {
    voxelizer = rv::create_multi_threaded_voxelizer(
      resolution, m.intensity, m.hit, m.target, destroy, construct, max_value, true,
      static_cast<uintptr_t>(voxelizer_threads_));
  } else {
    voxelizer = rv::create_wgpu_voxelizer(
      resolution, m.intensity, m.hit, m.target, destroy, construct, max_value, true,
      static_cast<uint32_t>(voxelizer_buckets_));
  }
  if (voxelizer == nullptr) {
    throw std::runtime_error(
            "creating the " + voxelizer_backend_ + " voxelizer failed" +
            (voxelizer_backend_ == "wgpu" ?
            " (no GPU adapter able to run compute shaders, or wgpu_bucket_count too big)" : ""));
  }
  return VoxelizerPtr(voxelizer);
}

void VoxelizerNode::createInterfaces() {
  tf_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  voxel_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_subs_ = subscribeTf(*this, *tf_buffer_, tf_topic_, tf_static_topic_, tf_group_);

  rclcpp::SubscriptionOptions cloud_options;
  cloud_options.callback_group = voxel_group_;
  for (size_t index = 0; index < cloud_configs_.size(); ++index) {
    const CloudConfig& config = cloud_configs_[index];
    const rclcpp::QoS qos =
      config.qos == "reliable" ? rclcpp::QoS(10) : rclcpp::QoS(rclcpp::SensorDataQoS());
    cloud_subs_.push_back(
      create_subscription<sensor_msgs::msg::PointCloud2>(
        config.topic, qos,
        [this, index](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          onCloud(index, *msg);
        }, cloud_options));
    RCLCPP_INFO(
      get_logger(), "Point cloud '%s' from '%s'", config.name.c_str(),
      cloud_subs_.back()->get_topic_name());
  }

  // Control topics only queue their change for the next onTick(). In the
  // voxel group they could wait long: a point cloud always ready keeps the
  // executor busy with it.
  rclcpp::SubscriptionOptions control_options;
  control_options.callback_group = control_group_;
  clear_voxels_sub_ = create_subscription<std_msgs::msg::Empty>(
    "~/clear_voxels", rclcpp::QoS(10),
    [this](std_msgs::msg::Empty::ConstSharedPtr) {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.clear_voxels = true;
    }, control_options);
  sync_request_sub_ = create_subscription<msg::FullSyncRequest>(
    "~/request_full_sync", rclcpp::QoS(10),
    [this](msg::FullSyncRequest::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.sync_requests.push_back(*msg);
    }, control_options);
  createSettingInterfaces(control_options);

  rclcpp::PublisherOptions stream_options;
  // Straight to renderers in the same process, as the combined launch file
  // runs them; others get it through the middleware as usual.
  stream_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  stream_pub_ = create_publisher<VoxelMapUpdate>(
    stream_topic_, voxelStreamQos(stream_depth_), stream_options);
  last_publish_ = Clock::now();
  tick_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / stream_rate_), [this]() { onTick(); }, voxel_group_);
}

void VoxelizerNode::createSettingInterfaces(const rclcpp::SubscriptionOptions& options) {
  // `set` queues the value in controls_. Float values failing `valid` are
  // dropped with a warning naming `what` and `requirement`.
  const auto float_setter = [this, &options](
    const std::string& topic, const std::string& what, bool (*valid)(double),
    const std::string& requirement, auto set) {
      return create_subscription<std_msgs::msg::Float64>(
        topic, rclcpp::QoS(10),
        [this, what, valid, requirement, set](std_msgs::msg::Float64::ConstSharedPtr msg) {
          if (!valid(msg->data)) {
            RCLCPP_WARN(
              get_logger(), "Ignoring %s %g: %s", what.c_str(), msg->data, requirement.c_str());
            return;
          }
          std::lock_guard<std::mutex> lock(controls_mutex_);
          set(controls_, msg->data);
        }, options);
    };
  const auto u32_setter = [this, &options](const std::string& topic, auto set) {
      return create_subscription<std_msgs::msg::UInt32>(
        topic, rclcpp::QoS(10),
        [this, set](std_msgs::msg::UInt32::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(controls_mutex_);
          set(controls_, msg->data);
        }, options);
    };

  resolution_sub_ = float_setter(
    "~/set_resolution", "voxelizer resolution", validPositiveFloat,
    "must be positive and fit in a float",
    [](Controls& controls, double value) { controls.resolution = value; });
  destroy_threshold_sub_ = u32_setter(
    "~/set_destroy_threshold",
    [](Controls& controls, uint32_t value) { controls.destroy_threshold = value; });
  construct_threshold_sub_ = u32_setter(
    "~/set_construct_threshold",
    [](Controls& controls, uint32_t value) { controls.construct_threshold = value; });
  max_value_sub_ = u32_setter(
    "~/set_max_value",
    [](Controls& controls, uint32_t value) { controls.max_value = value; });

  const rclcpp::QoS state_qos = rclcpp::QoS(1).reliable().transient_local();
  resolution_pub_ = create_publisher<std_msgs::msg::Float64>("~/get_resolution", state_qos);
  destroy_threshold_pub_ =
    create_publisher<std_msgs::msg::UInt32>("~/get_destroy_threshold", state_qos);
  construct_threshold_pub_ =
    create_publisher<std_msgs::msg::UInt32>("~/get_construct_threshold", state_qos);
  max_value_pub_ = create_publisher<std_msgs::msg::UInt32>("~/get_max_value", state_qos);

  for (size_t index = 0; index < cloud_configs_.size(); ++index) {
    const std::string& name = cloud_configs_[index].name;
    const std::string prefix = "~/point_clouds/" + name + "/";
    MultiplierTopics topics;
    topics.intensity_sub = float_setter(
      prefix + "set_intensity_multiplier", "point cloud '" + name + "' intensity multiplier",
      validFloat, "must be finite and fit in a float",
      [index](Controls& controls, double value) {
        controls.multipliers[index].intensity = static_cast<float>(value);
      });
    topics.hit_sub = u32_setter(
      prefix + "set_hit_multiplier",
      [index](Controls& controls, uint32_t value) { controls.multipliers[index].hit = value; });
    topics.target_sub = u32_setter(
      prefix + "set_target_multiplier",
      [index](Controls& controls, uint32_t value) {
        controls.multipliers[index].target = value;
      });
    topics.intensity_pub =
      create_publisher<std_msgs::msg::Float64>(prefix + "get_intensity_multiplier", state_qos);
    topics.hit_pub =
      create_publisher<std_msgs::msg::UInt32>(prefix + "get_hit_multiplier", state_qos);
    topics.target_pub =
      create_publisher<std_msgs::msg::UInt32>(prefix + "get_target_multiplier", state_qos);
    multiplier_topics_.push_back(std::move(topics));
  }
  publishSettings();
}

std::optional<Eigen::Isometry3d> VoxelizerNode::lookup(
  const std::string& frame, const rclcpp::Time& stamp, std::string* error) const {
  if (frame.empty() || frame == fixed_frame_) {
    return Eigen::Isometry3d::Identity();
  }
  try {
    return tf2::transformToEigen(
      tf_buffer_->lookupTransform(
        fixed_frame_, frame, stamp, rclcpp::Duration::from_seconds(tf_timeout_)));
  } catch (const tf2::TransformException& e) {
    if (error != nullptr) {
      *error = e.what();
    }
    return std::nullopt;
  }
}

void VoxelizerNode::onCloud(size_t index, const sensor_msgs::msg::PointCloud2& cloud) {
  const CloudConfig& config = cloud_configs_[index];
  if (!voxelizer_) {
    return;  // onTick() keeps trying to make one
  }
  const rclcpp::Time stamp(cloud.header.stamp);
  std::string error;
  const auto cloud_to_fixed = lookup(cloud.header.frame_id, stamp, &error);
  if (!cloud_to_fixed) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping point cloud '%s': %s", config.name.c_str(),
      error.c_str());
    return;
  }
  Eigen::Vector3d sensor = cloud_to_fixed->translation();
  if (!config.sensor_frame.empty() && config.sensor_frame != cloud.header.frame_id) {
    const auto sensor_to_fixed = lookup(config.sensor_frame, stamp, &error);
    if (!sensor_to_fixed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping point cloud '%s': %s",
        config.name.c_str(), error.c_str());
      return;
    }
    sensor = sensor_to_fixed->translation();
  }

  try {
    extractPoints(cloud, config, *cloud_to_fixed, sensor, points_);
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping point cloud '%s': %s", config.name.c_str(),
      e.what());
    return;
  }
  if (points_.empty()) {
    return;
  }

  const Eigen::Vector3d origin = toRender(sensor);
  const rv::PointF sensor_pos{
    static_cast<float>(origin.x()), static_cast<float>(origin.y()),
    static_cast<float>(origin.z()), 0.0f };
  // Clouds share the voxelizer, each with its own multipliers. The voxel
  // group runs one callback at a time, so nothing runs in between.
  rv::set_multipliers(voxelizer_.get(), settings_.multipliers[index]);
  const OwnedUpdate update(
    rv::add_points(voxelizer_.get(), sensor_pos, points_.data(), points_.size()));
  // Blocks carry their whole occupancy mask, so the last one wins.
  for (size_t i = 0; i < update->len; ++i) {
    const rv::PointI& block = update->data[i];
    pending_[VoxelKey{ block.x, block.y, block.z }] = block.i;
  }
}

void VoxelizerNode::onTick() {
  try {
    applyControls();
    if (!voxelizer_) {
      try {
        voxelizer_ = makeVoxelizer();
      } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "%s; point clouds are dropped", e.what());
      }
    }
    if (!hasSubscribers()) {
      // Nobody to keep in sync. Whoever subscribes next starts from the
      // whole map, without having to ask for it.
      pending_.clear();
      full_pending_ = true;
      return;
    }
    if (full_pending_) {
      publishFull();
    } else if (!pending_.empty()) {
      publishDelta();
    } else if (Clock::now() - last_publish_ >= std::chrono::duration<double>(heartbeat_period_)) {
      // Renderers that just subscribed learn the sequence to ask for a FULL
      // message from, and a restart of this node.
      publishUpdate(VoxelMapUpdate::DELTA, nullptr, 0);
    }
  } catch (const std::exception& e) {
    // Only compressing can fail; the blocks stay pending for the next tick.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Publishing the voxel stream failed: %s", e.what());
  }
}

void VoxelizerNode::applyControls() {
  Controls controls;
  {
    std::lock_guard<std::mutex> lock(controls_mutex_);
    std::swap(controls, controls_);
  }
  VoxelizerSettings settings = settings_;
  settings.resolution = controls.resolution.value_or(settings.resolution);
  settings.destroy_threshold = controls.destroy_threshold.value_or(settings.destroy_threshold);
  settings.construct_threshold =
    controls.construct_threshold.value_or(settings.construct_threshold);
  settings.max_value = controls.max_value.value_or(settings.max_value);
  for (const auto& [index, change] : controls.multipliers) {
    rv::Multipliers& m = settings.multipliers[index];
    m.intensity = change.intensity.value_or(m.intensity);
    m.hit = change.hit.value_or(m.hit);
    m.target = change.target.value_or(m.target);
  }
  // A change clears the map too, so with a clear arriving together the map
  // is cleared once.
  if (!setVoxelizerSettings(settings) && controls.clear_voxels) {
    clearVoxels();
  }

  for (const auto& request : controls.sync_requests) {
    // A request about another session (of a previous run) is answered by
    // the stream itself: the renderer sees the new session and asks again.
    if (request.session != session_) {
      continue;
    }
    // A FULL message from `since` on is already on its way to the renderer.
    if (last_full_ && *last_full_ >= request.since) {
      continue;
    }
    full_pending_ = true;
  }
}

void VoxelizerNode::clearVoxels() {
  // The old one goes first, a wgpu one holds GPU memory.
  voxelizer_.reset();
  pending_.clear();
  // Tells renderers the map is empty, also at a new resolution.
  full_pending_ = true;
  try {
    voxelizer_ = makeVoxelizer();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s; trying again", e.what());
  }
  RCLCPP_INFO(get_logger(), "Cleared the voxel map");
}

bool VoxelizerNode::setVoxelizerSettings(const VoxelizerSettings& settings) {
  const VoxelizerSettings& old = settings_;
  const bool same_multipliers = std::equal(
    settings.multipliers.begin(), settings.multipliers.end(), old.multipliers.begin(),
    old.multipliers.end(), sameMultipliers);
  if (settings.resolution == old.resolution &&
    settings.destroy_threshold == old.destroy_threshold &&
    settings.construct_threshold == old.construct_threshold &&
    settings.max_value == old.max_value && same_multipliers) {
    return false;
  }
  settings_ = settings;
  clearVoxels();
  publishSettings();
  RCLCPP_INFO(
    get_logger(), "Voxelizer: resolution %g m, destroy threshold %u, construct threshold %u, "
    "max value %u", settings.resolution, settings.destroy_threshold,
    settings.construct_threshold, settings.max_value);
  for (size_t index = 0; index < cloud_configs_.size(); ++index) {
    const rv::Multipliers& m = settings.multipliers[index];
    RCLCPP_INFO(
      get_logger(), "Point cloud '%s': intensity multiplier %g, hit multiplier %u, "
      "target multiplier %u", cloud_configs_[index].name.c_str(),
      static_cast<double>(m.intensity), m.hit, m.target);
  }
  return true;
}

void VoxelizerNode::publishSettings() {
  std_msgs::msg::Float64 f64;
  std_msgs::msg::UInt32 u32;
  f64.data = settings_.resolution;
  resolution_pub_->publish(f64);
  u32.data = settings_.destroy_threshold;
  destroy_threshold_pub_->publish(u32);
  u32.data = settings_.construct_threshold;
  construct_threshold_pub_->publish(u32);
  u32.data = settings_.max_value;
  max_value_pub_->publish(u32);
  for (size_t index = 0; index < multiplier_topics_.size(); ++index) {
    const rv::Multipliers& m = settings_.multipliers[index];
    const MultiplierTopics& topics = multiplier_topics_[index];
    f64.data = m.intensity;
    topics.intensity_pub->publish(f64);
    u32.data = m.hit;
    topics.hit_pub->publish(u32);
    u32.data = m.target;
    topics.target_pub->publish(u32);
  }
}

bool VoxelizerNode::hasSubscribers() const {
  return stream_pub_->get_subscription_count() > 0 ||
         stream_pub_->get_intra_process_subscription_count() > 0;
}

void VoxelizerNode::publishFull() {
  // Empty when making the voxelizer failed, as the map is.
  const OwnedVoxelGroups groups(rv::get_all_points_compressed(voxelizer_.get()));
  const uint64_t sequence = sequence_;
  const size_t size =
    publishUpdate(VoxelMapUpdate::FULL, groups->data, groups->len * sizeof(WireGroup));
  // It holds every block changed so far.
  last_full_ = sequence;
  pending_.clear();
  full_pending_ = false;
  RCLCPP_INFO(
    get_logger(), "Published the whole voxel map: %zu groups, %zu bytes (message %" PRIu64 ")",
    static_cast<size_t>(groups->len), size, sequence);
}

void VoxelizerNode::publishDelta() {
  std::vector<WireBlock> blocks;
  blocks.reserve(pending_.size());
  for (const auto& [key, mask] : pending_) {
    blocks.push_back({ key.x, key.y, key.z, mask });
  }
  publishUpdate(VoxelMapUpdate::DELTA, blocks.data(), blocks.size() * sizeof(WireBlock));
  pending_.clear();
}

size_t VoxelizerNode::publishUpdate(uint8_t kind, const void* data, size_t bytes) {
  auto msg = std::make_unique<VoxelMapUpdate>();
  msg->data = encodePayload(data, bytes, compression_, compression_level_);
  msg->header.stamp = now();
  msg->header.frame_id = fixed_frame_;
  msg->session = session_;
  msg->sequence = sequence_++;
  msg->kind = kind;
  msg->compression = compression_;
  msg->resolution = settings_.resolution;
  const size_t size = msg->data.size();
  stream_pub_->publish(std::move(msg));
  last_publish_ = Clock::now();
  return size;
}

}  // namespace r3v_ros

RCLCPP_COMPONENTS_REGISTER_NODE(r3v_ros::VoxelizerNode)
