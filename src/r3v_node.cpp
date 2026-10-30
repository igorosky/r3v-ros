#include "r3v_ros/r3v_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2_ros/qos.hpp"
#if __has_include("tf2_eigen/tf2_eigen.hpp")
#include "tf2_eigen/tf2_eigen.hpp"
#else
#include "tf2_eigen/tf2_eigen.h"
#endif

#include "r3v_ros/frames.hpp"

namespace r3v_ros {

namespace {

namespace enc = sensor_msgs::image_encodings;

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

bool validColor(double r, double g, double b) {
  const auto unit = [](double c) { return c >= 0.0 && c <= 1.0; };
  return unit(r) && unit(g) && unit(b);
}

// For values the libraries take as a positive float.
bool validPositiveFloat(double value) {
  return value > 0.0 && value <= std::numeric_limits<float>::max() &&
         static_cast<float>(value) > 0.0f;
}

bool validFloat(double value) {
  return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}

bool sameMultipliers(const rv::Multipliers& a, const rv::Multipliers& b) {
  return a.intensity == b.intensity && a.hit == b.hit && a.target == b.target;
}

}  // namespace

R3vNode::R3vNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("r3v_node", options) {
  declareParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // TF is fed from its own callback group, so lookups may wait for it.
  tf_buffer_->setUsingDedicatedThread(true);

  createVoxelizer();
  createRenderer();

  robot_ = std::make_unique<RobotScene>(
    get_logger(), get_clock(), MeshLoader(package_paths_), double_sided_);
  robot_->setHiddenLinks(hidden_links_);
  if (!robot_description_file_.empty()) {
    std::ifstream file(robot_description_file_);
    require(
      file.good(), "cannot open robot_description.file '" + robot_description_file_ + "'");
    std::stringstream urdf_xml;
    urdf_xml << file.rdbuf();
    robot_->load(urdf_xml.str(), renderer_.get());
  }

  createInterfaces();
  RCLCPP_INFO(
    get_logger(), "Rendering %ux%u at %.1f Hz to '%s' (fixed frame '%s', %zu point clouds)",
    width_, height_, rate_, image_pub_.getTopic().c_str(), fixed_frame_.c_str(),
    cloud_configs_.size());
}

void R3vNode::declareParameters() {
  fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
  rate_ = declare_parameter<double>("rate", 15.0);
  tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
  tf_static_topic_ = declare_parameter<std::string>("tf_static_topic", "/tf_static");
  tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
  require(!fixed_frame_.empty(), "fixed_frame must not be empty");
  require(rate_ > 0.0, "rate must be positive");
  require(tf_timeout_ >= 0.0, "tf_timeout must not be negative");

  robot_description_topic_ =
    declare_parameter<std::string>("robot_description.topic", "/robot_description");
  robot_description_file_ = declare_parameter<std::string>("robot_description.file", "");
  package_paths_ = declare_parameter<std::vector<std::string>>(
    "robot_description.package_paths", std::vector<std::string>{ });
  package_paths_.erase(
    std::remove(package_paths_.begin(), package_paths_.end(), ""), package_paths_.end());
  double_sided_ = declare_parameter<bool>("robot_description.double_sided", false);
  hidden_links_ = declare_parameter<std::vector<std::string>>(
    "robot_description.hidden_links", std::vector<std::string>{ });
  hidden_links_.erase(
    std::remove(hidden_links_.begin(), hidden_links_.end(), ""), hidden_links_.end());

  const auto vector3 = [this](const std::string& name, const std::vector<double>& fallback) {
      const auto v = declare_parameter<std::vector<double>>(name, fallback);
      require(v.size() == 3, name + " must have 3 elements");
      return Eigen::Vector3d(v[0], v[1], v[2]);
    };
  output_topic_ = declare_parameter<std::string>("output.topic", "/r3v/image");
  const int64_t width = declare_parameter<int64_t>("output.width", 1280);
  const int64_t height = declare_parameter<int64_t>("output.height", 720);
  encoding_ = declare_parameter<std::string>("output.encoding", enc::BGR8);
  output_frame_id_ = declare_parameter<std::string>("output.frame_id", "");
  lazy_ = declare_parameter<bool>("output.lazy", true);
  output_qos_ = declare_parameter<std::string>("output.qos", "sensor_data");
  require(
    output_qos_ == "sensor_data" || output_qos_ == "reliable",
    "output.qos must be sensor_data or reliable");
  require(
    width >= 1 && width <= 8192 && height >= 1 && height <= 8192,
    "output.width and output.height must be between 1 and 8192");
  width_ = static_cast<uint32_t>(width);
  height_ = static_cast<uint32_t>(height);
  require(
    encoding_ == enc::BGR8 || encoding_ == enc::RGB8 || encoding_ == enc::BGRA8 ||
    encoding_ == enc::RGBA8, "output.encoding must be bgr8, rgb8, bgra8 or rgba8");
  const Eigen::Vector3d background = vector3("output.background_color", { 0.0, 0.0, 0.0 });
  require(
    validColor(background.x(), background.y(), background.z()),
    "output.background_color components must be between 0 and 1");
  background_color_ = {
    static_cast<float>(background.x()), static_cast<float>(background.y()),
    static_cast<float>(background.z()) };

  camera_frame_ = declare_parameter<std::string>("camera.frame", "");
  camera_position_ = vector3("camera.position", { 4.0, 4.0, 3.0 });
  camera_target_ = vector3("camera.target", { 0.0, 0.0, 0.5 });
  lens_.fov_deg = declare_parameter<double>("camera.fov_deg", 60.0);
  lens_.near = declare_parameter<double>("camera.near", 0.05);
  lens_.far = declare_parameter<double>("camera.far", 200.0);
  camera_pose_topic_ = declare_parameter<std::string>("camera.pose_topic", "~/camera_pose");
  color_offset_ = declare_parameter<double>("camera.color_offset", 0.0);
  require(std::isfinite(color_offset_), "camera.color_offset must be finite");
  color_depth_range_ = declare_parameter<double>("camera.color_depth_range", 50.0);
  require(
    validPositiveFloat(color_depth_range_),
    "camera.color_depth_range must be positive and fit in a float");
  require(
    lens_.fov_deg > 0.0 && lens_.fov_deg < 180.0, "camera.fov_deg must be between 0 and 180");
  require(lens_.near > 0.0 && lens_.far > lens_.near, "camera needs 0 < near < far");

  voxelizer_settings_.resolution = declare_parameter<double>("voxelizer.resolution", 0.1);
  require(
    validPositiveFloat(voxelizer_settings_.resolution),
    "voxelizer.resolution must be positive and fit in a float");
  voxels_visible_ = declare_parameter<bool>("voxelizer.visible", true);
  marching_cubes_ = declare_parameter<bool>("voxelizer.marching_cubes", false);
  multipliers_.intensity =
    static_cast<float>(declare_parameter<double>("voxelizer.intensity_multiplier", 1.0));
  multipliers_.hit = declareU32("voxelizer.hit_multiplier", 1);
  multipliers_.target = declareU32("voxelizer.target_multiplier", 2);

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
    multipliers.hit = declareU32(prefix + "hit_multiplier", multipliers_.hit);
    multipliers.target = declareU32(prefix + "target_multiplier", multipliers_.target);
    require(!config.topic.empty(), prefix + "topic must be set");
    require(
      config.qos == "sensor_data" || config.qos == "reliable",
      prefix + "qos must be sensor_data or reliable");
    require(
      config.min_range >= 0.0 && config.max_range > config.min_range,
      prefix + "needs 0 <= min_range < max_range");
    require(config.decimation >= 1, prefix + "decimation must be at least 1");
    cloud_configs_.push_back(std::move(config));
    voxelizer_settings_.multipliers.push_back(multipliers);
  }
}

uint32_t R3vNode::declareU32(const std::string& name, int64_t fallback) {
  const int64_t v = declare_parameter<int64_t>(name, fallback);
  require(v >= 0 && v <= std::numeric_limits<uint32_t>::max(), name + " must fit in 32 bits");
  return static_cast<uint32_t>(v);
}

void R3vNode::createVoxelizer() {
  voxelizer_backend_ = declare_parameter<std::string>("voxelizer.backend", "multi_threaded");
  voxelizer_threads_ = declare_parameter<int64_t>("voxelizer.threads", 0);
  voxelizer_buckets_ = declare_parameter<int64_t>("voxelizer.wgpu_bucket_count", 131072);
  voxelizer_settings_.destroy_threshold = declareU32("voxelizer.destroy_threshold", 0);
  voxelizer_settings_.construct_threshold = declareU32("voxelizer.construct_threshold", 2);
  voxelizer_settings_.max_value = declareU32("voxelizer.max_value", 10);
  require(voxelizer_threads_ >= 0, "voxelizer.threads must not be negative");
  require(
    voxelizer_buckets_ > 0 && voxelizer_buckets_ <= std::numeric_limits<uint32_t>::max(),
    "voxelizer.wgpu_bucket_count must be positive and fit in 32 bits");
  require(
    voxelizer_backend_ == "single_threaded" || voxelizer_backend_ == "multi_threaded" ||
    voxelizer_backend_ == "wgpu",
    "voxelizer.backend must be single_threaded, multi_threaded or wgpu");
  active_settings_ = voxelizer_settings_;
  voxelizer_ = makeVoxelizer(active_settings_);
}

VoxelizerPtr R3vNode::makeVoxelizer(const VoxelizerSettings& settings) const {
  const auto resolution = static_cast<float>(settings.resolution);
  // Every cloud sets its own before adding points.
  const rv::Multipliers& m = multipliers_;
  const uint32_t destroy = settings.destroy_threshold;
  const uint32_t construct = settings.construct_threshold;
  const uint32_t max_value = settings.max_value;
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

void R3vNode::createRenderer() {
  renderer_.reset(rr::r3v_wgpu_renderer_create());
  if (!renderer_) {
    throw std::runtime_error(
            "creating the renderer failed, no usable GPU adapter (is a Vulkan driver "
            "installed and the GPU device available?): " + lastRendererError());
  }
  configureRenderer();
  frame_.resize(static_cast<size_t>(width_) * height_ * 4);

  tesselator_.reset(rr::r3v_cpu_tesselator_create());
  if (!tesselator_) {
    throw std::runtime_error("creating the tesselator failed: " + lastRendererError());
  }
  configureTesselator();
}

void R3vNode::configureRenderer() {
  // The same frame size keeps the frame the renderer holds.
  checkStatus(
    rr::r3v_wgpu_renderer_configure(
      renderer_.get(),
      { width_, height_, background_color_, static_cast<float>(color_depth_range_) }),
    "configuring the renderer");
}

void R3vNode::configureTesselator() {
  // Keeps the blocks the tesselator holds.
  checkStatus(
    rr::r3v_cpu_tesselator_configure(
      tesselator_.get(), { static_cast<float>(voxelizer_settings_.resolution), marching_cubes_ }),
    "configuring the tesselator");
}

void R3vNode::createInterfaces() {
  tf_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cloud_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  render_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions tf_options;
  tf_options.callback_group = tf_group_;
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    tf_topic_, tf2_ros::DynamicListenerQoS(),
    [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { onTf(*msg, false); }, tf_options);
  tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    tf_static_topic_, tf2_ros::StaticListenerQoS(),
    [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { onTf(*msg, true); }, tf_options);

  rclcpp::SubscriptionOptions cloud_options;
  cloud_options.callback_group = cloud_group_;
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

  rclcpp::SubscriptionOptions render_options;
  render_options.callback_group = render_group_;
  if (robot_description_file_.empty() && !robot_description_topic_.empty()) {
    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { onRobotDescription(msg->data); },
      render_options);
  }
  // Runtime controls only queue their change for the next onRender(). In the
  // render group they could wait forever: a render longer than its period
  // leaves the timer always ready, and the executor serves timers first.
  rclcpp::SubscriptionOptions control_options;
  control_options.callback_group = control_group_;
  if (!camera_pose_topic_.empty()) {
    camera_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      camera_pose_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(controls_mutex_);
        controls_.camera_pose = *msg;
      }, control_options);
  }
  hidden_links_sub_ = create_subscription<std_msgs::msg::String>(
    "~/hidden_links", rclcpp::QoS(10),
    [this](std_msgs::msg::String::ConstSharedPtr msg) { onHiddenLinks(msg->data); },
    control_options);
  voxels_visible_sub_ = create_subscription<std_msgs::msg::Bool>(
    "~/voxels_visible", rclcpp::QoS(10),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.voxels_visible = msg->data;
    }, control_options);
  clear_voxels_sub_ = create_subscription<std_msgs::msg::Empty>(
    "~/clear_voxels", rclcpp::QoS(10),
    [this](std_msgs::msg::Empty::ConstSharedPtr) {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.clear_voxels = true;
    }, control_options);
  // Values the renderer rejects are dropped here, failing there stops the node.
  marching_cubes_sub_ = create_subscription<std_msgs::msg::Bool>(
    "~/marching_cubes/set_enabled", rclcpp::QoS(10),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.marching_cubes = msg->data;
    }, control_options);
  background_color_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
    "~/background/set_color", rclcpp::QoS(10),
    [this](std_msgs::msg::ColorRGBA::ConstSharedPtr msg) {
      if (!validColor(msg->r, msg->g, msg->b)) {
        RCLCPP_WARN(
          get_logger(), "Ignoring background color (%g, %g, %g): components must be "
          "between 0 and 1", msg->r, msg->g, msg->b);
        return;
      }
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.background_color = rr::R3vVec3{ msg->r, msg->g, msg->b };
    }, control_options);
  color_depth_range_sub_ = create_subscription<std_msgs::msg::Float64>(
    "~/colormap/set_depth_range", rclcpp::QoS(10),
    [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
      if (!validPositiveFloat(msg->data)) {
        RCLCPP_WARN(
          get_logger(), "Ignoring colormap depth range %g: must be positive and fit in a "
          "float", msg->data);
        return;
      }
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.color_depth_range = msg->data;
    }, control_options);

  const rclcpp::QoS state_qos = rclcpp::QoS(1).reliable().transient_local();
  marching_cubes_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/marching_cubes/get_enabled", state_qos);
  background_color_pub_ =
    create_publisher<std_msgs::msg::ColorRGBA>("~/background/get_color", state_qos);
  color_depth_range_pub_ =
    create_publisher<std_msgs::msg::Float64>("~/colormap/get_depth_range", state_qos);
  publishMarchingCubes();
  publishBackgroundColor();
  publishColorDepthRange();
  createVoxelizerInterfaces(control_options);

  image_pub_ = image_transport::create_publisher(
    this, output_topic_,
    output_qos_ == "reliable" ? rmw_qos_profile_default : rmw_qos_profile_sensor_data);
  render_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_), [this]() { onRender(); }, render_group_);
}

void R3vNode::createVoxelizerInterfaces(const rclcpp::SubscriptionOptions& options) {
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
    "~/voxelizer/set_resolution", "voxelizer resolution", validPositiveFloat,
    "must be positive and fit in a float",
    [](Controls& controls, double value) { controls.resolution = value; });
  destroy_threshold_sub_ = u32_setter(
    "~/voxelizer/set_destroy_threshold",
    [](Controls& controls, uint32_t value) { controls.destroy_threshold = value; });
  construct_threshold_sub_ = u32_setter(
    "~/voxelizer/set_construct_threshold",
    [](Controls& controls, uint32_t value) { controls.construct_threshold = value; });
  max_value_sub_ = u32_setter(
    "~/voxelizer/set_max_value",
    [](Controls& controls, uint32_t value) { controls.max_value = value; });

  const rclcpp::QoS state_qos = rclcpp::QoS(1).reliable().transient_local();
  resolution_pub_ =
    create_publisher<std_msgs::msg::Float64>("~/voxelizer/get_resolution", state_qos);
  destroy_threshold_pub_ =
    create_publisher<std_msgs::msg::UInt32>("~/voxelizer/get_destroy_threshold", state_qos);
  construct_threshold_pub_ =
    create_publisher<std_msgs::msg::UInt32>("~/voxelizer/get_construct_threshold", state_qos);
  max_value_pub_ =
    create_publisher<std_msgs::msg::UInt32>("~/voxelizer/get_max_value", state_qos);

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
  publishVoxelizerSettings();
}

void R3vNode::onTf(const tf2_msgs::msg::TFMessage& msg, bool is_static) {
  for (const auto& transform : msg.transforms) {
    try {
      tf_buffer_->setTransform(transform, "r3v_node", is_static);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Bad transform: %s", e.what());
    }
  }
}

std::optional<Eigen::Isometry3d> R3vNode::lookup(
  const std::string& frame, const rclcpp::Time& stamp, double timeout,
  std::string* error) const {
  if (frame.empty() || frame == fixed_frame_) {
    return Eigen::Isometry3d::Identity();
  }
  try {
    return tf2::transformToEigen(
      tf_buffer_->lookupTransform(
        fixed_frame_, frame, stamp, rclcpp::Duration::from_seconds(timeout)));
  } catch (const tf2::TransformException& e) {
    if (error != nullptr) {
      *error = e.what();
    }
    return std::nullopt;
  }
}

void R3vNode::onCloud(size_t index, const sensor_msgs::msg::PointCloud2& cloud) {
  const CloudConfig& config = cloud_configs_[index];
  uint64_t generation = 0;
  {
    // Settings change before the generation does, so a new generation comes
    // with its settings.
    std::lock_guard<std::mutex> lock(settings_mutex_);
    generation = voxel_generation_.load();
    if (generation != voxelizer_generation_) {
      active_settings_ = voxelizer_settings_;
    }
  }
  if (generation != voxelizer_generation_) {
    // The map was cleared or its settings changed: start over with an empty
    // voxelizer. The old one goes first, a wgpu one holds GPU memory.
    voxelizer_.reset();
    try {
      voxelizer_ = makeVoxelizer(active_settings_);
    } catch (const std::exception& e) {
      // The next cloud tries again.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping point cloud '%s': %s", config.name.c_str(),
        e.what());
      return;
    }
    voxelizer_generation_ = generation;
  }

  const rclcpp::Time stamp(cloud.header.stamp);
  std::string error;
  const auto cloud_to_fixed = lookup(cloud.header.frame_id, stamp, tf_timeout_, &error);
  if (!cloud_to_fixed) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping point cloud '%s': %s", config.name.c_str(),
      error.c_str());
    return;
  }
  Eigen::Vector3d sensor = cloud_to_fixed->translation();
  if (!config.sensor_frame.empty() && config.sensor_frame != cloud.header.frame_id) {
    const auto sensor_to_fixed = lookup(config.sensor_frame, stamp, tf_timeout_, &error);
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
  // Clouds share the voxelizer, each with its own multipliers. Cloud
  // callbacks are mutually exclusive, so nothing runs in between.
  rv::set_multipliers(voxelizer_.get(), active_settings_.multipliers[index]);
  const rv::VoxelsUpdate update =
    rv::add_points(voxelizer_.get(), sensor_pos, points_.data(), points_.size());
  if (update.len > 0) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (generation != voxel_generation_.load()) {
      rv::drop_update(update);  // the map was cleared meanwhile
      return;
    }
    const size_t old_size = pending_blocks_.size();
    pending_blocks_.resize(old_size + update.len);
    std::memcpy(
      pending_blocks_.data() + old_size, update.data, update.len * sizeof(rr::R3vPointI));
  }
  rv::drop_update(update);
}

void R3vNode::onRobotDescription(const std::string& urdf_xml) {
  try {
    robot_->load(urdf_xml, renderer_.get());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Ignoring robot description: %s", e.what());
  }
}

void R3vNode::onHiddenLinks(const std::string& links) {
  std::vector<std::string> names;
  std::string name;
  for (const char c : links + ',') {
    if (c == ',' || std::isspace(static_cast<unsigned char>(c))) {
      if (!name.empty()) {
        names.push_back(std::move(name));
        name.clear();
      }
    } else {
      name += c;
    }
  }
  std::lock_guard<std::mutex> lock(controls_mutex_);
  controls_.hidden_links = std::move(names);
}

void R3vNode::applyControls() {
  Controls controls;
  {
    std::lock_guard<std::mutex> lock(controls_mutex_);
    std::swap(controls, controls_);
  }
  if (controls.camera_pose) {
    camera_pose_ = std::move(controls.camera_pose);
  }
  if (controls.hidden_links) {
    robot_->setHiddenLinks(*controls.hidden_links);
    std::string list;
    for (const auto& name : *controls.hidden_links) {
      list += (list.empty() ? "" : ", ") + name;
    }
    RCLCPP_INFO(get_logger(), "Hidden links: %s", list.empty() ? "none" : list.c_str());
  }
  // Cleared first: of a clear and a visibility change arriving together, the
  // visibility is the newer state.
  if (controls.clear_voxels) {
    clearVoxels();
  }
  VoxelizerSettings settings = voxelizer_settings_;
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
  // Clears the map too, so also before marching cubes and the visibility.
  setVoxelizerSettings(settings);
  // Before the visibility, which would tesselate the shown map in the old mode.
  if (controls.marching_cubes) {
    setMarchingCubes(*controls.marching_cubes);
  }
  if (controls.voxels_visible) {
    setVoxelsVisible(*controls.voxels_visible);
  }
  if (controls.background_color) {
    setBackgroundColor(*controls.background_color);
  }
  if (controls.color_depth_range) {
    setColorDepthRange(*controls.color_depth_range);
  }
}

void R3vNode::flushVoxels() {
  blocks_.clear();
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    blocks_.swap(pending_blocks_);
  }
  // Blocks carry their whole occupancy mask, so the last one wins.
  for (const auto& block : blocks_) {
    const BlockKey key{ block.x, block.y, block.z };
    if (block.i == 0) {
      live_blocks_.erase(key);
    } else {
      live_blocks_[key] = block.i;
    }
  }
  if (voxels_visible_) {
    tesselate(blocks_);
  }
}

void R3vNode::tesselateLiveBlocks(bool empty) {
  blocks_.clear();
  blocks_.reserve(live_blocks_.size());
  for (const auto& [key, mask] : live_blocks_) {
    blocks_.push_back({ key.x, key.y, key.z, empty ? 0u : mask });
  }
  // Empty blocks empty their chunks, which the renderer then drops.
  tesselate(blocks_);
}

void R3vNode::setVoxelsVisible(bool visible) {
  if (visible == voxels_visible_) {
    return;
  }
  voxels_visible_ = visible;
  tesselateLiveBlocks(!visible);
  RCLCPP_INFO(get_logger(), "%s the voxel map", visible ? "Showing" : "Hiding");
}

void R3vNode::clearVoxels() {
  // Bumped before pending_blocks_ is emptied, so cloud callbacks still
  // working with the old voxelizer cannot add to it afterwards.
  ++voxel_generation_;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_blocks_.clear();
  }
  if (voxels_visible_) {
    tesselateLiveBlocks(true);
  }
  live_blocks_.clear();
  RCLCPP_INFO(get_logger(), "Cleared the voxel map");
}

void R3vNode::setMarchingCubes(bool enabled) {
  if (enabled == marching_cubes_) {
    return;
  }
  marching_cubes_ = enabled;
  checkStatus(
    rr::r3v_cpu_tesselator_set_marching_cubes_enabled(tesselator_.get(), enabled),
    "switching marching cubes");
  // Every chunk with voxels is re-tesselated, new blocks or not.
  uploadGeometryDelta();
  publishMarchingCubes();
  RCLCPP_INFO(get_logger(), "%s marching cubes", enabled ? "Enabled" : "Disabled");
}

void R3vNode::setBackgroundColor(const rr::R3vVec3& color) {
  const rr::R3vVec3& old = background_color_;
  if (color.x == old.x && color.y == old.y && color.z == old.z) {
    return;
  }
  background_color_ = color;
  configureRenderer();
  publishBackgroundColor();
  RCLCPP_INFO(get_logger(), "Background color (%g, %g, %g)", color.x, color.y, color.z);
}

void R3vNode::setColorDepthRange(double range) {
  if (range == color_depth_range_) {
    return;
  }
  color_depth_range_ = range;
  configureRenderer();
  publishColorDepthRange();
  RCLCPP_INFO(get_logger(), "Colormap depth range %g m", range);
}

void R3vNode::setVoxelizerSettings(const VoxelizerSettings& settings) {
  const VoxelizerSettings& old = voxelizer_settings_;
  const bool same_multipliers = std::equal(
    settings.multipliers.begin(), settings.multipliers.end(), old.multipliers.begin(),
    old.multipliers.end(), sameMultipliers);
  if (settings.resolution == old.resolution &&
    settings.destroy_threshold == old.destroy_threshold &&
    settings.construct_threshold == old.construct_threshold &&
    settings.max_value == old.max_value && same_multipliers) {
    return;
  }
  const bool new_resolution = settings.resolution != old.resolution;
  {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    voxelizer_settings_ = settings;
  }
  // After the settings: cloud callbacks seeing the new generation make the
  // new voxelizer with them. The tesselator gets the new voxel size once the
  // renderer dropped every chunk.
  clearVoxels();
  if (new_resolution) {
    configureTesselator();
  }
  publishVoxelizerSettings();
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
}

void R3vNode::publishMarchingCubes() {
  std_msgs::msg::Bool msg;
  msg.data = marching_cubes_;
  marching_cubes_pub_->publish(msg);
}

void R3vNode::publishBackgroundColor() {
  std_msgs::msg::ColorRGBA msg;
  msg.r = background_color_.x;
  msg.g = background_color_.y;
  msg.b = background_color_.z;
  msg.a = 1.0f;
  background_color_pub_->publish(msg);
}

void R3vNode::publishColorDepthRange() {
  std_msgs::msg::Float64 msg;
  msg.data = color_depth_range_;
  color_depth_range_pub_->publish(msg);
}

void R3vNode::publishVoxelizerSettings() {
  const VoxelizerSettings& settings = voxelizer_settings_;
  std_msgs::msg::Float64 f64;
  std_msgs::msg::UInt32 u32;
  f64.data = settings.resolution;
  resolution_pub_->publish(f64);
  u32.data = settings.destroy_threshold;
  destroy_threshold_pub_->publish(u32);
  u32.data = settings.construct_threshold;
  construct_threshold_pub_->publish(u32);
  u32.data = settings.max_value;
  max_value_pub_->publish(u32);
  for (size_t index = 0; index < multiplier_topics_.size(); ++index) {
    const rv::Multipliers& m = settings.multipliers[index];
    const MultiplierTopics& topics = multiplier_topics_[index];
    f64.data = m.intensity;
    topics.intensity_pub->publish(f64);
    u32.data = m.hit;
    topics.hit_pub->publish(u32);
    u32.data = m.target;
    topics.target_pub->publish(u32);
  }
}

void R3vNode::tesselate(const std::vector<rr::R3vPointI>& blocks) {
  if (blocks.empty()) {
    return;
  }
  checkStatus(
    rr::r3v_cpu_tesselator_update_input_geometry(
      tesselator_.get(), blocks.data(), blocks.size()),
    "tesselating voxels");
  uploadGeometryDelta();
}

void R3vNode::uploadGeometryDelta() {
  rr::R3vGeometryDelta* delta = nullptr;
  checkStatus(
    rr::r3v_cpu_tesselator_get_output_geometry_delta(tesselator_.get(), &delta),
    "tesselating voxels");
  if (delta != nullptr) {
    // Takes ownership of delta, also on failure.
    checkStatus(
      rr::r3v_wgpu_renderer_update_geometry(renderer_.get(), delta), "uploading voxels");
  }
}

bool R3vNode::cameraView(CameraView& view) const {
  if (camera_pose_) {
    const auto to_fixed =
      robot_->lookupLatest(*tf_buffer_, fixed_frame_, camera_pose_->header.frame_id);
    if (!to_fixed) {
      return false;
    }
    Eigen::Isometry3d pose;
    tf2::fromMsg(camera_pose_->pose, pose);
    // REP-103: the camera looks along +X with +Z up.
    const Eigen::Isometry3d camera = *to_fixed * pose;
    view.eye = camera.translation();
    view.target = view.eye + camera.linear() * Eigen::Vector3d::UnitX();
    view.up = camera.linear() * Eigen::Vector3d::UnitZ();
    return true;
  }
  const auto to_fixed = robot_->lookupLatest(*tf_buffer_, fixed_frame_, camera_frame_);
  if (!to_fixed) {
    return false;
  }
  view.eye = *to_fixed * camera_position_;
  view.target = *to_fixed * camera_target_;
  view.up = Eigen::Vector3d::UnitZ();
  return true;
}

void R3vNode::onRender() {
  using Clock = std::chrono::steady_clock;
  const auto ms = [](Clock::time_point from, Clock::time_point to) {
      return std::chrono::duration<double, std::milli>(to - from).count();
    };
  try {
    const auto t0 = Clock::now();
    applyControls();
    flushVoxels();
    const auto t1 = Clock::now();
    robot_->update(*tf_buffer_, fixed_frame_, renderer_.get());
    const auto t2 = Clock::now();

    if (lazy_ && image_pub_.getNumSubscribers() == 0) {
      // The frame the renderer holds gets stale; never publish it later.
      rendered_stamp_.reset();
      return;
    }

    CameraView view;
    rr::R3vCameraState state;
    if (!cameraView(view) ||
      !makeCameraState(
        view, lens_, static_cast<double>(width_) / static_cast<double>(height_), state)) {
      const std::string& frame =
        camera_pose_ ? camera_pose_->header.frame_id : camera_frame_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot place the camera: no transform from '%s' to '%s', or eye on target",
        frame.c_str(), fixed_frame_.c_str());
      return;
    }
    // The renderer colours voxels by their distance from state.position
    // along the view, through turbo over 0..color_depth_range, and reads the
    // position for nothing else: moving it back keeps close voxels off
    // turbo's dark end.
    state.position.x -= static_cast<float>(color_offset_) * state.forward.x;
    state.position.y -= static_cast<float>(color_offset_) * state.forward.y;
    state.position.z -= static_cast<float>(color_offset_) * state.forward.z;

    const rclcpp::Time now = this->now();
    checkStatus(
      rr::r3v_wgpu_renderer_render(renderer_.get(), &state, frame_.data(), frame_.size()),
      "rendering");
    // Frames are read back one call late: frame_ is the one rendered at the
    // previous call (none after configure).
    const auto t3 = Clock::now();
    if (rendered_stamp_) {
      publish(frame_, *rendered_stamp_);
    }
    rendered_stamp_ = now;
    RCLCPP_DEBUG(
      get_logger(), "Frame: voxels %.1f ms, robot %.1f ms, render %.1f ms, publish %.1f ms",
      ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t3, Clock::now()));
  } catch (const std::exception& e) {
    // Renderer errors leave it unusable; stop so a supervisor can restart us.
    RCLCPP_FATAL(get_logger(), "%s", e.what());
    render_timer_->cancel();
    rclcpp::shutdown();
  }
}

void R3vNode::publish(const std::vector<uint8_t>& bgra, const rclcpp::Time& stamp) {
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = output_frame_id_.empty() ? fixed_frame_ : output_frame_id_;
  msg.width = width_;
  msg.height = height_;
  msg.encoding = encoding_;
  msg.is_bigendian = false;

  const size_t pixels = static_cast<size_t>(width_) * height_;
  if (encoding_ == enc::BGRA8) {
    msg.step = width_ * 4;
    msg.data = bgra;
  } else if (encoding_ == enc::RGBA8) {
    msg.step = width_ * 4;
    msg.data.resize(pixels * 4);
    for (size_t i = 0; i < pixels; ++i) {
      msg.data[i * 4 + 0] = bgra[i * 4 + 2];
      msg.data[i * 4 + 1] = bgra[i * 4 + 1];
      msg.data[i * 4 + 2] = bgra[i * 4 + 0];
      msg.data[i * 4 + 3] = bgra[i * 4 + 3];
    }
  } else {
    const bool rgb = encoding_ == enc::RGB8;
    msg.step = width_ * 3;
    msg.data.resize(pixels * 3);
    for (size_t i = 0; i < pixels; ++i) {
      msg.data[i * 3 + 0] = bgra[i * 4 + (rgb ? 2 : 0)];
      msg.data[i * 3 + 1] = bgra[i * 4 + 1];
      msg.data[i * 3 + 2] = bgra[i * 4 + (rgb ? 0 : 2)];
    }
  }
  image_pub_.publish(msg);
}

}  // namespace r3v_ros

RCLCPP_COMPONENTS_REGISTER_NODE(r3v_ros::R3vNode)
