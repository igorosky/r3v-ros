#include "r3v_ros/renderer_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"
#if __has_include("tf2_eigen/tf2_eigen.hpp")
#include "tf2_eigen/tf2_eigen.hpp"
#else
#include "tf2_eigen/tf2_eigen.h"
#endif

namespace r3v_ros {

namespace {

namespace enc = sensor_msgs::image_encodings;
using msg::VoxelMapUpdate;

// The tesselator takes the voxel stream's payload as it is.
static_assert(
  sizeof(rr::R3vPointI) == sizeof(WireBlock) &&
  offsetof(rr::R3vPointI, i) == offsetof(WireBlock, mask),
  "r3v-renderer R3vPointI must have the layout of WireBlock");
static_assert(
  sizeof(rr::R3vVoxelGroup) == sizeof(WireGroup) &&
  offsetof(rr::R3vVoxelGroup, values) == offsetof(WireGroup, pages) &&
  rr::R3V_VOXEL_GROUP_PAGES == kGroupPages,
  "r3v-renderer R3vVoxelGroup must have the layout of WireGroup");

// Voxel size the tesselator starts with; the first FULL message brings the
// real one.
constexpr double kInitialResolution = 0.1;

}  // namespace

RendererNode::RendererNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("r3v_renderer", options) {
  declareParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

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
    get_logger(), "Rendering %ux%u at %.1f Hz to '%s' (fixed frame '%s', voxels from '%s'), %s",
    width_, height_, rate_, image_pub_.getTopic().c_str(), fixed_frame_.c_str(),
    stream_topic_.c_str(), enabled_ ? "enabled" : "disabled");
}

void RendererNode::declareParameters() {
  fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
  rate_ = declare_parameter<double>("rate", 15.0);
  tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
  tf_static_topic_ = declare_parameter<std::string>("tf_static_topic", "/tf_static");
  enabled_ = declare_parameter<bool>("enabled", true);
  require(!fixed_frame_.empty(), "fixed_frame must not be empty");
  require(rate_ > 0.0, "rate must be positive");

  stream_topic_ = declare_parameter<std::string>("voxel_stream.topic", "/r3v/voxels");
  const int64_t depth = declare_parameter<int64_t>("voxel_stream.depth", 100);
  request_topic_ = declare_parameter<std::string>(
    "voxel_stream.request_topic", "/r3v_voxelizer/request_full_sync");
  sync_retry_ = declare_parameter<double>("voxel_stream.sync_retry", 1.0);
  require(!stream_topic_.empty(), "voxel_stream.topic must not be empty");
  require(depth >= 1 && depth <= 100000, "voxel_stream.depth must be between 1 and 100000");
  require(!request_topic_.empty(), "voxel_stream.request_topic must not be empty");
  require(sync_retry_ > 0.0, "voxel_stream.sync_retry must be positive");
  stream_depth_ = static_cast<size_t>(depth);

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

  voxels_visible_ = declare_parameter<bool>("voxels.visible", true);
  marching_cubes_ = declare_parameter<bool>("voxels.marching_cubes", false);
}

void RendererNode::createRenderer() {
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
  resolution_ = kInitialResolution;
  configureTesselator();
}

void RendererNode::configureRenderer() {
  // The same frame size keeps the frame the renderer holds. Default terrain
  // coloring: no height coloring, voxels keep the depth colormap.
  checkStatus(
    rr::r3v_wgpu_renderer_configure(
      renderer_.get(),
      { width_, height_, background_color_, static_cast<float>(color_depth_range_),
        rr::r3v_terrain_coloring_default() }),
    "configuring the renderer");
}

void RendererNode::configureTesselator() {
  // Keeps the blocks the tesselator holds.
  checkStatus(
    rr::r3v_cpu_tesselator_configure(
      tesselator_.get(), { static_cast<float>(resolution_), marching_cubes_ }),
    "configuring the tesselator");
}

void RendererNode::createInterfaces() {
  tf_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  stream_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  render_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Runtime controls only queue their change for the next onRender(). In the
  // render group they could wait forever: a render longer than its period
  // leaves the timer always ready, and the executor serves timers first.
  rclcpp::SubscriptionOptions control_options;
  control_options.callback_group = control_group_;
  enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "~/set_enabled", rclcpp::QoS(10),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      controls_.enabled = msg->data;
    }, control_options);
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
  enabled_pub_ = create_publisher<std_msgs::msg::Bool>("~/get_enabled", state_qos);
  marching_cubes_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/marching_cubes/get_enabled", state_qos);
  background_color_pub_ =
    create_publisher<std_msgs::msg::ColorRGBA>("~/background/get_color", state_qos);
  color_depth_range_pub_ =
    create_publisher<std_msgs::msg::Float64>("~/colormap/get_depth_range", state_qos);
  publishEnabled();
  publishMarchingCubes();
  publishBackgroundColor();
  publishColorDepthRange();

  request_pub_ = create_publisher<msg::FullSyncRequest>(request_topic_, rclcpp::QoS(10));
  if (enabled_) {
    subscribeInputs();
  }

  image_pub_ = image_transport::create_publisher(
    this, output_topic_,
    output_qos_ == "reliable" ? rmw_qos_profile_default : rmw_qos_profile_sensor_data);
  render_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_), [this]() { onRender(); }, render_group_);
  sync_timer_ = create_wall_timer(
    std::chrono::duration<double>(sync_retry_), [this]() { onSyncRetry(); }, stream_group_);
}

void RendererNode::subscribeInputs() {
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    listening_ = true;
    ++epoch_;
    sync_ = StreamSync();
    pending_ = PendingVoxels();
  }
  tf_subs_ = subscribeTf(*this, *tf_buffer_, tf_topic_, tf_static_topic_, tf_group_);

  rclcpp::SubscriptionOptions stream_options;
  stream_options.callback_group = stream_group_;
  // Straight from a voxelizer in the same process (see VoxelizerNode).
  stream_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  stream_sub_ = create_subscription<VoxelMapUpdate>(
    stream_topic_, voxelStreamQos(stream_depth_),
    [this](VoxelMapUpdate::ConstSharedPtr msg) { onVoxelUpdate(*msg); }, stream_options);

  if (robot_description_file_.empty() && !robot_description_topic_.empty()) {
    rclcpp::SubscriptionOptions render_options;
    render_options.callback_group = render_group_;
    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { onRobotDescription(msg->data); },
      render_options);
  }
}

void RendererNode::unsubscribeInputs() {
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    listening_ = false;
    ++epoch_;
    sync_ = StreamSync();
    pending_ = PendingVoxels();
  }
  tf_subs_ = TfSubscriptions();
  stream_sub_.reset();
  robot_description_sub_.reset();
}

void RendererNode::onVoxelUpdate(const VoxelMapUpdate& msg) {
  if (msg.header.frame_id != fixed_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring the voxel stream: its voxels are in '%s', "
      "not in fixed_frame '%s'", msg.header.frame_id.c_str(), fixed_frame_.c_str());
    return;
  }
  // Only this callback group and setEnabled() change the sync state, and
  // the stream group runs one callback at a time: the lock is only taken
  // against setEnabled(), and left while decoding.
  std::unique_lock<std::mutex> lock(stream_mutex_);
  if (!listening_) {
    return;
  }
  const bool in_row =
    sync_.session && *sync_.session == msg.session && msg.sequence == sync_.last + 1;
  if (!in_row) {
    if (sync_.session && *sync_.session != msg.session) {
      RCLCPP_INFO(
        get_logger(), "New voxelizer session %016" PRIx64 ", synchronizing", msg.session);
    } else if (sync_.session) {
      RCLCPP_WARN(
        get_logger(), "Voxel stream message %" PRIu64 " follows %" PRIu64 ", synchronizing "
        "again", msg.sequence, sync_.last);
    }
    // Every FULL message from here on reaches us, in order.
    sync_.session = msg.session;
    sync_.since = msg.sequence;
    sync_.synced = false;
  }
  sync_.last = msg.sequence;
  if (msg.kind == VoxelMapUpdate::DELTA && !sync_.synced) {
    if (!in_row) {
      requestFullSync();  // onSyncRetry() repeats it
    }
    return;
  }
  const uint64_t epoch = epoch_;
  const double synced_resolution = sync_.resolution;
  lock.unlock();

  std::optional<std::vector<WireGroup>> groups;
  std::vector<WireBlock> blocks;
  std::string error;
  try {
    if (!validPositiveFloat(msg.resolution)) {
      throw std::runtime_error("invalid resolution " + std::to_string(msg.resolution));
    }
    if (msg.kind == VoxelMapUpdate::FULL) {
      groups = decodePayload<WireGroup>(msg);
      const auto invalid = std::find_if_not(
        groups->begin(), groups->end(),
        [](const WireGroup& group) { return validGroupKey(group.key); });
      if (invalid != groups->end()) {
        throw std::runtime_error("group key out of range");
      }
    } else if (msg.kind == VoxelMapUpdate::DELTA) {
      if (msg.resolution != synced_resolution) {
        throw std::runtime_error("resolution differs from the one of the last FULL message");
      }
      blocks = decodePayload<WireBlock>(msg);
      if (!std::all_of(blocks.begin(), blocks.end(), validBlock)) {
        throw std::runtime_error("block off the 4x4x2 grid");
      }
    } else {
      throw std::runtime_error("unknown kind " + std::to_string(msg.kind));
    }
  } catch (const std::exception& e) {
    error = e.what();
  }

  lock.lock();
  if (epoch != epoch_) {
    return;  // disabled meanwhile
  }
  if (!error.empty()) {
    RCLCPP_WARN(
      get_logger(), "Bad voxel stream message %" PRIu64 ": %s; synchronizing again",
      msg.sequence, error.c_str());
    // A FULL message after this one: this one may be the last FULL.
    sync_.since = msg.sequence + 1;
    sync_.synced = false;
    requestFullSync();
    return;
  }
  if (groups) {
    if (!sync_.synced) {
      RCLCPP_INFO(
        get_logger(), "Synchronized with the voxel map: %zu groups at %g m", groups->size(),
        msg.resolution);
    }
    sync_.synced = true;
    sync_.resolution = msg.resolution;
    // It holds every block before it.
    pending_.full = std::move(groups);
    pending_.resolution = msg.resolution;
    pending_.blocks.clear();
  } else {
    pending_.blocks.insert(pending_.blocks.end(), blocks.begin(), blocks.end());
  }
}

void RendererNode::onSyncRetry() {
  std::lock_guard<std::mutex> lock(stream_mutex_);
  // Requests and FULL messages get lost, and a voxelizer that just started
  // may not know us yet. It ignores requests a FULL message answers already.
  if (listening_ && sync_.session && !sync_.synced) {
    requestFullSync();
  }
}

void RendererNode::requestFullSync() {
  msg::FullSyncRequest request;
  request.session = *sync_.session;
  request.since = sync_.since;
  request_pub_->publish(request);
}

void RendererNode::onRobotDescription(const std::string& urdf_xml) {
  if (urdf_xml == loaded_urdf_) {
    // Delivered again after an enable. The renderer cannot delete assets,
    // so loading it again would upload every mesh once more.
    return;
  }
  try {
    robot_->load(urdf_xml, renderer_.get());
    loaded_urdf_ = urdf_xml;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Ignoring robot description: %s", e.what());
  }
}

void RendererNode::onHiddenLinks(const std::string& links) {
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

void RendererNode::applyControls() {
  Controls controls;
  {
    std::lock_guard<std::mutex> lock(controls_mutex_);
    std::swap(controls, controls_);
  }
  if (controls.enabled) {
    setEnabled(*controls.enabled);
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

void RendererNode::flushVoxels() {
  PendingVoxels pending;
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    std::swap(pending, pending_);
  }
  if (pending.full) {
    if (pending.resolution != resolution_) {
      // The tesselator gets the new voxel size once the renderer dropped
      // every chunk.
      replaceTesselated({ });
      resolution_ = pending.resolution;
      configureTesselator();
    }
    map_.replace(std::move(*pending.full));
    if (voxels_visible_) {
      replaceTesselated(map_.groups());
    }
  }
  if (!pending.blocks.empty()) {
    map_.apply(pending.blocks);
    if (voxels_visible_) {
      tesselate(pending.blocks);
    }
  }
}

void RendererNode::tesselate(const std::vector<WireBlock>& blocks) {
  if (blocks.empty()) {
    return;
  }
  checkStatus(
    rr::r3v_cpu_tesselator_update_input_geometry(
      tesselator_.get(), reinterpret_cast<const rr::R3vPointI*>(blocks.data()), blocks.size()),
    "tesselating voxels");
  uploadGeometryDelta();
}

void RendererNode::replaceTesselated(const std::vector<WireGroup>& groups) {
  // Chunks left without voxels come out empty, which the renderer drops.
  checkStatus(
    rr::r3v_cpu_tesselator_replace_input_geometry(
      tesselator_.get(), reinterpret_cast<const rr::R3vVoxelGroup*>(groups.data()),
      groups.size()),
    "tesselating voxels");
  uploadGeometryDelta();
}

void RendererNode::uploadGeometryDelta() {
  rr::R3vGeometryDelta* delta = nullptr;
  // No camera state: every changed chunk, not only the ones in view.
  checkStatus(
    rr::r3v_cpu_tesselator_get_output_geometry_delta(tesselator_.get(), nullptr, &delta),
    "tesselating voxels");
  if (delta != nullptr) {
    // Takes ownership of delta, also on failure.
    checkStatus(
      rr::r3v_wgpu_renderer_update_geometry(renderer_.get(), delta), "uploading voxels");
  }
}

void RendererNode::setEnabled(bool enabled) {
  if (enabled == enabled_) {
    return;
  }
  enabled_ = enabled;
  if (enabled) {
    // The voxel stream brings the whole map again.
    subscribeInputs();
  } else {
    unsubscribeInputs();
    map_.clear();
    replaceTesselated({ });
    // Stale by the time the renderer is enabled again.
    tf_buffer_->clear();
    rendered_stamp_.reset();
  }
  publishEnabled();
  RCLCPP_INFO(get_logger(), "%s the renderer", enabled ? "Enabled" : "Disabled");
}

void RendererNode::setVoxelsVisible(bool visible) {
  if (visible == voxels_visible_) {
    return;
  }
  voxels_visible_ = visible;
  // Point clouds keep building the map while it is hidden.
  if (visible) {
    replaceTesselated(map_.groups());
  } else {
    replaceTesselated({ });
  }
  RCLCPP_INFO(get_logger(), "%s the voxel map", visible ? "Showing" : "Hiding");
}

void RendererNode::setMarchingCubes(bool enabled) {
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

void RendererNode::setBackgroundColor(const rr::R3vVec3& color) {
  const rr::R3vVec3& old = background_color_;
  if (color.x == old.x && color.y == old.y && color.z == old.z) {
    return;
  }
  background_color_ = color;
  configureRenderer();
  publishBackgroundColor();
  RCLCPP_INFO(get_logger(), "Background color (%g, %g, %g)", color.x, color.y, color.z);
}

void RendererNode::setColorDepthRange(double range) {
  if (range == color_depth_range_) {
    return;
  }
  color_depth_range_ = range;
  configureRenderer();
  publishColorDepthRange();
  RCLCPP_INFO(get_logger(), "Colormap depth range %g m", range);
}

void RendererNode::publishEnabled() {
  std_msgs::msg::Bool msg;
  msg.data = enabled_;
  enabled_pub_->publish(msg);
}

void RendererNode::publishMarchingCubes() {
  std_msgs::msg::Bool msg;
  msg.data = marching_cubes_;
  marching_cubes_pub_->publish(msg);
}

void RendererNode::publishBackgroundColor() {
  std_msgs::msg::ColorRGBA msg;
  msg.r = background_color_.x;
  msg.g = background_color_.y;
  msg.b = background_color_.z;
  msg.a = 1.0f;
  background_color_pub_->publish(msg);
}

void RendererNode::publishColorDepthRange() {
  std_msgs::msg::Float64 msg;
  msg.data = color_depth_range_;
  color_depth_range_pub_->publish(msg);
}

bool RendererNode::cameraView(CameraView& view) const {
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

void RendererNode::onRender() {
  using Clock = std::chrono::steady_clock;
  const auto ms = [](Clock::time_point from, Clock::time_point to) {
      return std::chrono::duration<double, std::milli>(to - from).count();
    };
  try {
    const auto t0 = Clock::now();
    applyControls();
    if (!enabled_) {
      return;
    }
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

void RendererNode::publish(const std::vector<uint8_t>& bgra, const rclcpp::Time& stamp) {
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

RCLCPP_COMPONENTS_REGISTER_NODE(r3v_ros::RendererNode)
