
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/parameter.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "sea_surface_segmentation/cost_mapping.hpp"
#include "sea_surface_segmentation/occupancy_buffer.hpp"
#include "sea_surface_segmentation/segments_projection.hpp"

namespace sea_surface_layer
{

// Costmap layer that fuses one OR MORE camera segmentation streams into a
// shared persistent, decaying log-odds occupancy buffer (see
// occupancy_buffer.hpp). Each segmentation frame contributes ground-plane
// observations via `project_observations_inverse` (iterate world cells in the
// camera's reach, classify each by the pixel it covers: waterline contact →
// hit, water → miss, occluded body / sky → skip); the buffer remembers
// obstacles through gaps, forgets them over time, and accumulates evidence
// from overlapping cameras into the same world cell. Phases 1, 3, 4 of #19.
//
// Sources are configured via `observation_sources` (a vector of string names);
// each source has `<name>.segmentation_topic` and `<name>.camera_info_topic`.
// When `observation_sources` is empty, the layer falls back to the legacy
// top-level `segmentation_topic` / `camera_info_topic` as a single source
// named "default" — pre-#19 configs work unchanged.
class SeaSurfaceLayer: public nav2_costmap_2d::Layer
{
private:
  // Per-source state — one per configured observation source. Defined here so
  // the per-callback `Source &` parameter types resolve below.
  //
  // Stored as `unique_ptr<Source>` (not raw `Source`) so the address stays
  // stable for the lifetime of the layer: the subscriptions capture `Source *`
  // in their lambdas, and `sources_` would otherwise reallocate-and-invalidate
  // those pointers if grown after onInitialize. The vector is built once and
  // not modified after onInitialize completes.
  struct Source
  {
    std::string name;
    std::string segmentation_topic;
    std::string camera_info_topic;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr seg_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    std::shared_ptr<image_geometry::PinholeCameraModel> camera_model;
    // Pixel-space rectangles to mask out before projection — e.g. on-boat
    // structures (GPS antenna, hull) that consistently sit in this camera's
    // FOV and would otherwise back-project to false near-boat marks. Written
    // by the param callback under costmap_mutex_; snapshotted by
    // segmentsCallback under the same lock.
    std::vector<cv::Rect> image_mask_rects;
  };

public:
  SeaSurfaceLayer() {}
  ~SeaSurfaceLayer() {}

  void onInitialize() override
  {
    auto node = node_.lock();

    declareParameter("maximum_range", rclcpp::ParameterValue(maximum_range_));
    node->get_parameter(name_ + ".maximum_range", maximum_range_);

    // Reject rays whose grazing angle to the water plane is below this — they
    // back-project to large ground errors per arc-second of pitch and dominate
    // long-range noise. Camera height bounds the cut: a camera at height h
    // enforces max horizontal reach ≈ h / tan(min_grazing). Default 0 = off.
    declareParameter(
      "min_grazing_angle_deg", rclcpp::ParameterValue(min_grazing_angle_deg_));
    node->get_parameter(name_ + ".min_grazing_angle_deg", min_grazing_angle_deg_);
    if (!std::isfinite(min_grazing_angle_deg_) ||
      min_grazing_angle_deg_ < 0.0 || min_grazing_angle_deg_ >= 90.0)
    {
      RCLCPP_WARN_STREAM(
        logger_,
        "Invalid min_grazing_angle_deg (" << min_grazing_angle_deg_
          << "); must be in [0, 90). Using 0 (filter off).");
      min_grazing_angle_deg_ = 0.0;
    }

    // Graded log-odds occupancy parameters (all runtime-tunable).
    declareParameter("obstacle_clamp", rclcpp::ParameterValue(params_.obstacle_clamp));
    node->get_parameter(name_ + ".obstacle_clamp", params_.obstacle_clamp);
    declareParameter("clear_floor", rclcpp::ParameterValue(params_.clear_floor));
    node->get_parameter(name_ + ".clear_floor", params_.clear_floor);
    declareParameter("free_threshold", rclcpp::ParameterValue(params_.free_threshold));
    node->get_parameter(name_ + ".free_threshold", params_.free_threshold);
    declareParameter("lethal_threshold", rclcpp::ParameterValue(params_.lethal_threshold));
    node->get_parameter(name_ + ".lethal_threshold", params_.lethal_threshold);
    declareParameter("decay_half_life_s", rclcpp::ParameterValue(params_.decay_half_life_s));
    node->get_parameter(name_ + ".decay_half_life_s", params_.decay_half_life_s);

    // Graded evidence-model knobs (live-tunable). `obstacle_prob_min` shifts the
    // zero-crossing of the per-pixel logit; `max_evidence_step` caps any single
    // frame's contribution. Read fresh each segmentsCallback under the lock.
    declareParameter("obstacle_prob_min", rclcpp::ParameterValue(obstacle_prob_min_));
    node->get_parameter(name_ + ".obstacle_prob_min", obstacle_prob_min_);
    declareParameter("max_evidence_step", rclcpp::ParameterValue(max_evidence_step_));
    node->get_parameter(name_ + ".max_evidence_step", max_evidence_step_);

    // Validate the projection knobs at startup (they live on the layer, not in
    // OccupancyParams, so OccupancyBuffer::validate below doesn't cover them).
    // Mirror the runtime onParametersSet checks + the min_grazing_angle_deg
    // clamp-to-default pattern: a bad YAML value must not flow into the logit
    // math. obstacle_prob_min must be in (0,1) — 0 or 1 makes logit(prior) ±inf
    // (saturates / disables grading) and NaN silently disables the whole layer;
    // max_evidence_step must be finite and > 0 (a negative gives std::clamp
    // lo > hi, which is UB).
    if (!std::isfinite(obstacle_prob_min_) ||
      obstacle_prob_min_ <= 0.0 || obstacle_prob_min_ >= 1.0)
    {
      RCLCPP_WARN_STREAM(
        logger_, "Invalid obstacle_prob_min (" << obstacle_prob_min_
          << "); must be in (0, 1). Using default 0.35.");
      obstacle_prob_min_ = 0.35;
    }
    if (!std::isfinite(max_evidence_step_) || max_evidence_step_ <= 0.0) {
      RCLCPP_WARN_STREAM(
        logger_, "Invalid max_evidence_step (" << max_evidence_step_
          << "); must be finite and > 0. Using default 0.85.");
      max_evidence_step_ = 0.85;
    }

    std::string why;
    if (!sea_surface_segmentation::OccupancyBuffer::validate(params_, why)) {
      RCLCPP_WARN_STREAM(
        logger_, "Invalid occupancy params (" << why << "); using defaults.");
      params_ = sea_surface_segmentation::OccupancyParams{};
    }

    // Source list — empty falls back to single-source legacy config.
    declareParameter("observation_sources", rclcpp::ParameterValue(std::vector<std::string>{}));
    std::vector<std::string> source_names;
    node->get_parameter(name_ + ".observation_sources", source_names);

    if (source_names.empty()) {
      // Back-compat: pre-#19 single-source configuration.
      declareParameter("segmentation_topic", rclcpp::ParameterValue(std::string("segmentation")));
      declareParameter("camera_info_topic", rclcpp::ParameterValue(std::string("camera_info")));
      auto src = std::make_unique<Source>();
      src->name = "default";
      node->get_parameter(name_ + ".segmentation_topic", src->segmentation_topic);
      node->get_parameter(name_ + ".camera_info_topic", src->camera_info_topic);
      sources_.push_back(std::move(src));
    } else {
      // Multi-source: one (segmentation, camera_info) pair per named source.
      // All sources feed the shared world-frame buffer, so a single obstacle
      // in the overlap region accumulates evidence from every camera that
      // sees it — the multi-camera fusion the per-camera layers couldn't do.
      for (const auto & name : source_names) {
        declareParameter(name + ".segmentation_topic", rclcpp::ParameterValue(std::string{}));
        declareParameter(name + ".camera_info_topic", rclcpp::ParameterValue(std::string{}));
        // Pixel-space masks for this source (e.g. on-boat structures). Flat
        // int array of [x_min, y_min, x_max, y_max] tuples; empty = no mask.
        // Runtime-tunable via `ros2 param set <node> <layer>.<source>.image_mask_rects`.
        declareParameter(
          name + ".image_mask_rects",
          rclcpp::ParameterValue(std::vector<int64_t>{}));
        auto src = std::make_unique<Source>();
        src->name = name;
        node->get_parameter(name_ + "." + name + ".segmentation_topic", src->segmentation_topic);
        node->get_parameter(name_ + "." + name + ".camera_info_topic", src->camera_info_topic);
        std::vector<int64_t> mask_flat;
        node->get_parameter(name_ + "." + name + ".image_mask_rects", mask_flat);
        std::string mask_why;
        if (!parseMaskRects(mask_flat, src->image_mask_rects, mask_why)) {
          RCLCPP_WARN_STREAM(
            logger_,
            "SeaSurfaceLayer source '" << name << "' image_mask_rects invalid ("
              << mask_why << "); ignoring masks for this source");
          src->image_mask_rects.clear();
        }
        if (src->segmentation_topic.empty() || src->camera_info_topic.empty()) {
          RCLCPP_ERROR_STREAM(
            logger_,
            "SeaSurfaceLayer source '" << name << "' is missing segmentation_topic "
              "or camera_info_topic — skipping");
          continue;
        }
        sources_.push_back(std::move(src));
      }
    }

    if (sources_.empty()) {
      RCLCPP_WARN_STREAM(
        logger_,
        "SeaSurfaceLayer: no observation sources configured; layer will not "
        "ingest any segmentation frames");
    }

    global_frame_id_ = layered_costmap_->getGlobalFrameID();

    // Build the buffer before wiring subscribers so a queued segmentation
    // message can't dispatch segmentsCallback against a null buffer_.
    matchSize();

    for (auto & src_uptr : sources_) {
      // Capture the raw pointer in the lambda (Source's address is stable for
      // the lifetime of sources_; unique_ptr only moves when the vector is
      // modified, which we don't do after onInitialize). The Source struct
      // outlives the subscriptions because the subs are members of it.
      Source * src = src_uptr.get();
      src->seg_sub = node->create_subscription<sensor_msgs::msg::Image>(
        src->segmentation_topic, rclcpp::SensorDataQoS(),
        [this, src](sensor_msgs::msg::Image::SharedPtr msg) {
          segmentsCallback(*src, msg);
        });
      src->info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        src->camera_info_topic, rclcpp::SensorDataQoS(),
        [this, src](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          cameraInfoCallback(*src, msg);
        });
    }

    // Optional publish-and-relay: when `published_topic` is non-empty, the
    // layer republishes its lethal cells as a nav_msgs/OccupancyGrid on that
    // topic so the companion `SeaSurfaceRelayLayer` (in global_costmap) can
    // stamp them into the global costmap before inflation. Mirrors the
    // s57_grids / s57_layer producer/consumer pattern; the projection still
    // runs only once. Defaults to off (empty topic = no publisher).
    declareParameter("published_topic", rclcpp::ParameterValue(std::string{}));
    node->get_parameter(name_ + ".published_topic", published_topic_);
    if (!published_topic_.empty()) {
      lethal_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        published_topic_, rclcpp::QoS(1).transient_local());
      RCLCPP_INFO_STREAM(
        logger_, "SeaSurfaceLayer publishing lethal grid on '" << published_topic_ << "'");
    }

    // Live-tunable params via `ros2 param set` (phase 6). Decay half-life,
    // obstacle_clamp, clear_floor, free/lethal thresholds, obstacle_prob_min,
    // max_evidence_step, and maximum_range all reconfigure at runtime; topic
    // and source-list params stay configure-time
    // (subscriber re-bind is not supported here). The callback validates the
    // proposed change before applying — a fat-fingered set can't silently
    // poison the buffer interpretation.
    //
    // IMPORTANT: must register AFTER every `declareParameter` above. rclcpp
    // invokes on-set-parameters callbacks for the initial-value validation
    // inside `declare_parameter`; if a configure-time param (e.g.
    // `.published_topic`) were declared after this point, the callback's
    // `is_configure_time` rejection would throw `InvalidParameterValueException`
    // and abort `onInitialize` — the layer would fail to load.
    param_callback_handle_ = node->add_on_set_parameters_callback(
      std::bind(&SeaSurfaceLayer::onParametersSet, this, std::placeholders::_1));
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (buffer_) {
      buffer_->clear();
    }
  }

  // The layer clears itself via decay + water-miss observations; nav2's
  // clear-costmap behaviors don't need to (and shouldn't) wipe it.
  bool isClearable() override { return false; }

  void updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    // Snapshot the live-tunable scalars + parent-geometry tracking under the
    // lock. `maximum_range_` is written by the parameter callback under the
    // same lock; the 4 bound-expansion reads below would otherwise race a
    // concurrent `ros2 param set`. `count_x_/count_y_/resolution_` are written
    // under the lock in `matchSize()`; reading them here under the lock keeps
    // the geometry-change detection from tearing.
    auto parent = layered_costmap_->getCostmap();
    double maximum_range;
    bool geometry_changed;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      maximum_range = maximum_range_;
      geometry_changed =
        parent->getSizeInCellsX() != count_x_ ||
        parent->getSizeInCellsY() != count_y_ ||
        parent->getResolution() != resolution_;
    }

    // Expand (don't clobber) the master bounds so earlier layers' contributions survive.
    *min_x = std::min(*min_x, robot_x - maximum_range);
    *min_y = std::min(*min_y, robot_y - maximum_range);
    *max_x = std::max(*max_x, robot_x + maximum_range);
    *max_y = std::max(*max_y, robot_y + maximum_range);

    if (geometry_changed) {
      matchSize();  // recreate the buffer at the new size/resolution (locks internally)
    } else {
      // Rolling-window origin shift: roll the buffer to follow the parent so
      // accumulated evidence stays at its world location (grid_map::move()).
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (buffer_) {
        buffer_->move(parentCenter(parent));
      }
    }

    // Decay accumulated evidence toward the prior once per cycle.
    if (auto node = node_.lock()) {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (buffer_) {
        buffer_->decay(node->now().seconds());
      }
    }

    // Republish the lethal cells for any downstream consumer (e.g. the
    // SeaSurfaceRelayLayer in global_costmap). No-op when no publisher was
    // wired in onInitialize (published_topic was empty).
    publishLethalGrid();
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!buffer_) {
      return;
    }
    // Stamp the graded cost into the master from the buffer's occupancy ramp:
    // LETHAL at the lethal threshold, a soft cost below it, and nothing where the
    // buffer has no obstacle opinion (occ <= 0 → cost -1). We only ADD; free /
    // unknown remain owned by other layers + inflation.
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        double wx, wy;
        master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
        const int occ = buffer_->occupancyAt(grid_map::Position(wx, wy));
        const int c = sea_surface_segmentation::occupancy_to_cost(occ);
        if (c < 0) {
          continue;  // no obstacle opinion — leave the master cell untouched
        }
        // Combine with MAX (nav2 updateWithMax semantics): only raise a known
        // cost, and write over unknown. A graded soft cost (1..252) must never
        // downgrade another layer's INSCRIBED / LETHAL mark — sea-surface only
        // ADDS risk.
        const unsigned int mi = static_cast<unsigned int>(i);
        const unsigned int mj = static_cast<unsigned int>(j);
        const unsigned char old_cost = master_grid.getCost(mi, mj);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION ||
          old_cost < static_cast<unsigned char>(c))
        {
          master_grid.setCost(mi, mj, static_cast<unsigned char>(c));
        }
      }
    }
  }

  void matchSize() override
  {
    auto parent = layered_costmap_->getCostmap();
    std::lock_guard<std::mutex> lock(costmap_mutex_);

    origin_x_ = parent->getOriginX();
    origin_y_ = parent->getOriginY();
    resolution_ = parent->getResolution();
    count_x_ = parent->getSizeInCellsX();
    count_y_ = parent->getSizeInCellsY();

    const double size_x = count_x_ * resolution_;
    const double size_y = count_y_ * resolution_;
    buffer_ = std::make_unique<sea_surface_segmentation::OccupancyBuffer>(
      size_x, size_y, resolution_, parentCenter(parent), params_);
    RCLCPP_INFO_STREAM(
      logger_, "SeaSurfaceLayer occupancy buffer sized to " << count_x_ << "x" << count_y_);

    // The projection AABB has half-extent = maximum_range_ centred on the
    // camera; the buffer rolls with the parent costmap (half-extent ≈
    // min(size_x, size_y) / 2). When the camera is centred in the buffer,
    // observations past the buffer's half-extent project successfully but
    // grid_map drops them at write time. Surface the mismatch so the operator
    // can either raise the parent costmap size or lower maximum_range_; the
    // layer doesn't silently clamp (would mis-report effective reach).
    const double buffer_half_extent = std::min(size_x, size_y) / 2.0;
    if (maximum_range_ > buffer_half_extent) {
      RCLCPP_WARN_STREAM(
        logger_,
        "SeaSurfaceLayer maximum_range " << maximum_range_ << " m exceeds buffer "
          "half-extent " << buffer_half_extent << " m (parent costmap " << size_x
          << "x" << size_y << " m). Observations past the buffer edge will be "
          "dropped; raise the costmap size or lower maximum_range.");
    }
  }

private:
  // World position of the parent costmap's center (grid_map is positioned by
  // its center; nav2 Costmap2D by its lower-left origin).
  static grid_map::Position parentCenter(nav2_costmap_2d::Costmap2D * parent)
  {
    return grid_map::Position(
      parent->getOriginX() + parent->getSizeInCellsX() * parent->getResolution() / 2.0,
      parent->getOriginY() + parent->getSizeInCellsY() * parent->getResolution() / 2.0);
  }

  // Parse a flat int array of [x_min, y_min, x_max, y_max] tuples into
  // cv::Rect's (each rect's width/height = x_max - x_min / y_max - y_min).
  // The parameter is intentionally a flat int array because rclcpp doesn't
  // expose nested-list types; every 4 entries is one rectangle. Returns
  // true on success and populates `out`; returns false and writes a reason
  // into `why` on any validation failure (image bounds are NOT validated
  // here — the layer doesn't know image dimensions at init time, so masks
  // are clipped to image bounds at apply time in segmentsCallback).
  static bool parseMaskRects(
    const std::vector<int64_t> & flat,
    std::vector<cv::Rect> & out,
    std::string & why)
  {
    if (flat.size() % 4 != 0) {
      why = "image_mask_rects must have a multiple of 4 ints "
        "(x_min, y_min, x_max, y_max per rectangle)";
      return false;
    }
    out.clear();
    out.reserve(flat.size() / 4);
    for (size_t i = 0; i < flat.size(); i += 4) {
      const int x_min = static_cast<int>(flat[i]);
      const int y_min = static_cast<int>(flat[i + 1]);
      const int x_max = static_cast<int>(flat[i + 2]);
      const int y_max = static_cast<int>(flat[i + 3]);
      if (x_min < 0 || y_min < 0 || x_max <= x_min || y_max <= y_min) {
        why = "image_mask_rects entry must satisfy 0 <= x_min < x_max and "
          "0 <= y_min < y_max";
        return false;
      }
      out.emplace_back(x_min, y_min, x_max - x_min, y_max - y_min);
    }
    return true;
  }

  // Per-source segmentation callback. `src` is the source that owns this
  // subscription (captured by raw pointer in onInitialize's lambda); the
  // shared occupancy buffer accumulates evidence from every source's hits and
  // misses, so an obstacle in two cameras' overlap gets twice the evidence.
  void segmentsCallback(
    Source & src, const sensor_msgs::msg::Image::SharedPtr segments_msg)
  {
    // Pin the per-source camera model + shared buffer geometry under the
    // lock, then project off-lock. cameraInfoCallback swaps in a *fresh*
    // model (never mutates in place), so this local copy is an immutable
    // snapshot — no torn read / use-after-free during projection.
    // `resolution_` is captured here too because matchSize() can update it
    // from updateBounds(); reading it inside the off-lock projection would race.
    std::shared_ptr<image_geometry::PinholeCameraModel> camera_model;
    double res;
    double maximum_range;
    double min_grazing_deg;
    double obstacle_prob_min;
    double max_evidence_step;
    std::vector<cv::Rect> mask_rects;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      camera_model = src.camera_model;
      res = resolution_;
      maximum_range = maximum_range_;
      min_grazing_deg = min_grazing_angle_deg_;
      obstacle_prob_min = obstacle_prob_min_;
      max_evidence_step = max_evidence_step_;
      mask_rects = src.image_mask_rects;
    }
    if (!camera_model || res <= 0.0) {
      return;
    }
    try {
      // Camera pose in the world (costmap global) frame: translation = optical
      // center, rotation = camera-optical → world.
      const auto tf = tf_->lookupTransform(
        global_frame_id_, segments_msg->header.frame_id, segments_msg->header.stamp,
        std::chrono::seconds(1));
      // If this source has pixel-space masks (on-boat structures etc.), deep-
      // copy the image and black out the masked rectangles before projection
      // — black pixels fail both is_obstacle_ish (R near 0) and the green-
      // dominant water check, so they contribute no observations. Rectangles
      // are clipped to image bounds here (image dimensions weren't known at
      // param-set time). When no masks are configured, use the cheap zero-copy
      // toCvShare path so the common case isn't penalised.
      cv::Mat seg_image;
      if (mask_rects.empty()) {
        seg_image = cv_bridge::toCvShare(segments_msg, "rgb8")->image;
      } else {
        auto image_copy = cv_bridge::toCvCopy(segments_msg, "rgb8");
        const cv::Rect full(0, 0, image_copy->image.cols, image_copy->image.rows);
        for (const auto & r : mask_rects) {
          const cv::Rect clipped = r & full;
          if (clipped.area() > 0) {
            image_copy->image(clipped).setTo(cv::Vec3b(0, 0, 0));
          }
        }
        seg_image = image_copy->image;
      }

      const auto & t = tf.transform.translation;
      const auto & q = tf.transform.rotation;
      const cv::Vec3d camera_origin(t.x, t.y, t.z);
      const cv::Matx33d rotation_cam_to_world =
        sea_surface_segmentation::rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w);

      // INVERSE (cell→pixel) projection — iterate the world cells this camera can
      // reach (square AABB of half-width `maximum_range_` centred on the camera's
      // XY) and classify each by the pixel it covers. Fills each pixel's full
      // footprint (no gaps) and is naturally range-bounded. Water surface is z=0
      // in the tide-tracked global frame; the boat floats, so this holds at any
      // tide (see #19 / #10 H). Phase 3 of #19 restores the pre-#19 inverse
      // direction; see the offline `bag_to_costmap_video` for the A/B that drove
      // the change.
      //
      // TODO(#19, phase >5): the square AABB over-iterates by ~21% relative to
      // the inscribed Euclidean range gate, and many cells project off-image or
      // behind the camera. A per-camera FOV-cone cull (using the pinhole
      // model's image bounds + the camera pose) would prune those before
      // projection — meaningful CPU once we sustain N>1 cameras (phase 4).
      const auto observations = sea_surface_segmentation::project_observations_inverse(
        seg_image, *camera_model, camera_origin, rotation_cam_to_world,
        maximum_range, camera_origin[0], camera_origin[1], res, maximum_range,
        /*plane_z=*/0.0, min_grazing_deg, obstacle_prob_min, max_evidence_step);

      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!buffer_) {
        return;
      }
      for (const auto & obs : observations) {
        const grid_map::Position p(obs.x, obs.y);
        buffer_->accumulate(p, obs.log_odds);  // graded evidence (signed)
      }
      // Mark the layer current so LayeredCostmap::isCurrent() doesn't report a
      // stale layer once a frame has been ingested. Stays true once set — the
      // layer's persistence-and-decay semantics mean an unmodified buffer is
      // still up-to-date (decay runs in updateBounds every cycle).
      current_ = true;
    } catch (const std::exception & e) {
      if (auto node = node_.lock()) {
        // Throttle: a stuck TF or wrong-encoding image floods at the
        // segmentation publish rate (~5 Hz / camera × N cameras). Include the
        // source name + image frame + stamp so the log line is self-diagnostic.
        auto clock = node->get_clock();
        RCLCPP_WARN_THROTTLE(
          logger_, *clock, 5000,
          "SeaSurfaceLayer source '%s' projection failed for %s @ %d.%09d: %s",
          src.name.c_str(),
          segments_msg->header.frame_id.c_str(),
          segments_msg->header.stamp.sec, segments_msg->header.stamp.nanosec,
          e.what());
      }
    }
  }

  // Per-source camera_info callback. Updates the source's own camera model;
  // each source carries its own intrinsics (the 4-camera config has different
  // optics per direction, e.g. forward narrow-FOV vs side wide-FOV).
  void cameraInfoCallback(
    Source & src, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
  {
    // Build a fresh model off-lock, then swap the pointer under the lock. Readers
    // that already copied the old pointer keep using an unchanged object — no
    // in-place mutation of a model a concurrent segmentsCallback may be reading.
    auto model = std::make_shared<image_geometry::PinholeCameraModel>();
    model->fromCameraInfo(*camera_info_msg);
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    src.camera_model = model;
  }

  // Publish the graded occupancy as a nav_msgs/OccupancyGrid so a downstream
  // `SeaSurfaceRelayLayer` (or any consumer) can stamp the same graded gradient
  // into a different costmap without rerunning the segmentation projection.
  // Each cell carries its `occupancyAt` value: 100 at the lethal threshold, a
  // soft 1..99 ramp below it, and -1 ("no opinion") where the buffer has no
  // obstacle evidence, so the consumer never inadvertently clears another
  // layer's marks. Header frame is the costmap's global frame (e.g. `map_tide`).
  void publishLethalGrid()
  {
    if (!lethal_publisher_) {
      return;
    }
    auto node = node_.lock();
    if (!node) {
      return;
    }

    // Snapshot pattern: copy the grid_map under the lock, walk it off-lock.
    // The O(W*H) per-cell scan (~40k Eigen reads on a 200×200 buffer) would
    // otherwise serialize against every segmentsCallback wanting to write a
    // hit/miss. The copy is one Eigen MatrixXf deep-copy (≈ width*height*4 B
    // — 160 kB on the same 200×200) — far cheaper than holding the lock
    // through the scan.
    grid_map::GridMap map_copy;
    sea_surface_segmentation::OccupancyParams occ_params;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!buffer_) {
        return;
      }
      map_copy = buffer_->map();  // grid_map copy assignment = deep copy of Eigen data
      occ_params = params_;
    }
    // Standalone copy of the buffer so the O(W*H) occupancy scan runs off-lock
    // against the snapshot, yet still goes through `occupancyAt` (single source
    // of the graded ramp). The OccupancyBuffer is move-cheap (it holds the
    // GridMap we already deep-copied); rebuilding around `map_copy` avoids a
    // duplicate ramp implementation here.
    sea_surface_segmentation::OccupancyBuffer snapshot(map_copy, occ_params);

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = global_frame_id_;

    const auto size = map_copy.getSize();
    const auto center = map_copy.getPosition();
    msg.info.resolution = static_cast<float>(map_copy.getResolution());
    msg.info.width = static_cast<uint32_t>(size(0));
    msg.info.height = static_cast<uint32_t>(size(1));
    // grid_map is positioned by its center; OccupancyGrid origin is the
    // lower-left corner of cell (0, 0).
    msg.info.origin.position.x = center(0) - 0.5 * size(0) * map_copy.getResolution();
    msg.info.origin.position.y = center(1) - 0.5 * size(1) * map_copy.getResolution();
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(static_cast<size_t>(size(0)) * size(1), -1);

    // Walk OccupancyGrid cells in row-major order; for each cell's world
    // position, query the snapshot copy. Walking by world position rather
    // than direct index avoids dancing through grid_map's column-major /
    // circular-buffer index layout.
    for (uint32_t y = 0; y < msg.info.height; ++y) {
      for (uint32_t x = 0; x < msg.info.width; ++x) {
        const double wx = msg.info.origin.position.x + (x + 0.5) * msg.info.resolution;
        const double wy = msg.info.origin.position.y + (y + 0.5) * msg.info.resolution;
        const grid_map::Position p(wx, wy);
        msg.data[static_cast<size_t>(y) * msg.info.width + x] =
          static_cast<int8_t>(snapshot.occupancyAt(p));
      }
    }

    lethal_publisher_->publish(msg);
  }

  // Validate-then-apply param updates. Runs on the parameter service thread.
  // Builds a candidate copy of the live-tunable state from `params` (each named
  // entry overrides the candidate field), validates the candidate (NaN /
  // out-of-range / safety inversions are rejected via OccupancyBuffer::validate
  // + a maximum_range > 0 check), and only on success swaps into the live state
  // under `costmap_mutex_`. The buffer reinterprets accumulated evidence
  // against the new threshold/clamp immediately; new increments and decay rate
  // take effect on the next update cycle. Unknown / configure-time params
  // (topics) pass through as a no-op so the parameter store can still record
  // them.
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    sea_surface_segmentation::OccupancyParams candidate_occ;
    double candidate_max_range;
    double candidate_min_grazing_deg;
    double candidate_obstacle_prob_min;
    double candidate_max_evidence_step;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      candidate_occ = params_;
      candidate_max_range = maximum_range_;
      candidate_min_grazing_deg = min_grazing_angle_deg_;
      candidate_obstacle_prob_min = obstacle_prob_min_;
      candidate_max_evidence_step = max_evidence_step_;
    }
    // Per-source mask candidates: validated below, applied at the end under
    // the same lock that swaps the scalar candidates. Keyed by Source* so a
    // batched set targeting two sources in one call is atomic.
    std::unordered_map<Source *, std::vector<cv::Rect>> candidate_mask_updates;

    // Configure-time params — subscriber re-bind / publisher re-bind isn't
    // supported here, so accepting these silently would leave the parameter
    // store updated but the wiring stale (the operator's #10-K failure mode
    // in a different disguise). Reject with a clear message instead. Each
    // entry is matched as a suffix that may appear top-level (e.g.
    // `<layer>.observation_sources`) OR per-source (e.g.
    // `<layer>.<source>.segmentation_topic`).
    const std::vector<std::string> configure_time_suffixes{
      ".observation_sources", ".segmentation_topic", ".camera_info_topic",
      ".published_topic"};

    auto is_configure_time = [&](const std::string & n) {
      if (n.find(name_ + ".") != 0) {
        return false;  // not one of this layer's params
      }
      for (const auto & suffix : configure_time_suffixes) {
        if (n.size() >= suffix.size() &&
          n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0)
        {
          return true;
        }
      }
      return false;
    };

    // `as_double()` raises rclcpp::exceptions::InvalidParameterTypeException
    // when the parameter is set to a non-double value (e.g. a bool or string).
    // That exception would propagate out of the parameter service thread and
    // crash the node; wrap the dispatch so a wrong-type set returns a clean
    // SetParametersResult{successful=false, reason=…} instead.
    try {
      for (const auto & p : params) {
        const auto & n = p.get_name();

        if (is_configure_time(n)) {
          result.successful = false;
          result.reason = n + " is configure-time only (subscriber/publisher "
            "re-bind is not supported); restart the costmap to apply";
          return result;
        }

        if (n == name_ + ".obstacle_clamp") {
          candidate_occ.obstacle_clamp = p.as_double();
        } else if (n == name_ + ".clear_floor") {
          candidate_occ.clear_floor = p.as_double();
        } else if (n == name_ + ".free_threshold") {
          candidate_occ.free_threshold = p.as_double();
        } else if (n == name_ + ".lethal_threshold") {
          candidate_occ.lethal_threshold = p.as_double();
        } else if (n == name_ + ".decay_half_life_s") {
          candidate_occ.decay_half_life_s = p.as_double();
        } else if (n == name_ + ".obstacle_prob_min") {
          const double v = p.as_double();
          if (!std::isfinite(v) || v <= 0.0 || v >= 1.0) {
            result.successful = false;
            result.reason = "obstacle_prob_min must be finite and in (0, 1)";
            return result;
          }
          candidate_obstacle_prob_min = v;
        } else if (n == name_ + ".max_evidence_step") {
          const double v = p.as_double();
          if (!std::isfinite(v) || v <= 0.0) {
            result.successful = false;
            result.reason = "max_evidence_step must be finite and > 0";
            return result;
          }
          candidate_max_evidence_step = v;
        } else if (n == name_ + ".maximum_range") {
          const double v = p.as_double();
          if (!std::isfinite(v) || v <= 0.0) {
            result.successful = false;
            result.reason = "maximum_range must be finite and > 0";
            return result;
          }
          candidate_max_range = v;
        } else if (n == name_ + ".min_grazing_angle_deg") {
          const double v = p.as_double();
          if (!std::isfinite(v) || v < 0.0 || v >= 90.0) {
            result.successful = false;
            result.reason = "min_grazing_angle_deg must be finite and in [0, 90)";
            return result;
          }
          candidate_min_grazing_deg = v;
        } else {
          // Per-source pixel-space mask: <layer>.<source_name>.image_mask_rects.
          // Match by suffix, then resolve the middle segment to a known source.
          const std::string mask_suffix = ".image_mask_rects";
          if (n.size() > mask_suffix.size() &&
            n.compare(n.size() - mask_suffix.size(), mask_suffix.size(), mask_suffix) == 0)
          {
            for (const auto & src_uptr : sources_) {
              if (n == name_ + "." + src_uptr->name + mask_suffix) {
                std::vector<cv::Rect> rects;
                std::string why_mask;
                if (!parseMaskRects(p.as_integer_array(), rects, why_mask)) {
                  result.successful = false;
                  result.reason = why_mask;
                  return result;
                }
                candidate_mask_updates[src_uptr.get()] = std::move(rects);
                break;
              }
            }
          }
        }
      }
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
      result.successful = false;
      result.reason = std::string("invalid parameter type: ") + e.what();
      return result;
    }

    std::string why;
    if (!sea_surface_segmentation::OccupancyBuffer::validate(candidate_occ, why)) {
      result.successful = false;
      result.reason = why;
      return result;
    }

    std::lock_guard<std::mutex> lock(costmap_mutex_);
    params_ = candidate_occ;
    maximum_range_ = candidate_max_range;
    min_grazing_angle_deg_ = candidate_min_grazing_deg;
    obstacle_prob_min_ = candidate_obstacle_prob_min;
    max_evidence_step_ = candidate_max_evidence_step;
    for (auto & kv : candidate_mask_updates) {
      kv.first->image_mask_rects = std::move(kv.second);
    }
    if (buffer_) {
      buffer_->setParams(params_);
    }
    return result;
  }

  std::string global_frame_id_;

  // Parent-costmap geometry tracking (to detect true size/resolution changes
  // vs rolling-origin shifts).
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double resolution_ = 1.0;
  unsigned int count_x_ = 0;
  unsigned int count_y_ = 0;

  double maximum_range_ = 100.0;
  double min_grazing_angle_deg_ = 0.0;  // 0 = filter off (back-compat)
  // Graded evidence-model knobs (live-tunable; snapshotted per segmentsCallback).
  double obstacle_prob_min_ = 0.35;
  double max_evidence_step_ = 0.85;
  sea_surface_segmentation::OccupancyParams params_;
  std::unique_ptr<sea_surface_segmentation::OccupancyBuffer> buffer_;

  std::vector<std::unique_ptr<Source>> sources_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Optional publish-and-relay output (phase 8 / new offline-review scope):
  // when `published_topic_` is non-empty, the layer republishes its lethal
  // cells on it so a downstream `SeaSurfaceRelayLayer` can stamp them into
  // a different costmap (e.g. global_costmap, before inflation) without
  // running the segmentation projection a second time.
  std::string published_topic_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr lethal_publisher_;

  std::mutex costmap_mutex_;
};

}  // namespace sea_surface_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sea_surface_layer::SeaSurfaceLayer, nav2_costmap_2d::Layer)
