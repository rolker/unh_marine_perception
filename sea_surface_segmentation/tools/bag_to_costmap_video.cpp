// Offline replay: read recorded OAK segmentation from a bag, run it through the
// corrected (inverse cell→pixel) projection + OccupancyBuffer, and render a review
// MOSAIC per frame — so the layer's behavior can be seen on real water imagery,
// alongside what the cameras saw and what the boat's live costmap held, before
// deploying to the boat (#19).
//
// Mosaic layout (all costmap panels boat-centred, world-aligned, N-up):
//   top row:    [ port | fwd | stbd | aft ]   (segmentation tiles, 4:3)
//   bottom row: [ LIVE costmap (boat) | OURS (inverse, corrected) ]
//
// LIVE = the boat's own recorded /…/local_costmap/costmap, sampled into the same
// boat-centred window for 1:1 comparison. OURS = occupancy_buffer fed by the
// inverse projection (project_observations_inverse) — the corrected path. The
// earlier forward-projection panel was dropped; the offline A/B that justified
// the switch lives in git history / the #19 plan.
//
// Usage:
//   bag_to_costmap_video <bag_uri> <out_dir> [--window-m 120] [--res 0.25]
//       [--render-dt 0.2] [--max-range 150] [--start-s 0] [--end-s -1]
// --start-s/--end-s window the replay to [start,end] seconds from bag start
// (end<0 == to the end). The occupancy buffer accumulates from --start-s, so set
// it a little before the moment of interest to give the buffer warm-up time.
// Then: ffmpeg -framerate 10 -i <out_dir>/frame_%05d.png -pix_fmt yuv420p out.mp4

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "occupancy_buffer.hpp"
#include "segments_projection.hpp"

namespace
{
using sea_surface_segmentation::OccupancyBuffer;
using sea_surface_segmentation::OccupancyParams;

const std::array<const char *, 4> kCameras{"oak_forward", "oak_port", "oak_starboard", "oak_aft"};
const std::string kWorldFrame = "bizzy/map_tide";
const std::string kBoatFrame = "bizzy/base_link";
const std::string kCostmapTopic = "/bizzy/local_costmap/costmap";

double arg_double(int argc, char ** argv, const std::string & flag, double def)
{
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) { return std::atof(argv[i + 1]); }
  }
  return def;
}

tf2::TimePoint to_tf_time(const builtin_interfaces::msg::Time & t)
{
  return tf2::TimePoint(
    std::chrono::seconds(t.sec) + std::chrono::nanoseconds(t.nanosec));
}

template<typename T>
T deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  rclcpp::SerializedMessage serialized(*msg->serialized_data);
  T out;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &out);
  return out;
}

// Colour a log-odds value for the NEW-costmap render (BGR). Shared palette with
// the LIVE render below so the two panels are read the same way.
cv::Vec3b colour_logodds(double v, double threshold)
{
  if (std::isnan(v)) { return {110, 110, 110}; }          // unknown — grey
  if (v >= threshold) { return {40, 40, 230}; }            // lethal — red
  if (v > 0.0) {                                           // sub-threshold occupied — green→yellow
    double f = std::min(v / threshold, 1.0);
    return {30, static_cast<uchar>(120 + 100 * f), static_cast<uchar>(200 * f)};
  }
  return {70, 40, 20};                                     // free / negative — dark blue
}

// Colour a nav2 OccupancyGrid cost value (-1 unknown, 0 free, 100 lethal) with
// the SAME palette as colour_logodds so LIVE and NEW compare apples-to-apples.
cv::Vec3b colour_cost(int v)
{
  if (v < 0) { return {110, 110, 110}; }                   // NO_INFORMATION — grey
  if (v >= 99) { return {40, 40, 230}; }                   // inscribed/lethal — red
  if (v > 0) {                                             // intermediate cost — green→yellow
    double f = std::min(v / 99.0, 1.0);
    return {30, static_cast<uchar>(120 + 100 * f), static_cast<uchar>(200 * f)};
  }
  return {70, 40, 20};                                     // free — dark blue
}

// Render the NEW occupancy buffer into a boat-centred, world-aligned (N-up) panel.
cv::Mat render_new(const OccupancyBuffer & buffer, double bx, double by,
  int panel_px, double res, double threshold)
{
  cv::Mat panel(panel_px, panel_px, CV_8UC3);
  for (int v = 0; v < panel_px; ++v) {
    for (int u = 0; u < panel_px; ++u) {
      const double wx = bx + (u - panel_px / 2) * res;
      const double wy = by - (v - panel_px / 2) * res;  // image y down → world y up
      panel.at<cv::Vec3b>(v, u) = colour_logodds(
        buffer.logOdds(grid_map::Position(wx, wy)), threshold);
    }
  }
  return panel;
}

// Render the LIVE recorded costmap into the same boat-centred window/scale so it
// overlays the NEW panel 1:1. Samples the OccupancyGrid by world (x,y); the grid's
// origin is in its own frame (bizzy/map), which shares the xy plane with map_tide
// (the tide transform is z-only), so boat-xy and grid-xy are consistent.
cv::Mat render_live(const nav_msgs::msg::OccupancyGrid & grid, bool have_grid,
  double bx, double by, int panel_px, double res)
{
  cv::Mat panel(panel_px, panel_px, CV_8UC3, cv::Scalar(110, 110, 110));
  if (!have_grid || grid.info.resolution <= 0.0) { return panel; }
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  const double gres = grid.info.resolution;
  const int gw = static_cast<int>(grid.info.width);
  const int gh = static_cast<int>(grid.info.height);
  for (int v = 0; v < panel_px; ++v) {
    for (int u = 0; u < panel_px; ++u) {
      const double wx = bx + (u - panel_px / 2) * res;
      const double wy = by - (v - panel_px / 2) * res;
      const int gx = static_cast<int>(std::floor((wx - ox) / gres));
      const int gy = static_cast<int>(std::floor((wy - oy) / gres));
      int cost = -1;  // outside the live window reads as unknown
      if (gx >= 0 && gx < gw && gy >= 0 && gy < gh) {
        cost = grid.data[static_cast<std::size_t>(gy) * gw + gx];
      }
      panel.at<cv::Vec3b>(v, u) = colour_cost(cost);
    }
  }
  return panel;
}

// PROTOTYPE — inverse (ground-up) projection, for A/B comparison against the
// forward `project_observations`. Instead of mapping each image pixel to one cell,
// it iterates the world cells in the boat-centred window and asks "which pixel
// covers this cell?", so a single large/distant pixel fills EVERY cell in its
// footprint (no gaps) and the search is naturally range-bounded to the window
// (a near-horizon pixel can't smear past the window edge). Same contact-only
// classification as the forward path: a cell that projects to its column's
// waterline-contact pixel → obstacle; to water → free; to an above-contact body
// pixel → skipped (occluded/unknown). Lives in the tool, not the shared header —
// this is a comparison probe, not (yet) the layer's path.
std::vector<sea_surface_segmentation::OccupancyObservation> project_observations_inverse(
  const cv::Mat & mask, const image_geometry::PinholeCameraModel & cam,
  const cv::Vec3d & cam_origin, const cv::Matx33d & rot_cam_to_world,
  double max_range, double bx, double by, double res, double half_extent, double plane_z = 0.0)
{
  // Lowest (nearest) waterline contact per column — same primitive as forward.
  std::vector<int> contact_row(mask.cols, -1);
  for (int col = 0; col < mask.cols; ++col) {
    for (int row = mask.rows - 1; row >= 0; --row) {
      if (sea_surface_segmentation::is_waterline_contact_pixel(mask, row, col)) {
        contact_row[col] = row;
        break;
      }
    }
  }

  const cv::Matx33d rot_world_to_cam = rot_cam_to_world.t();
  std::vector<sea_surface_segmentation::OccupancyObservation> obs;
  const int n = static_cast<int>(std::lround(2.0 * half_extent / res));
  for (int iy = 0; iy < n; ++iy) {
    for (int ix = 0; ix < n; ++ix) {
      const double wx = bx + (ix - n / 2) * res;
      const double wy = by + (iy - n / 2) * res;
      const cv::Vec3d d(wx - cam_origin[0], wy - cam_origin[1], plane_z - cam_origin[2]);
      if (std::sqrt(d.dot(d)) > max_range) { continue; }     // beyond sensor range
      const cv::Vec3d pc = rot_world_to_cam * d;             // cell in camera optical frame
      if (pc[2] <= 0.0) { continue; }                        // behind the camera
      const cv::Point2d uv = cam.project3dToPixel(cv::Point3d(pc[0], pc[1], pc[2]));
      const int u = static_cast<int>(std::lround(uv.x));
      const int v = static_cast<int>(std::lround(uv.y));
      if (u < 0 || u >= mask.cols || v < 0 || v >= mask.rows) { continue; }
      const cv::Vec3b px = mask.at<cv::Vec3b>(v, u);
      // Class channels (rgb8): R=obstacle prob, G=water prob, B=sky prob.
      // Only positively-observed water marks the cell free; sky/ambiguous → skip
      // (a ground cell on z=0 sampling a sky pixel is a geometric inconsistency,
      // and clearing on it would erase legitimate hits from other frames/cameras).
      if (sea_surface_segmentation::is_obstacle_pixel(px)) {
        if (contact_row[u] >= 0 && v >= contact_row[u]) {
          obs.push_back({wx, wy, true});                     // waterline contact → hit
        }
        // else: above the contact = occluded body → unobserved
      } else if (px[1] > px[0] && px[1] > px[2]) {           // green-dominant = water
        obs.push_back({wx, wy, false});                      // positively observed water → miss
      }
      // else: sky (blue-dominant) or ambiguous → skip (no observation)
    }
  }
  return obs;
}

void label(cv::Mat & img, const std::string & text, cv::Point org, double scale = 0.5)
{
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, scale, {0, 0, 0}, 3);
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, scale, {255, 255, 255}, 1);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr,
      "usage: %s <bag_uri> <out_dir> [--window-m 120] [--res 0.25] "
      "[--render-dt 0.2] [--max-range 150] [--start-s 0] [--end-s -1]\n", argv[0]);
    return 1;
  }
  const std::string bag_uri = argv[1];
  const std::string out_dir = argv[2];
  const double window_m = arg_double(argc, argv, "--window-m", 120.0);
  const double res = arg_double(argc, argv, "--res", 0.25);
  const double render_dt = arg_double(argc, argv, "--render-dt", 0.2);
  const double max_range = arg_double(argc, argv, "--max-range", 150.0);
  const double start_s = arg_double(argc, argv, "--start-s", 0.0);
  const double end_s = arg_double(argc, argv, "--end-s", -1.0);

  // Reject non-positive numeric args — a typoed `--res --window-m 200` would
  // otherwise leave res=0 and divide-by-zero in panel_px / setGeometry; a
  // negative window_m would make an invalid cv::Mat. start_s/end_s are
  // intentionally allowed any sign (end_s < 0 means "to end of bag").
  for (auto [name, val] : std::initializer_list<std::pair<const char *, double>>{
    {"--window-m", window_m}, {"--res", res},
    {"--render-dt", render_dt}, {"--max-range", max_range}})
  {
    if (!(val > 0.0)) {
      std::fprintf(stderr, "error: %s must be > 0 (got %g)\n", name, val);
      return 1;
    }
  }

  OccupancyParams params;  // defaults — same as the layer's compiled-in defaults

  // topic → camera name and the reverse, for placing each segmentation tile.
  std::map<std::string, std::string> seg_topic_to_cam;
  std::vector<std::string> seg_topics, info_topics;
  for (const auto * cam : kCameras) {
    const std::string seg = std::string("/bizzy/sensors/cameras/") + cam + "/segmentation";
    seg_topic_to_cam[seg] = cam;
    seg_topics.push_back(seg);
    info_topics.push_back(seg + "/camera_info");
  }
  auto is_in = [](const std::vector<std::string> & v, const std::string & s) {
    return std::find(v.begin(), v.end(), s) != v.end();
  };

  // ---- Pass 1: fill the TF buffer (whole-bag cache) + collect camera models. ----
  tf2::BufferCore tf_buffer(tf2::durationFromSec(7200.0));
  std::map<std::string, image_geometry::PinholeCameraModel> camera_models;  // keyed by optical frame
  std::map<std::string, std::string> optical_to_cam;  // optical frame → camera name

  std::fprintf(stderr, "pass 1: indexing tf + camera_info ...\n");
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag_uri);
    while (reader.has_next()) {
      auto bag_msg = reader.read_next();
      const std::string & topic = bag_msg->topic_name;
      if (topic == "/tf" || topic == "/tf_static") {
        auto tfm = deserialize<tf2_msgs::msg::TFMessage>(bag_msg);
        const bool is_static = (topic == "/tf_static");
        for (const auto & tr : tfm.transforms) {
          tf_buffer.setTransform(tr, "bag", is_static);
        }
      } else if (is_in(info_topics, topic)) {
        auto info = deserialize<sensor_msgs::msg::CameraInfo>(bag_msg);
        camera_models[info.header.frame_id].fromCameraInfo(info);
        // /…/<cam>/segmentation/camera_info → <cam>
        const std::string base = topic.substr(0, topic.size() - std::string("/camera_info").size());
        if (seg_topic_to_cam.count(base)) {
          optical_to_cam[info.header.frame_id] = seg_topic_to_cam[base];
        }
      }
    }
  }
  std::fprintf(stderr, "  camera models: %zu\n", camera_models.size());

  // ---- Pass 2: replay segmentation through the pipeline, render mosaic frames. ----
  // Layout: top row = 4 camera tiles [ port | fwd | stbd | aft ];
  //         bottom row = [ LIVE costmap (boat) | OURS (inverse, corrected) ].
  const int panel_px = static_cast<int>(std::lround(window_m / res));  // costmap panel (square)
  const int cam_w = panel_px / 2;            // 4 tiles span the 2-panel bottom width
  const int cam_h = cam_w * 3 / 4;           // 128x96 mask is 4:3
  const int header_px = 22;                  // label band above each row
  const int canvas_w = panel_px * 2;
  const int cam_row_y = header_px;                        // camera tiles start here
  const int cm_label_y = header_px + cam_h;              // costmap label band
  const int cm_row_y = cm_label_y + header_px;           // costmap panels start here
  const int canvas_h = cm_row_y + panel_px;
  const double half_extent = window_m / 2.0;
  OccupancyBuffer buffer(window_m, window_m, res, grid_map::Position(0.0, 0.0), params);

  // Camera tiles left→right: port, forward, starboard, aft.
  const std::array<const char *, 4> cam_order{"oak_port", "oak_forward", "oak_starboard", "oak_aft"};
  std::map<std::string, cv::Mat> latest_mask_bgr;  // camera name → latest seg (BGR, for display)

  nav_msgs::msg::OccupancyGrid live_costmap;
  bool have_costmap = false;
  std::string live_frame_reported;

  std::fprintf(stderr, "pass 2: replaying (panel %dx%d, canvas %dx%d) window [%.0f, %.0f]s ...\n",
    panel_px, panel_px, canvas_w, canvas_h, start_s, end_s);
  rosbag2_cpp::Reader reader;
  reader.open(bag_uri);
  const int64_t bag_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    reader.get_metadata().starting_time.time_since_epoch()).count();
  const int64_t start_ns = bag_start_ns + static_cast<int64_t>(start_s * 1e9);
  const int64_t end_ns = (end_s < 0.0) ? std::numeric_limits<int64_t>::max()
    : bag_start_ns + static_cast<int64_t>(end_s * 1e9);

  double last_render = -1.0;
  int frame_idx = 0;
  long processed = 0;
  // Per-camera accounting so a silently-dropped camera (bad TF, no obs) is
  // observable instead of just a blank tile / empty quadrant. `obs_inv_cells`
  // counts world cells the inverse projection actually applied hit/miss to
  // (different unit from the forward path's per-pixel counts — don't compare
  // to historical numbers).
  std::map<std::string, long> seg_seen, tf_fail, obs_inv_cells;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (bag_msg->recv_timestamp < start_ns) { continue; }
    if (bag_msg->recv_timestamp > end_ns) { break; }
    const std::string & topic = bag_msg->topic_name;

    if (topic == kCostmapTopic) {
      live_costmap = deserialize<nav_msgs::msg::OccupancyGrid>(bag_msg);
      have_costmap = true;
      if (live_frame_reported.empty()) {
        live_frame_reported = live_costmap.header.frame_id;
        std::fprintf(stderr, "  live costmap frame: '%s' (res %.3f m, %ux%u)\n",
          live_frame_reported.c_str(), live_costmap.info.resolution,
          live_costmap.info.width, live_costmap.info.height);
      }
      continue;
    }
    if (!is_in(seg_topics, topic)) { continue; }

    auto img_msg = deserialize<sensor_msgs::msg::Image>(bag_msg);
    const std::string & optical_frame = img_msg.header.frame_id;
    auto model_it = camera_models.find(optical_frame);
    if (model_it == camera_models.end()) { continue; }
    const double stamp_s = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9;
    const auto tf_time = to_tf_time(img_msg.header.stamp);

    // Resolve the camera name up front (for the mosaic tile + per-camera counters).
    std::string cam_name;
    if (auto it = optical_to_cam.find(optical_frame); it != optical_to_cam.end()) {
      cam_name = it->second;
      ++seg_seen[cam_name];
    }

    // Decode + stash the display tile BEFORE the TF lookup: the mosaic should show
    // each camera's imagery regardless of whether a pose is available for this
    // exact stamp (a TF miss must not blank the tile — it only blocks projection).
    cv::Mat mask;
    try {
      mask = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
    } catch (const cv_bridge::Exception &) { continue; }
    if (!cam_name.empty()) {
      cv::Mat bgr;
      cv::cvtColor(mask, bgr, cv::COLOR_RGB2BGR);
      latest_mask_bgr[cam_name] = bgr;
    }

    try {
      // Camera pose in world + boat pose in world at the image stamp.
      const auto cam_tf = tf_buffer.lookupTransform(kWorldFrame, optical_frame, tf_time);
      const auto boat_tf = tf_buffer.lookupTransform(kWorldFrame, kBoatFrame, tf_time);

      const auto & ct = cam_tf.transform.translation;
      const auto & cq = cam_tf.transform.rotation;
      const cv::Vec3d camera_origin(ct.x, ct.y, ct.z);
      const cv::Matx33d rot =
        sea_surface_segmentation::rotation_matrix_from_quaternion(cq.x, cq.y, cq.z, cq.w);

      // Moving window: re-centre the buffer on the boat, decay, then ingest.
      const double bx = boat_tf.transform.translation.x;
      const double by = boat_tf.transform.translation.y;
      buffer.move(grid_map::Position(bx, by));
      buffer.decay(stamp_s);

      // INVERSE (cell→pixel) projection — the corrected path; fills each pixel's footprint.
      for (const auto & o : project_observations_inverse(
          mask, model_it->second, camera_origin, rot, max_range, bx, by, res, half_extent, 0.0))
      {
        const grid_map::Position p(o.x, o.y);
        if (o.obstacle) { buffer.hit(p); } else { buffer.miss(p); }
        if (!cam_name.empty()) { ++obs_inv_cells[cam_name]; }
      }
      ++processed;

      // Render the mosaic at the requested cadence.
      if (last_render < 0.0 || stamp_s - last_render >= render_dt) {
        last_render = stamp_s;
        cv::Mat canvas(canvas_h, canvas_w, CV_8UC3, cv::Scalar(20, 20, 20));

        // --- Top row: camera tiles, port | fwd | stbd | aft. ---
        for (int k = 0; k < 4; ++k) {
          const std::string cam = cam_order[k];
          const int x = k * cam_w;
          auto it = latest_mask_bgr.find(cam);
          cv::Mat tile;
          if (it != latest_mask_bgr.end()) {
            cv::resize(it->second, tile, cv::Size(cam_w, cam_h), 0, 0, cv::INTER_NEAREST);
          } else {
            tile = cv::Mat(cam_h, cam_w, CV_8UC3, cv::Scalar(0, 0, 0));
          }
          tile.copyTo(canvas(cv::Rect(x, cam_row_y, cam_w, cam_h)));
          label(canvas, cam.substr(4), {x + 4, header_px - 6}, 0.45);  // drop "oak_"
        }
        char tlabel[48];
        std::snprintf(tlabel, sizeof(tlabel), "t=%.1fs", stamp_s);
        label(canvas, tlabel, {canvas_w - 110, header_px - 6}, 0.45);

        // --- Bottom row: LIVE costmap | OURS (inverse, corrected). --- (boat-centred, N-up)
        cv::Mat live = render_live(live_costmap, have_costmap, bx, by, panel_px, res);
        cv::circle(live, {panel_px / 2, panel_px / 2}, 4, {0, 255, 255}, -1);
        live.copyTo(canvas(cv::Rect(0, cm_row_y, panel_px, panel_px)));

        cv::Mat ours = render_new(buffer, bx, by, panel_px, res, params.lethal_threshold);
        cv::circle(ours, {panel_px / 2, panel_px / 2}, 4, {0, 255, 255}, -1);
        ours.copyTo(canvas(cv::Rect(panel_px, cm_row_y, panel_px, panel_px)));

        // --- Costmap row labels. ---
        label(canvas, "LIVE costmap (boat)", {6, cm_label_y + 16}, 0.5);
        char nhdr[80];
        std::snprintf(nhdr, sizeof(nhdr), "OURS - inverse  win=%.0fm", window_m);
        label(canvas, nhdr, {panel_px + 6, cm_label_y + 16}, 0.5);

        char path[512];
        std::snprintf(path, sizeof(path), "%s/frame_%05d.png", out_dir.c_str(), frame_idx++);
        cv::imwrite(path, canvas);
      }
    } catch (const tf2::TransformException &) {
      // TF not available for this stamp — projection skipped (tile already shown).
      if (!cam_name.empty()) { ++tf_fail[cam_name]; }
      continue;
    }
  }

  std::fprintf(stderr, "done: %ld segmentation frames processed, %d mosaic frames in %s\n",
    processed, frame_idx, out_dir.c_str());
  std::fprintf(stderr, "per-camera (inverse path): seg_seen / tf_fail / cells_fed_to_buffer\n");
  for (const auto * cam : kCameras) {
    std::fprintf(stderr, "  %-14s %6ld / %6ld / %10ld%s\n", cam,
      seg_seen[cam], tf_fail[cam], obs_inv_cells[cam],
      seg_seen[cam] > 0 && tf_fail[cam] == seg_seen[cam] ? "   <-- ALL TF FAILED" : "");
  }
  std::fprintf(stderr,
    "assemble: ffmpeg -framerate 10 -i %s/frame_%%05d.png -pix_fmt yuv420p %s/mosaic.mp4\n",
    out_dir.c_str(), out_dir.c_str());
  return 0;
}
