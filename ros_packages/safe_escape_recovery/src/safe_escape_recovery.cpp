#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <costmap_2d/footprint.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <boost/thread/mutex.hpp>

namespace safe_escape_recovery {

class SafeEscapeRecovery : public nav_core::RecoveryBehavior {
 public:
  SafeEscapeRecovery() : global_(nullptr), local_(nullptr), tf_(nullptr),
                         initialized_(false), have_plan_(false) {}

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* global,
                  costmap_2d::Costmap2DROS* local) override {
    if (initialized_) return;
    global_ = global;
    local_ = local;
    tf_ = tf;
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("max_distance", max_distance_, 0.30);
    private_nh.param("min_distance", min_distance_, 0.12);
    private_nh.param("safety_margin", safety_margin_, 0.04);
    private_nh.param("speed", speed_, 0.10);
    std::string global_plan_topic;
    private_nh.param("global_plan_topic", global_plan_topic,
                     std::string("/move_base/NavfnROS/plan"));
    plan_sub_ = ros::NodeHandle().subscribe(
        global_plan_topic, 1, &SafeEscapeRecovery::planCallback, this);
    velocity_ = private_nh.advertise<geometry_msgs::Twist>("/my_car/cmd_vel_nav", 1);
    initialized_ = true;
  }

  void planCallback(const nav_msgs::PathConstPtr& message) {
    boost::mutex::scoped_lock lock(plan_mutex_);
    global_plan_ = *message;
    have_plan_ = !message->poses.empty();
  }

  void runBehavior() override {
    if (!initialized_ || local_ == nullptr) return;
    geometry_msgs::PoseStamped start;
    if (!local_->getRobotPose(start)) {
      ROS_WARN("Safe escape: robot pose unavailable");
      return;
    }
    const double yaw = tf2::getYaw(start.pose.orientation);
    double best_v = 0.0, best_w = 0.0, best_progress = 0.0;
    double best_path_score = -std::numeric_limits<double>::infinity();
    double best_cross_track = 0.0;
    unsigned char best_endpoint_cost = costmap_2d::LETHAL_OBSTACLE;
    bool found_clear_endpoint = false;
    std::vector<geometry_msgs::Point> local_path;
    const bool have_path = transformGlobalPlan(start, local_path) && local_path.size() >= 2;
    PathProjection start_projection;
    const bool start_on_path = have_path && projectToPath(
        start.pose.position.x, start.pose.position.y, local_path, start_projection);
    // Search low-curvature arcs in both directions; each candidate is checked
    // against the complete footprint before it is sent to the drivetrain.
    // Short, high-curvature arcs repeatedly produced tiny sideways loops on
    // photo-route legs, so recovery must make useful forward progress without
    // swinging the chassis sharply away from the global plan.
    for (double v : {-0.16, -0.12, 0.12, 0.16}) {
      for (double w : {-0.38, 0.0, 0.38}) {
        for (double duration : {1.20, 1.60, 2.00}) {
          double progress = 0.0;
          double end_x = 0.0, end_y = 0.0;
          unsigned char endpoint_cost = costmap_2d::LETHAL_OBSTACLE;
          if (!arcClear(start, yaw, v, w, duration, progress, endpoint_cost,
                        end_x, end_y)) continue;

          // A swept arc can be collision-free yet leave the chassis with no
          // room to turn or replan. Prefer endpoints outside the inscribed
          // obstacle halo. When Navfn has a path, prefer an arc that advances
          // along it without increasing cross-track error.
          const bool clear_endpoint =
              endpoint_cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
          double path_score = progress;
          double cross_track = 0.0;
          if (start_on_path) {
            PathProjection end_projection;
            if (!projectToPath(end_x, end_y, local_path, end_projection)) continue;
            cross_track = end_projection.cross_track;
            path_score = end_projection.arclength - start_projection.arclength -
                2.0 * (end_projection.cross_track - start_projection.cross_track);
            // Do not accept an arc that only changes pose locally or moves
            // away from the active route. MoveBase will replan the same goal
            // after recovery; an unhelpful arc just repeats the oscillation.
            if (path_score < 0.03 ||
                cross_track > start_projection.cross_track + 0.05) continue;
          }
          if (clear_endpoint) {
            const bool better_without_path =
                !start_on_path &&
                (endpoint_cost < best_endpoint_cost ||
                 (endpoint_cost == best_endpoint_cost && progress > best_progress));
            const bool better_with_path = start_on_path &&
                (path_score > best_path_score + 1e-3 ||
                 (std::abs(path_score - best_path_score) <= 1e-3 &&
                  (endpoint_cost < best_endpoint_cost ||
                   (endpoint_cost == best_endpoint_cost && progress > best_progress))));
            if (!found_clear_endpoint || better_without_path || better_with_path) {
              found_clear_endpoint = true;
              best_v = v;
              best_w = w;
              best_progress = progress;
              best_endpoint_cost = endpoint_cost;
              best_path_score = path_score;
              best_cross_track = cross_track;
            }
          } else if (!found_clear_endpoint && endpoint_cost < best_endpoint_cost) {
            best_v = v;
            best_w = w;
            best_progress = progress;
            best_endpoint_cost = endpoint_cost;
            best_path_score = path_score;
            best_cross_track = cross_track;
          }
        }
      }
    }
    ROS_WARN("Safe escape probe: v=%.2f w=%.2f progress=%.3f endpoint_cost=%u clear=%s path=%s score=%.3f cross=%.3f->%.3f",
             best_v, best_w, best_progress,
             static_cast<unsigned>(best_endpoint_cost),
             found_clear_endpoint ? "yes" : "no", start_on_path ? "yes" : "no",
             best_path_score, start_on_path ? start_projection.cross_track : 0.0,
             best_cross_track);
    const bool leaves_global_path = start_on_path &&
        best_cross_track > start_projection.cross_track + 0.05;
    if (best_progress < min_distance_ || !found_clear_endpoint ||
        (start_on_path && (best_path_score < 0.03 || leaves_global_path))) {
      ROS_WARN("Safe escape: no arc ends with enough footprint clearance");
      stop();
      return;
    }

    ROS_WARN("Safe escape: arc v=%.2f w=%.2f", best_v, best_w);
    ros::WallRate rate(15);
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(3.0);
    ros::WallTime last_progress = ros::WallTime::now();
    double moved = 0.0;
    while (ros::ok() && ros::WallTime::now() < deadline) {
      geometry_msgs::PoseStamped current;
      if (!local_->getRobotPose(current)) break;
      moved = std::hypot(current.pose.position.x - start.pose.position.x,
                         current.pose.position.y - start.pose.position.y);
      if (moved >= best_progress) break;
      if (moved > 0.01) {
        last_progress = ros::WallTime::now();
      }
      if ((ros::WallTime::now() - last_progress).toSec() > 2.0) {
        ROS_WARN("Safe escape: stopped because the chassis did not move");
        break;
      }
      geometry_msgs::Twist command;
      command.linear.x = best_v;
      command.angular.z = best_w;
      velocity_.publish(command);
      rate.sleep();
    }
    stop();
    ROS_WARN("Safe escape: moved %.2f m, move_base will replan the same goal", moved);
  }

 private:
  struct PathProjection {
    double arclength;
    double cross_track;
  };

  bool transformGlobalPlan(const geometry_msgs::PoseStamped& local_pose,
                           std::vector<geometry_msgs::Point>& local_path) {
    if (tf_ == nullptr || local_ == nullptr) return false;
    nav_msgs::Path plan;
    {
      boost::mutex::scoped_lock lock(plan_mutex_);
      if (!have_plan_) return false;
      plan = global_plan_;
    }
    if (plan.poses.size() < 2) return false;
    const std::string source_frame = plan.header.frame_id.empty() && global_
        ? global_->getGlobalFrameID() : plan.header.frame_id;
    if (source_frame.empty() || local_pose.header.frame_id.empty()) return false;
    if (!plan.header.stamp.isZero() &&
        (ros::Time::now() - plan.header.stamp).toSec() > 3.0) return false;

    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(local_pose.header.frame_id, source_frame,
                                       ros::Time(0), ros::Duration(0.10));
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0, "Safe escape: global path transform unavailable: %s",
                        error.what());
      return false;
    }
    const double heading = tf2::getYaw(transform.transform.rotation);
    const double c = std::cos(heading), s = std::sin(heading);
    local_path.clear();
    local_path.reserve(plan.poses.size());
    for (const auto& pose : plan.poses) {
      geometry_msgs::Point point;
      point.x = transform.transform.translation.x +
          c * pose.pose.position.x - s * pose.pose.position.y;
      point.y = transform.transform.translation.y +
          s * pose.pose.position.x + c * pose.pose.position.y;
      point.z = 0.0;
      local_path.push_back(point);
    }
    return true;
  }

  bool projectToPath(double x, double y,
                     const std::vector<geometry_msgs::Point>& path,
                     PathProjection& projection) const {
    if (path.size() < 2) return false;
    double cumulative = 0.0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      const double dx = path[i + 1].x - path[i].x;
      const double dy = path[i + 1].y - path[i].y;
      const double segment_length = std::hypot(dx, dy);
      if (segment_length < 1e-6) continue;
      const double t = std::max(0.0, std::min(1.0,
          ((x - path[i].x) * dx + (y - path[i].y) * dy) /
          (segment_length * segment_length)));
      const double px = path[i].x + t * dx;
      const double py = path[i].y + t * dy;
      const double distance = std::hypot(x - px, y - py);
      if (distance < best_distance) {
        best_distance = distance;
        projection.arclength = cumulative + t * segment_length;
        projection.cross_track = distance;
      }
      cumulative += segment_length;
    }
    return std::isfinite(best_distance);
  }

  double freeDistance(const geometry_msgs::PoseStamped& pose, double yaw, int direction) {
    const std::vector<geometry_msgs::Point> footprint = local_->getRobotFootprint();
    costmap_2d::Costmap2D* grid = local_->getCostmap();
    costmap_2d::Costmap2D::mutex_t* mutex = grid->getMutex();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*mutex);
    std::vector<geometry_msgs::Point> oriented;
    std::vector<costmap_2d::MapLocation> polygon;
    std::vector<costmap_2d::MapLocation> cells;
    for (double distance = 0.0; distance <= max_distance_ + safety_margin_; distance += 0.02) {
      const double x = pose.pose.position.x + direction * distance * std::cos(yaw);
      const double y = pose.pose.position.y + direction * distance * std::sin(yaw);
      costmap_2d::transformFootprint(x, y, yaw, footprint, oriented);
      polygon.clear();
      bool outside = false;
      for (const auto& point : oriented) {
        unsigned int mx, my;
        if (!grid->worldToMap(point.x, point.y, mx, my)) {
          outside = true;
          break;
        }
        costmap_2d::MapLocation cell;
        cell.x = mx;
        cell.y = my;
        polygon.push_back(cell);
      }
      if (outside) return distance;
      cells.clear();
      grid->convexFillCells(polygon, cells);
      cells.insert(cells.end(), polygon.begin(), polygon.end());
      for (const auto& cell : cells) {
        // 253 is the inflated/inscribed halo, not a measured obstacle.  DWA
        // may reject it, but recovery may leave it while still forbidding
        // lethal 254, unknown 255, and the static white-line layer.
        if (grid->getCost(cell.x, cell.y) >= costmap_2d::LETHAL_OBSTACLE)
          return distance;
      }
    }
    return max_distance_ + safety_margin_;
  }

  bool arcClear(const geometry_msgs::PoseStamped& pose, double yaw, double v,
                double w, double duration, double& progress,
                unsigned char& endpoint_cost, double& end_x, double& end_y) {
    // Never use a recovery arc that consumes more than the configured
    // displacement budget. The previous longest-free-arc policy could move
    // 0.32 m directly toward a wall and leave no room for the next turn.
    if (std::abs(v) * duration > max_distance_) return false;
    const std::vector<geometry_msgs::Point> footprint = local_->getRobotFootprint();
    costmap_2d::Costmap2D* grid = local_->getCostmap();
    costmap_2d::Costmap2D::mutex_t* mutex = grid->getMutex();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*mutex);
    double x = pose.pose.position.x, y = pose.pose.position.y, theta = yaw;
    auto footprintClear = [&](double px, double py, double heading,
                              unsigned char& max_cost) {
      std::vector<geometry_msgs::Point> oriented;
      costmap_2d::transformFootprint(px, py, heading, footprint, oriented);
      std::vector<costmap_2d::MapLocation> polygon, cells;
      for (const auto& point : oriented) {
        unsigned int mx, my;
        if (!grid->worldToMap(point.x, point.y, mx, my)) return false;
        costmap_2d::MapLocation cell;
        cell.x = mx;
        cell.y = my;
        polygon.push_back(cell);
      }
      grid->convexFillCells(polygon, cells);
      cells.insert(cells.end(), polygon.begin(), polygon.end());
      max_cost = 0;
      for (const auto& cell : cells) {
        const unsigned char cost = grid->getCost(cell.x, cell.y);
        if (cost >= costmap_2d::LETHAL_OBSTACLE) return false;
        max_cost = std::max(max_cost, cost);
      }
      return true;
    };

    double elapsed = 0.0;
    endpoint_cost = 0;
    while (true) {
      if (!footprintClear(x, y, theta, endpoint_cost)) return false;
      if (elapsed >= duration) break;
      const double dt = std::min(0.04, duration - elapsed);
      if (std::abs(w) < 1e-6) {
        x += v * dt * std::cos(theta);
        y += v * dt * std::sin(theta);
      } else {
        const double next = theta + w * dt;
        x += v / w * (std::sin(next) - std::sin(theta));
        y -= v / w * (std::cos(next) - std::cos(theta));
        theta = next;
      }
      elapsed += dt;
    }
    progress = std::hypot(x - pose.pose.position.x, y - pose.pose.position.y);
    end_x = x;
    end_y = y;
    return true;
  }

  void stop() {
    ros::WallRate rate(20);
    for (int i = 0; i < 5 && ros::ok(); ++i) {
      velocity_.publish(geometry_msgs::Twist());
      rate.sleep();
    }
  }

  costmap_2d::Costmap2DROS* global_;
  costmap_2d::Costmap2DROS* local_;
  tf2_ros::Buffer* tf_;
  ros::Publisher velocity_;
  ros::Subscriber plan_sub_;
  boost::mutex plan_mutex_;
  nav_msgs::Path global_plan_;
  bool initialized_;
  bool have_plan_;
  double max_distance_, min_distance_, safety_margin_, speed_;
};

}  // namespace safe_escape_recovery

PLUGINLIB_EXPORT_CLASS(safe_escape_recovery::SafeEscapeRecovery, nav_core::RecoveryBehavior)
