#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <XmlRpcValue.h>
#include <boost/thread/locks.hpp>

namespace forward_path_follower {

class ForwardPathFollower : public nav_core::BaseLocalPlanner {
 public:
  ForwardPathFollower()
      : costmap_ros_(nullptr), costmap_(nullptr), tf_(nullptr), initialized_(false),
        have_plan_(false), goal_reached_(false), plan_index_(0),
        max_speed_(0.30), min_speed_(0.08), max_yaw_rate_(0.75),
        lookahead_(0.32), xy_tolerance_(0.03), yaw_tolerance_(0.02),
        target_xy_tolerance_(0.03), target_yaw_tolerance_(0.02),
        final_heading_position_margin_(0.0),
        prediction_time_(0.75), prediction_step_(0.05),
        linear_accel_(0.75), angular_accel_(1.8), final_heading_gain_(1.8), last_v_(0.0),
        last_w_(0.0), have_last_command_(false), path_alignment_active_(false),
        final_heading_active_(false),
        goal_settle_tracking_(false), goal_settle_x_(0.0), goal_settle_y_(0.0),
        goal_settle_yaw_(0.0),
        path_progress_(0.0),
        current_route_progress_(0.0), route_length_(0.0), have_goal_(false),
        route_params_valid_(false), leg_route_start_s_(0.0),
        leg_route_goal_s_(0.0), route_cursor_s_(0.0),
        start_backtrack_tolerance_(0.10) {}

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override {
    if (initialized_) return;
    if (tf == nullptr || costmap_ros == nullptr || costmap_ros->getCostmap() == nullptr) {
      ROS_ERROR("ForwardPathFollower: missing TF buffer or costmap");
      return;
    }
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_.reset(new base_local_planner::CostmapModel(*costmap_));

    ros::NodeHandle private_nh("~/forward_path_follower");
    private_nh.param("max_speed", max_speed_, 0.30);
    private_nh.param("min_speed", min_speed_, 0.08);
    private_nh.param("max_yaw_rate", max_yaw_rate_, 0.75);
    private_nh.param("lookahead", lookahead_, 0.32);
    private_nh.param("xy_tolerance", xy_tolerance_, 0.03);
    private_nh.param("yaw_tolerance", yaw_tolerance_, 0.02);
    private_nh.param("prediction_time", prediction_time_, 0.75);
    private_nh.param("prediction_step", prediction_step_, 0.05);
    private_nh.param("linear_accel", linear_accel_, 0.75);
    private_nh.param("angular_accel", angular_accel_, 1.8);
    private_nh.param("final_heading_gain", final_heading_gain_, 1.8);
    private_nh.param("final_heading_position_margin", final_heading_position_margin_, 0.0);
    if (!std::isfinite(final_heading_position_margin_) || final_heading_position_margin_ < 0.0) {
      ROS_ERROR("ForwardPathFollower: final_heading_position_margin must be finite and nonnegative");
      return;
    }
    if (!std::isfinite(final_heading_gain_) || final_heading_gain_ <= 0.0) {
      ROS_ERROR("ForwardPathFollower: final_heading_gain must be positive and finite");
      return;
    }
    private_nh.param("start_backtrack_tolerance", start_backtrack_tolerance_, 0.10);
    XmlRpc::XmlRpcValue raw_points;
    if (!private_nh.getParam("route_contract/points", raw_points) ||
        raw_points.getType() != XmlRpc::XmlRpcValue::TypeArray || raw_points.size() < 2) {
      ROS_ERROR("ForwardPathFollower: ordered route contract is missing");
      return;
    }
    route_points_map_.clear();
    route_cumulative_.clear();
    route_cumulative_.push_back(0.0);
    for (int i = 0; i < raw_points.size(); ++i) {
      if (raw_points[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
          raw_points[i].size() < 2) {
        ROS_ERROR("ForwardPathFollower: invalid route contract point %d", i);
        return;
      }
      auto number = [](const XmlRpc::XmlRpcValue& value) -> double {
        if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
          return static_cast<int>(value);
        if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
          return static_cast<double>(value);
        throw std::runtime_error("route point coordinate is not numeric");
      };
      geometry_msgs::Point point;
      try {
        point.x = number(raw_points[i][0]);
        point.y = number(raw_points[i][1]);
      } catch (const std::exception& error) {
        ROS_ERROR("ForwardPathFollower: %s", error.what());
        return;
      }
      point.z = 0.0;
      if (!route_points_map_.empty()) {
        const auto& previous = route_points_map_.back();
        route_cumulative_.push_back(route_cumulative_.back() +
            std::hypot(point.x - previous.x, point.y - previous.y));
      }
      route_points_map_.push_back(point);
    }
    route_length_ = route_cumulative_.back();
    if (route_length_ <= 0.1) {
      ROS_ERROR("ForwardPathFollower: ordered route has no usable length");
      return;
    }

    local_plan_pub_ = nh_.advertise<nav_msgs::Path>(
        "/move_base/ForwardPathFollower/local_plan", 1);
    progress_pub_ = nh_.advertise<std_msgs::Float32>(
        "/move_base/ForwardPathFollower/progress", 1);
    status_pub_ = nh_.advertise<std_msgs::String>(
        "/move_base/ForwardPathFollower/status", 1, true);
    initialized_ = true;
    ROS_INFO("ForwardPathFollower initialized (%s), forward cap %.2f m/s",
             name.c_str(), max_speed_);
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    if (!initialized_ || plan.empty()) return false;
    const bool had_plan = have_plan_;
    plan_ = plan;
    double requested_x = 0.0, requested_y = 0.0, requested_yaw = 0.0;
    double requested_route_start = 0.0, requested_route_goal = 0.0;
    double requested_xy_tolerance = xy_tolerance_;
    double requested_yaw_tolerance = yaw_tolerance_;
    const bool have_requested_pose =
        ros::param::get("/move_base/forward_path_follower/target_x", requested_x) &&
        ros::param::get("/move_base/forward_path_follower/target_y", requested_y) &&
        ros::param::get("/move_base/forward_path_follower/target_yaw", requested_yaw);
    bool new_goal = false;
    const bool have_route_interval =
        ros::param::get("/move_base/forward_path_follower/route_start_s", requested_route_start) &&
        ros::param::get("/move_base/forward_path_follower/route_goal_s", requested_route_goal);
    const bool have_target_xy_tolerance = ros::param::get(
        "/move_base/forward_path_follower/target_xy_tolerance", requested_xy_tolerance);
    const bool have_target_yaw_tolerance = ros::param::get(
        "/move_base/forward_path_follower/target_yaw_tolerance", requested_yaw_tolerance);
    target_xy_tolerance_ = have_target_xy_tolerance ?
        std::max(0.005, requested_xy_tolerance) : xy_tolerance_;
    target_yaw_tolerance_ = have_target_yaw_tolerance ?
        std::max(0.005, requested_yaw_tolerance) : yaw_tolerance_;
    if (have_requested_pose) {
      new_goal = !have_goal_ ||
          std::hypot(requested_x - target_goal_.pose.position.x,
                     requested_y - target_goal_.pose.position.y) > 0.01 ||
          std::abs(angles::shortest_angular_distance(
              tf2::getYaw(target_goal_.pose.orientation), requested_yaw)) > 0.01;
      target_goal_.header.frame_id = "map";
      target_goal_.pose.position.x = requested_x;
      target_goal_.pose.position.y = requested_y;
      target_goal_.pose.position.z = 0.0;
      target_goal_.pose.orientation.x = 0.0;
      target_goal_.pose.orientation.y = 0.0;
      target_goal_.pose.orientation.z = std::sin(requested_yaw / 2.0);
      target_goal_.pose.orientation.w = std::cos(requested_yaw / 2.0);
      have_goal_ = true;
    }
    bool new_leg = false;
    if (have_route_interval) {
      new_leg = !route_params_valid_ ||
          std::abs(requested_route_start - leg_route_start_s_) > 0.01 ||
          std::abs(requested_route_goal - leg_route_goal_s_) > 0.01;
      leg_route_start_s_ = requested_route_start;
      leg_route_goal_s_ = requested_route_goal;
      route_params_valid_ = true;
      if (new_leg) route_cursor_s_ = leg_route_start_s_;
    }
    have_plan_ = true;
    // move_base refreshes Navfn plans while the action goal is unchanged.
    // Keep the settled state and command ramp across those refreshes; reset
    // only when the route interval or requested pose starts a new leg.
    if (!had_plan || new_goal || new_leg) {
      plan_index_ = 0;
      goal_reached_ = false;
      goal_settle_tracking_ = false;
      path_progress_ = 0.0;
      last_v_ = 0.0;
      last_w_ = 0.0;
      have_last_command_ = false;
      path_alignment_active_ = false;
      final_heading_active_ = false;
    }
    return true;
  }

  bool computeVelocityCommands(geometry_msgs::Twist& command) override {
    command = geometry_msgs::Twist();
    if (!initialized_ || !have_plan_ || plan_.empty()) {
      publishStatus("NO_PLAN");
      return false;
    }

    geometry_msgs::PoseStamped robot;
    if (!costmap_ros_->getRobotPose(robot)) {
      publishStatus("POSE_UNAVAILABLE");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> path;
    if (!buildOrderedPath(robot, path)) {
      publishStatus("ORDERED_ROUTE_UNAVAILABLE");
      return false;
    }
    plan_index_ = 0;
    if (path.size() < 2) {
      publishStatus("PLAN_TOO_SHORT");
      return false;
    }

    const double x = robot.pose.position.x;
    const double y = robot.pose.position.y;
    const double yaw = tf2::getYaw(robot.pose.orientation);
    const double gx = path.back().pose.position.x;
    const double gy = path.back().pose.position.y;
    const double goal_yaw = tf2::getYaw(path.back().pose.orientation);
    const double goal_distance = std::hypot(gx - x, gy - y);

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    if (!footprintClear(x, y, yaw)) {
      publishStatus("CURRENT_FOOTPRINT_BLOCKED");
      publishPlan(path);
      return true;
    }

    const double yaw_error = angles::shortest_angular_distance(yaw, goal_yaw);
    // Enter the photo turn inside the acceptance radius, leaving room for
    // position changes during rotation. Completion still uses the original
    // tolerances; the entry margin never permits an out-of-tolerance result.
    const double final_heading_entry_tolerance = std::max(0.005,
        target_xy_tolerance_ - std::min(final_heading_position_margin_,
                                       0.5 * target_xy_tolerance_));
    if (!final_heading_active_ && goal_distance <= final_heading_entry_tolerance) {
      final_heading_active_ = true;
    } else if (final_heading_active_ &&
               (goal_distance > target_xy_tolerance_ + 0.02 ||
                (goal_distance > target_xy_tolerance_ &&
                 std::abs(yaw_error) <= target_yaw_tolerance_))) {
      final_heading_active_ = false;
    }
    // AMCL can move the map pose slightly while the car turns in place.
    // Finish that turn before returning to path tracking; otherwise crossing
    // the XY threshold alternates the path heading and the photo heading.
    // Goal completion still requires the original XY and yaw tolerances.
    if (final_heading_active_) {
      path_alignment_active_ = false;
      if (std::abs(yaw_error) <= target_yaw_tolerance_) {
        smoothCommand(0.0, 0.0, command);
        const bool command_stopped = std::abs(last_v_) < 0.01 &&
                                     std::abs(last_w_) < 0.01;
        const ros::WallTime now = ros::WallTime::now();
        if (!command_stopped) {
          goal_settle_tracking_ = false;
        } else if (!goal_settle_tracking_) {
          goal_settle_tracking_ = true;
          goal_settle_started_ = now;
          goal_settle_x_ = x;
          goal_settle_y_ = y;
          goal_settle_yaw_ = yaw;
        } else {
          const double settle_distance = std::hypot(x - goal_settle_x_,
                                                    y - goal_settle_y_);
          const double settle_yaw = std::abs(angles::shortest_angular_distance(
              goal_settle_yaw_, yaw));
          if (settle_distance > 0.003 || settle_yaw > 0.01) {
            goal_settle_started_ = now;
            goal_settle_x_ = x;
            goal_settle_y_ = y;
            goal_settle_yaw_ = yaw;
          }
        }
        goal_reached_ = goal_settle_tracking_ && command_stopped &&
            (now - goal_settle_started_).toSec() >= 0.35;
        publishStatus(goal_reached_ ? "GOAL_REACHED" : "GOAL_SETTLE");
        publishPlan(path);
        return true;
      }
      goal_settle_tracking_ = false;
      if (!rotationSweepClear(x, y, yaw, goal_yaw)) {
        smoothCommand(0.0, 0.0, command);
        publishStatus("FINAL_TURN_BLOCKED");
        publishPlan(path);
        return true;
      }
      const double w = std::max(-max_yaw_rate_,
          std::min(max_yaw_rate_, final_heading_gain_ * yaw_error));
      smoothCommand(0.0, w, command);
      publishStatus("FINAL_HEADING");
      publishPlan(path);
      return true;
    }

    goal_settle_tracking_ = false;

    const size_t nearest = nearestPlanIndex(path, x, y);
    const geometry_msgs::Point long_carrot =
        lookaheadPoint(path, nearest, x, y, lookahead_);
    const double long_heading = std::atan2(long_carrot.y - y, long_carrot.x - x);
    const double short_lookahead = std::min(0.16, lookahead_);
    const geometry_msgs::Point short_carrot =
        lookaheadPoint(path, nearest, x, y, short_lookahead);
    const double short_heading = std::atan2(short_carrot.y - y,
                                            short_carrot.x - x);
    bool turn_ahead = false;
    double turn_distance = std::numeric_limits<double>::infinity();
    double path_distance = 0.0;
    double previous_x = x, previous_y = y;
    double previous_path_heading = tf2::getYaw(path[nearest].pose.orientation);
    for (size_t i = nearest + 1; i < path.size(); ++i) {
      const double next_x = path[i].pose.position.x;
      const double next_y = path[i].pose.position.y;
      const double segment = std::hypot(next_x - previous_x, next_y - previous_y);
      const double segment_heading = tf2::getYaw(path[i].pose.orientation);
      if (path_distance <= 0.80 && std::abs(angles::shortest_angular_distance(
              previous_path_heading, segment_heading)) > 0.35) {
        turn_ahead = true;
        turn_distance = path_distance;
        break;
      }
      path_distance += segment;
      previous_x = next_x;
      previous_y = next_y;
      previous_path_heading = segment_heading;
    }
    const bool near_path_turn = turn_ahead ||
        std::abs(angles::shortest_angular_distance(short_heading, long_heading)) > 0.22;
    const geometry_msgs::Point carrot = near_path_turn ? short_carrot : long_carrot;
    const double desired_heading = near_path_turn ? short_heading : long_heading;
    const double heading_error = angles::shortest_angular_distance(yaw, desired_heading);
    if (path_alignment_active_ && std::abs(heading_error) <= 0.35)
      path_alignment_active_ = false;
    if (path_alignment_active_ || std::abs(heading_error) > 0.55) {
      if (!path_alignment_active_ &&
          (std::abs(last_v_) > 0.01 || std::abs(last_w_) > 0.04)) {
        smoothCommand(0.0, 0.0, command);
        publishStatus("ALIGNMENT_BRAKING");
        publishPlan(path);
        return true;
      }
      path_alignment_active_ = true;
      // Once alignment begins, keep rotating until the lower exit threshold
      // is reached. Do not re-enter braking on every angular command update.
      if (!rotationSweepClear(x, y, yaw, desired_heading)) {
        smoothCommand(0.0, 0.0, command);
        publishStatus("PATH_ALIGNMENT_BLOCKED");
        publishPlan(path);
        return true;
      }
      const double w = std::max(-max_yaw_rate_,
          std::min(max_yaw_rate_, 1.8 * heading_error));
      smoothCommand(0.0, w, command);
      publishStatus("ALIGNING_TO_PATH");
      publishPlan(path);
      return true;
    }
    double speed = std::min(max_speed_, std::sqrt(2.0 * linear_accel_ *
        std::max(0.0, goal_distance - target_xy_tolerance_)));
    if (turn_ahead && turn_distance < 0.45)
      speed = std::min(speed, 0.18);
    if (goal_distance < 0.30)
      speed = std::min(speed, std::max(0.025, 0.8 * goal_distance));
    speed *= std::max(0.32, std::cos(std::min(1.45, std::abs(heading_error))));
    const double speed_floor = goal_distance < 0.16 ? 0.025 : min_speed_;
    speed = std::max(speed_floor, speed);

    const double curvature = 2.0 * std::sin(heading_error) /
                             std::max(lookahead_, std::hypot(carrot.x - x, carrot.y - y));
    const double preferred_w = std::max(-max_yaw_rate_,
        std::min(max_yaw_rate_, speed * curvature));
    double best_score = -std::numeric_limits<double>::infinity();
    double best_w = 0.0;
    bool found = false;
    // Evaluate the computed curvature itself as well as straight motion and
    // perturbations. Every candidate uses the same collision check and score.
    const double offsets[] = {0.0, -0.10, 0.10, -0.20, 0.20, -0.35, 0.35};
    const int candidate_count = static_cast<int>(sizeof(offsets) / sizeof(offsets[0]));
    for (int candidate = -1; candidate < candidate_count; ++candidate) {
      const double requested_w = candidate < 0 ? 0.0 : preferred_w + offsets[candidate];
      const double w = std::max(-max_yaw_rate_,
          std::min(max_yaw_rate_, requested_w));
      double end_x = x, end_y = y, end_yaw = yaw, max_cost = 0.0;
      if (!simulateClear(x, y, yaw, speed, w, end_x, end_y, end_yaw, max_cost)) continue;
      const double start_carrot_distance = std::hypot(carrot.x - x, carrot.y - y);
      const double end_carrot_distance = std::hypot(carrot.x - end_x, carrot.y - end_y);
      const double carrot_progress = start_carrot_distance - end_carrot_distance;
      const double cross_track = nearestPathDistance(path, end_x, end_y, nearest);
      const double heading_end = std::abs(angles::shortest_angular_distance(
          end_yaw, std::atan2(carrot.y - end_y, carrot.x - end_x)));
      const double score = 4.0 * carrot_progress - 5.0 * cross_track -
          0.35 * std::abs(w - preferred_w) - 0.003 * max_cost - 0.12 * heading_end;
      if (!found || score > best_score) {
        best_score = score;
        best_w = w;
        found = true;
      }
    }

    if (!found) {
      smoothCommand(0.0, 0.0, command);
      publishStatus("OBSTACLE_HOLD");
    } else {
      smoothCommand(speed, best_w, command);
      publishStatus("TRACKING");
    }
    publishPlan(path);
    std_msgs::Float32 progress;
    progress.data = static_cast<float>(current_route_progress_);
    progress_pub_.publish(progress);
    return true;
  }

  bool isGoalReached() override { return goal_reached_; }

 private:
  struct RouteProjection {
    double arclength;
    double distance;
    size_t segment;
    double fraction;
  };

  RouteProjection projectRoute(double x, double y) const {
    RouteProjection best{0.0, std::numeric_limits<double>::infinity(), 0, 0.0};
    for (size_t i = 0; i + 1 < route_points_map_.size(); ++i) {
      const auto& a = route_points_map_[i];
      const auto& b = route_points_map_[i + 1];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double length2 = dx * dx + dy * dy;
      if (length2 < 1e-10) continue;
      const double fraction = std::max(0.0, std::min(1.0,
          ((x - a.x) * dx + (y - a.y) * dy) / length2));
      const double px = a.x + fraction * dx;
      const double py = a.y + fraction * dy;
      const double distance = std::hypot(x - px, y - py);
      if (distance < best.distance) {
        best = {route_cumulative_[i] + fraction * std::sqrt(length2),
                distance, i, fraction};
      }
    }
    return best;
  }

  RouteProjection projectRouteInInterval(double x, double y,
                                        double low, double high) const {
    RouteProjection best{0.0, std::numeric_limits<double>::infinity(), 0, 0.0};
    for (size_t i = 0; i + 1 < route_points_map_.size(); ++i) {
      const auto& a = route_points_map_[i];
      const auto& b = route_points_map_[i + 1];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double length2 = dx * dx + dy * dy;
      if (length2 < 1e-10) continue;
      const double fraction = std::max(0.0, std::min(1.0,
          ((x - a.x) * dx + (y - a.y) * dy) / length2));
      const double px = a.x + fraction * dx;
      const double py = a.y + fraction * dy;
      const double distance = std::hypot(x - px, y - py);
      const double base = route_cumulative_[i] + fraction * std::sqrt(length2);
      for (int lap = -1; lap <= 2; ++lap) {
        const double progress = base + lap * route_length_;
        if (progress >= low - 1e-6 && progress <= high + 1e-6 &&
            distance < best.distance) {
          best = {progress, distance, i, fraction};
        }
      }
    }
    return best;
  }

  void routePointAt(double arclength, geometry_msgs::Point& point,
                    double& tangent) const {
    double wrapped = std::fmod(arclength, route_length_);
    if (wrapped < 0.0) wrapped += route_length_;
    size_t segment = route_points_map_.size() - 2;
    for (size_t i = 0; i + 1 < route_points_map_.size(); ++i) {
      if (wrapped <= route_cumulative_[i + 1] + 1e-9) {
        segment = i;
        break;
      }
    }
    const auto& a = route_points_map_[segment];
    const auto& b = route_points_map_[segment + 1];
    const double length = route_cumulative_[segment + 1] - route_cumulative_[segment];
    const double fraction = length > 1e-9 ?
        std::max(0.0, std::min(1.0,
            (wrapped - route_cumulative_[segment]) / length)) : 0.0;
    point.x = a.x + fraction * (b.x - a.x);
    point.y = a.y + fraction * (b.y - a.y);
    point.z = 0.0;
    tangent = std::atan2(b.y - a.y, b.x - a.x);
  }

  bool transformPose(const geometry_msgs::PoseStamped& source,
                     const std::string& target,
                     geometry_msgs::PoseStamped& transformed) const {
    try {
      if (source.header.frame_id.empty() || source.header.frame_id == target) {
        transformed = source;
        transformed.header.frame_id = target;
      } else {
        // The robot pose is stamped by odometry, while AMCL updates map->odom
        // less often. Transform against the latest correction to avoid a
        // short TF extrapolation race at photo-point heading settle.
        const auto transform = tf_->lookupTransform(
            target, source.header.frame_id, ros::Time(0), ros::Duration(0.10));
        tf2::doTransform(source, transformed, transform);
        transformed.header.frame_id = target;
      }
      return true;
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0, "ForwardPathFollower: route transform failed: %s",
                        error.what());
      return false;
    }
  }

  bool buildOrderedPath(const geometry_msgs::PoseStamped& robot,
                        std::vector<geometry_msgs::PoseStamped>& output) {
    if (plan_.empty()) return false;
    geometry_msgs::PoseStamped robot_map, goal_map;
    if (!transformPose(robot, "map", robot_map)) return false;
    const double start_interval_low = route_params_valid_ ?
        std::max(leg_route_start_s_ - 0.18,
                 route_cursor_s_ - start_backtrack_tolerance_) : 0.0;
    const double start_interval_high = route_params_valid_ ?
        leg_route_goal_s_ + 0.12 : route_length_;
    RouteProjection start = route_params_valid_ ? projectRouteInInterval(
        robot_map.pose.position.x, robot_map.pose.position.y,
        start_interval_low, start_interval_high) : projectRoute(
            robot_map.pose.position.x, robot_map.pose.position.y);
    geometry_msgs::PoseStamped target = have_goal_ ? target_goal_ : plan_.back();
    if (!transformPose(target, "map", goal_map)) return false;
    const RouteProjection goal = route_params_valid_ ? projectRouteInInterval(
        goal_map.pose.position.x, goal_map.pose.position.y,
        leg_route_goal_s_ - 0.18, leg_route_goal_s_ + 0.18) : projectRoute(
            goal_map.pose.position.x, goal_map.pose.position.y);
    const double projected_start_s = start.arclength;
    if (start.distance > 0.18 || goal.distance > 0.18) {
      ROS_WARN_THROTTLE(2.0,
          "ForwardPathFollower: ordered route unavailable: start=(%.3f,%.3f) "
          "s=%.3f d=%.3f interval=[%.3f,%.3f], goal=(%.3f,%.3f) "
          "s=%.3f d=%.3f interval=[%.3f,%.3f], cursor=%.3f",
          robot_map.pose.position.x, robot_map.pose.position.y,
          start.arclength, start.distance, start_interval_low,
          start_interval_high, goal_map.pose.position.x,
          goal_map.pose.position.y, goal.arclength, goal.distance,
          route_params_valid_ ? leg_route_goal_s_ - 0.18 : 0.0,
          route_params_valid_ ? leg_route_goal_s_ + 0.18 : route_length_,
          route_cursor_s_);
      return false;
    }
    if (route_params_valid_) {
      if (start.arclength > leg_route_goal_s_ + 0.12) {
        ROS_WARN_THROTTLE(2.0,
            "ForwardPathFollower: route start %.3f passed leg goal %.3f",
            start.arclength, leg_route_goal_s_);
        return false;
      }
      if (start.arclength > route_cursor_s_)
        route_cursor_s_ = std::min(leg_route_goal_s_, start.arclength);
      start.arclength = route_cursor_s_;
    }
    double end_s = route_params_valid_ ? leg_route_goal_s_ : goal.arclength;
    current_route_progress_ = start.arclength;
    if (!route_params_valid_ && end_s < start.arclength - 0.01) end_s += route_length_;
    if (end_s < start.arclength - 0.02) {
      ROS_WARN_THROTTLE(2.0,
          "ForwardPathFollower: ordered interval reversed start=%.3f end=%.3f",
          start.arclength, end_s);
      return false;
    }
    if (end_s - start.arclength > route_length_ + 0.02) {
      ROS_WARN_THROTTLE(2.0,
          "ForwardPathFollower: ordered interval exceeds lap start=%.3f end=%.3f length=%.3f",
          start.arclength, end_s, route_length_);
      return false;
    }

    const std::string local_frame = costmap_ros_->getGlobalFrameID();
    output.clear();
    const double target_distance = std::hypot(
        goal_map.pose.position.x - robot_map.pose.position.x,
        goal_map.pose.position.y - robot_map.pose.position.y);
    if (route_params_valid_ &&
        projected_start_s >= leg_route_goal_s_ - 0.12 &&
        target_distance <= 0.20) {
      // The saved photo pose may sit a few centimetres off the route center.
      // At the terminal approach, follow that exact checked connector instead
      // of projecting behind the robot and starting the whole lane leg again.
      geometry_msgs::PoseStamped local_goal;
      if (!transformPose(target, local_frame, local_goal)) return false;
      output.push_back(robot);
      output.push_back(local_goal);
      current_route_progress_ = leg_route_goal_s_;
      return true;
    }
    output.reserve(static_cast<size_t>((end_s - start.arclength) / 0.05) + 3);
    output.push_back(robot);
    geometry_msgs::Point start_point;
    double start_tangent = 0.0;
    routePointAt(start.arclength, start_point, start_tangent);
    geometry_msgs::PoseStamped start_map_pose;
    start_map_pose.header.frame_id = "map";
    start_map_pose.header.stamp = ros::Time(0);
    start_map_pose.pose.position = start_point;
    tf2::Quaternion start_q;
    start_q.setRPY(0.0, 0.0, start_tangent);
    start_map_pose.pose.orientation = tf2::toMsg(start_q);
    geometry_msgs::PoseStamped start_local_pose;
    if (!transformPose(start_map_pose, local_frame, start_local_pose)) return false;
    output.push_back(start_local_pose);
    for (double s = start.arclength + 0.05; s < end_s - 0.02; s += 0.05) {
      geometry_msgs::Point point;
      double tangent = 0.0;
      routePointAt(s, point, tangent);
      geometry_msgs::PoseStamped map_pose;
      map_pose.header.frame_id = "map";
      map_pose.header.stamp = ros::Time(0);
      map_pose.pose.position = point;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, tangent);
      map_pose.pose.orientation = tf2::toMsg(q);
      geometry_msgs::PoseStamped local_pose;
      if (!transformPose(map_pose, local_frame, local_pose)) return false;
      output.push_back(local_pose);
    }
    geometry_msgs::PoseStamped local_goal;
    if (!transformPose(target, local_frame, local_goal)) return false;
    output.push_back(local_goal);
    return output.size() >= 2;
  }

  size_t nearestPlanIndex(const std::vector<geometry_msgs::PoseStamped>& path,
                          double x, double y) {
    const size_t begin = std::min(plan_index_, path.size() - 1);
    const size_t previous = begin;
    size_t best = begin;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = begin; i < path.size(); ++i) {
      const double dx = path[i].pose.position.x - x;
      const double dy = path[i].pose.position.y - y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best = i; }
      if (i > begin && d2 > best_d2 + 0.20 * 0.20) break;
    }
    plan_index_ = std::max(plan_index_, best);
    for (size_t i = previous; i < plan_index_ && i + 1 < path.size(); ++i) {
      path_progress_ += std::hypot(
          path[i + 1].pose.position.x - path[i].pose.position.x,
          path[i + 1].pose.position.y - path[i].pose.position.y);
    }
    return plan_index_;
  }

  geometry_msgs::Point lookaheadPoint(
      const std::vector<geometry_msgs::PoseStamped>& path, size_t start,
      double x, double y, double distance) const {
    double remaining = distance;
    double px = x, py = y;
    for (size_t i = start; i < path.size(); ++i) {
      const double nx = path[i].pose.position.x;
      const double ny = path[i].pose.position.y;
      const double segment = std::hypot(nx - px, ny - py);
      if (segment >= remaining && segment > 1e-8) {
        const double t = remaining / segment;
        geometry_msgs::Point point;
        point.x = px + t * (nx - px);
        point.y = py + t * (ny - py);
        point.z = 0.0;
        return point;
      }
      remaining -= segment;
      px = nx;
      py = ny;
    }
    geometry_msgs::Point point = path.back().pose.position;
    point.z = 0.0;
    return point;
  }

  double nearestPathDistance(
      const std::vector<geometry_msgs::PoseStamped>& path,
      double x, double y, size_t start) const {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = start; i < path.size(); ++i) {
      best = std::min(best, std::hypot(
          path[i].pose.position.x - x, path[i].pose.position.y - y));
    }
    return best;
  }

  bool footprintClear(double x, double y, double yaw) const {
    return world_model_->footprintCost(
        x, y, yaw, costmap_ros_->getRobotFootprint()) >= 0.0;
  }

  bool simulateClear(double x, double y, double yaw, double v, double w,
                     double& end_x, double& end_y, double& end_yaw,
                     double& max_cost) const {
    end_x = x;
    end_y = y;
    end_yaw = yaw;
    max_cost = 0.0;
    double elapsed = 0.0;
    while (elapsed <= prediction_time_ + 1e-9) {
      const double cost = world_model_->footprintCost(
          end_x, end_y, end_yaw, costmap_ros_->getRobotFootprint());
      if (cost < 0.0) return false;
      max_cost = std::max(max_cost, cost);
      const double dt = std::min(prediction_step_, prediction_time_ - elapsed);
      if (dt <= 0.0) break;
      if (std::abs(w) < 1e-6) {
        end_x += v * dt * std::cos(end_yaw);
        end_y += v * dt * std::sin(end_yaw);
      } else {
        const double next_yaw = end_yaw + w * dt;
        end_x += v / w * (std::sin(next_yaw) - std::sin(end_yaw));
        end_y -= v / w * (std::cos(next_yaw) - std::cos(end_yaw));
        end_yaw = next_yaw;
      }
      elapsed += dt;
    }
    return true;
  }

  bool rotationSweepClear(double x, double y, double from, double to) const {
    const double delta = angles::shortest_angular_distance(from, to);
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(delta) / 0.025)));
    for (int i = 0; i <= steps; ++i) {
      const double yaw = from + delta * i / static_cast<double>(steps);
      if (!footprintClear(x, y, yaw)) return false;
    }
    return true;
  }

  void smoothCommand(double v, double w, geometry_msgs::Twist& output) {
    const ros::WallTime now = ros::WallTime::now();
    const double dt = have_last_command_ ?
        std::max(0.01, std::min(0.2, (now - last_command_time_).toSec())) : 0.1;
    last_v_ += std::max(-linear_accel_ * dt,
        std::min(linear_accel_ * dt, v - last_v_));
    last_w_ += std::max(-angular_accel_ * dt,
        std::min(angular_accel_ * dt, w - last_w_));
    output.linear.x = std::max(0.0, std::min(max_speed_, last_v_));
    output.angular.z = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, last_w_));
    last_command_time_ = now;
    have_last_command_ = true;
  }

  void publishStatus(const std::string& value) {
    std_msgs::String message;
    message.data = value;
    status_pub_.publish(message);
  }

  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    nav_msgs::Path message;
    message.header.frame_id = costmap_ros_->getGlobalFrameID();
    message.header.stamp = ros::Time::now();
    message.poses = path;
    local_plan_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  tf2_ros::Buffer* tf_;
  std::unique_ptr<base_local_planner::CostmapModel> world_model_;
  bool initialized_;
  bool have_plan_;
  bool goal_reached_;
  size_t plan_index_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  geometry_msgs::PoseStamped target_goal_;
  bool have_goal_;
  bool route_params_valid_;
  double leg_route_start_s_, leg_route_goal_s_, route_cursor_s_;
  double start_backtrack_tolerance_;
  std::vector<geometry_msgs::Point> route_points_map_;
  std::vector<double> route_cumulative_;
  double route_length_;
  double max_speed_, min_speed_, max_yaw_rate_, lookahead_;
  double xy_tolerance_, yaw_tolerance_;
  double target_xy_tolerance_, target_yaw_tolerance_;
  double final_heading_position_margin_;
  double prediction_time_, prediction_step_;
  double linear_accel_, angular_accel_, final_heading_gain_;
  double last_v_, last_w_;
  bool have_last_command_;
  bool path_alignment_active_;
  bool final_heading_active_;
  bool goal_settle_tracking_;
  ros::WallTime goal_settle_started_;
  double goal_settle_x_, goal_settle_y_, goal_settle_yaw_;
  ros::WallTime last_command_time_;
  double path_progress_;
  double current_route_progress_;
  ros::Publisher local_plan_pub_, progress_pub_, status_pub_;
};

}  // namespace forward_path_follower

PLUGINLIB_EXPORT_CLASS(forward_path_follower::ForwardPathFollower,
                       nav_core::BaseLocalPlanner)
