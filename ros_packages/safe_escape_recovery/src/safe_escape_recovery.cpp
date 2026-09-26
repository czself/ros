#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <costmap_2d/footprint.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/recovery_behavior.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <tf2/utils.h>

namespace safe_escape_recovery {

class SafeEscapeRecovery : public nav_core::RecoveryBehavior {
 public:
  SafeEscapeRecovery() : local_(nullptr), initialized_(false) {}

  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*,
                  costmap_2d::Costmap2DROS* local) override {
    if (initialized_) return;
    local_ = local;
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("max_distance", max_distance_, 0.30);
    private_nh.param("min_distance", min_distance_, 0.12);
    private_nh.param("safety_margin", safety_margin_, 0.04);
    private_nh.param("speed", speed_, 0.10);
    velocity_ = private_nh.advertise<geometry_msgs::Twist>("/my_car/cmd_vel_nav", 1);
    initialized_ = true;
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
    // A straight reverse is not enough at a corner.  Search short constant-
    // curvature arcs in both directions; each candidate is checked against
    // the complete footprint before it is ever sent to the drivetrain.
    for (double v : {-0.16, -0.12, 0.12, 0.16}) {
      for (double w : {-1.10, -0.75, -0.38, 0.0, 0.38, 0.75, 1.10}) {
        for (double duration : {0.60, 0.90, 1.20, 1.60, 2.00}) {
        double progress = 0.0;
        if (arcClear(start, yaw, v, w, duration, progress) && progress > best_progress) {
          best_v = v;
          best_w = w;
          best_progress = progress;
        }
        }
      }
    }
    ROS_WARN("Safe escape probe: v=%.2f w=%.2f progress=%.3f", best_v, best_w,
             best_progress);
    if (best_progress < min_distance_) {
      ROS_WARN("Safe escape: no footprint-clean escape arc");
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
                double w, double duration, double& progress) {
    const std::vector<geometry_msgs::Point> footprint = local_->getRobotFootprint();
    costmap_2d::Costmap2D* grid = local_->getCostmap();
    costmap_2d::Costmap2D::mutex_t* mutex = grid->getMutex();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*mutex);
    double x = pose.pose.position.x, y = pose.pose.position.y, theta = yaw;
    for (double t = 0.0; t <= duration; t += 0.04) {
      std::vector<geometry_msgs::Point> oriented;
      costmap_2d::transformFootprint(x, y, theta, footprint, oriented);
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
      for (const auto& cell : cells) {
        if (grid->getCost(cell.x, cell.y) >= costmap_2d::LETHAL_OBSTACLE)
          return false;
      }
      if (std::abs(w) < 1e-6) {
        x += v * 0.04 * std::cos(theta);
        y += v * 0.04 * std::sin(theta);
      } else {
        const double next = theta + w * 0.04;
        x += v / w * (std::sin(next) - std::sin(theta));
        y -= v / w * (std::cos(next) - std::cos(theta));
        theta = next;
      }
    }
    progress = std::hypot(x - pose.pose.position.x, y - pose.pose.position.y);
    return true;
  }

  void stop() {
    ros::WallRate rate(20);
    for (int i = 0; i < 5 && ros::ok(); ++i) {
      velocity_.publish(geometry_msgs::Twist());
      rate.sleep();
    }
  }

  costmap_2d::Costmap2DROS* local_;
  ros::Publisher velocity_;
  bool initialized_;
  double max_distance_, min_distance_, safety_margin_, speed_;
};

}  // namespace safe_escape_recovery

PLUGINLIB_EXPORT_CLASS(safe_escape_recovery::SafeEscapeRecovery, nav_core::RecoveryBehavior)
