#include <dwb_critics/rotate_to_start.h>
#include <nav_2d_utils/parameters.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToStartCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

void RotateToStartCritic::onInit()
{
    xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(critic_nh_, "xy_goal_tolerance", 0.25);
    xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
    reset();
}

void RotateToStartCritic::reset()
{
    startRotationReached_ = false;
    goal_yaw_ = 0.0;
}

bool RotateToStartCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D& global_plan,
                                 const size_t closest_index)
{
    if (!startRotationReached_) {
        if (goal_yaw_ == 0.0) {
            int index = global_plan.poses.size() < 10 ? global_plan.poses.size() : 10;
            auto &p1 = global_plan.poses[0];
            auto &p2 = global_plan.poses[index];
            goal_yaw_ = atan2(p2.y-p1.y, p2.x-p1.x);
            if (isnan(goal_yaw_)) {
                goal_yaw_ = pose.theta;
            }
        }

        start_yaw_ = pose.theta;

        if (hypot_sq(pose.x - goal.x, pose.y - goal.y) <= xy_goal_tolerance_sq_
                || fabs(angles::shortest_angular_distance(pose.theta, goal_yaw_)) < 0.2) {
            startRotationReached_ = true;
        }
    }

    return true;
}

double RotateToStartCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  if (startRotationReached_) {
    return 0.0;
  }

  if (traj.velocity.x != 0.0) {
      return 100.0;
  }

  double end_yaw = traj.poses.back().theta;
  double score = fabs(angles::shortest_angular_distance(end_yaw, goal_yaw_));

  if (angles::shortest_angular_distance(start_yaw_, goal_yaw_) > 0) {
      if (traj.velocity.theta < 1.0) score += 100;
  } else {
      if (traj.velocity.theta > -1.0) score += 100;
  }

  return score;
}

} /* namespace dwb_critics */
