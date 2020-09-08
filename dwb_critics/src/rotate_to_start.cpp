#include <dwb_critics/rotate_to_start.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToStartCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{

void RotateToStartCritic::onInit()
{
  reset();
}

void RotateToStartCritic::reset()
{
    startRotationReached_ = false;
    start_yaw_ = 0.0;
}

bool RotateToStartCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D& global_plan,
                                 const size_t closest_index)
{
    if (!startRotationReached_) {
        if (start_yaw_ == 0.0) {
            int index = global_plan.poses.size() < 10 ? global_plan.poses.size() : 10;
            auto &p1 = global_plan.poses[0];
            auto &p2 = global_plan.poses[index];
            start_yaw_ = atan2(p2.y-p1.y, p2.x-p1.x);
        }

        if (fabs(angles::shortest_angular_distance(pose.theta, start_yaw_)) < 0.2) {
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
  return fabs(angles::shortest_angular_distance(end_yaw, start_yaw_));
}

} /* namespace dwb_critics */
