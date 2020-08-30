#ifndef DWB_CRITICS_ROTATE_TO_START_H_
#define DWB_CRITICS_ROTATE_TO_START_H_

#include <dwb_local_planner/trajectory_critic.h>
#include <string>
#include <vector>

namespace dwb_critics
{

class RotateToStartCritic : public dwb_local_planner::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan) override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;

protected:
  bool startRotationReached_;
  double start_yaw_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS_ROTATE_TO_START_H_
