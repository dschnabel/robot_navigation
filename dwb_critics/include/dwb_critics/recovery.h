#ifndef DWB_CRITICS_RECOVERY_H_
#define DWB_CRITICS_RECOVERY_H_

#include <dwb_local_planner/trajectory_critic.h>
#include <string>
#include <vector>

namespace dwb_critics
{

class RecoveryCritic : public dwb_local_planner::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan) override;
  double getScale() const override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
  void debrief(const nav_2d_msgs::Twist2D& cmd_vel) override;

protected:
  int stuck_counter_;
  bool stuck_;
  geometry_msgs::Pose2D stuck_pose_;
  double chosen_x_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS_RECOVERY_H_
