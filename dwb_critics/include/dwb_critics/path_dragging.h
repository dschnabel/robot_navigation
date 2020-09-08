#ifndef DWB_CRITICS_PATH_DRAGGING_H_
#define DWB_CRITICS_PATH_DRAGGING_H_

#include <dwb_local_planner/trajectory_critic.h>
#include <string>

//#define PATH_DRAG_VIZ

namespace dwb_critics
{

class PathDraggingCritic: public dwb_local_planner::TrajectoryCritic
{
public:
    PathDraggingCritic() : forwardX_(0.0), forwardY_(0.0) {}
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan,
               const size_t closest_index = 0) override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
private:
  double forwardX_, forwardY_;
#ifdef PATH_DRAG_VIZ
  ros::Publisher debug_pub_;
#endif
};

} /* namespace dwb_critics */
#endif /* DWB_CRITICS_PATH_DRAGGING_H_ */
