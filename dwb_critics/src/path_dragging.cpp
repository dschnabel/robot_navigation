#include <dwb_critics/path_dragging.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Path.h>
#include <nav_2d_utils/conversions.h>
#include <nav_2d_utils/path_ops.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathDraggingCritic, dwb_local_planner::TrajectoryCritic)

using namespace std;

#define MODE_FULL_SPEED 0
#define MODE_SHARP_TURN 1
#define MODE_AIM_GOAL   2

namespace dwb_critics
{

void PathDraggingCritic::onInit()
{
#ifdef PATH_DRAG_VIZ
    debug_pub_ = planner_nh_.advertise<nav_msgs::Path>("path_drag", 1);
#endif
}

bool PathDraggingCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                             const geometry_msgs::Pose2D& goal,
                             const nav_2d_msgs::Path2D& global_plan,
                             const size_t closest_index)
{
    const nav_core2::Costmap& costmap = *costmap_;
    const nav_grid::NavGridInfo& info = costmap.getInfo();
    nav_2d_msgs::Path2D adjusted_gp = nav_2d_utils::adjustPlanResolution(global_plan, info.resolution);

    double firstAngle;
    size_t step = 3;
    double chosenX = 0;
    double chosenY = 0;
    int mode = MODE_FULL_SPEED;
    auto &p1 = pose;

    for (size_t i = step; i <= (step * 4); i += step) {
        size_t forward_index = closest_index + i;
        if (forward_index > adjusted_gp.poses.size()-1) {
            chosenX = goal.x;
            chosenY = goal.y;
            mode = MODE_AIM_GOAL;
            break;
        }

        auto &p2 = adjusted_gp.poses[forward_index];
        double x = p2.x - p1.x;
        double y = p2.y - p1.y;
        double angle = atan2(y, x);

        if (i != step) {
            double diff = fabs(angles::shortest_angular_distance(angle, firstAngle));
            if (diff > 0.1) {
//                printf("sharp angle (index=%d)!\n", i);
                mode = MODE_SHARP_TURN;
                break;
            }
        } else {
            firstAngle = angle;
        }

        chosenX = x;
        chosenY = y;
    }

    if (mode == MODE_FULL_SPEED) {
        double magnitude_inv = 1.0 / sqrt(pow(chosenX, 2.0) + pow(chosenY, 2.0));
        double x_norm = magnitude_inv * chosenX;
        double y_norm = magnitude_inv * chosenY;

        chosenX = p1.x + (x_norm * 2);
        chosenY = p1.y + (y_norm * 2);
    } else if (mode == MODE_SHARP_TURN) {
        chosenX += p1.x;
        chosenY += p1.y;
    }

    forwardX_ = chosenX;
    forwardY_ = chosenY;

#ifdef PATH_DRAG_VIZ
    geometry_msgs::Pose2D pose2;
    pose2.x = forwardX_;
    pose2.y = forwardY_;
    nav_2d_msgs::Path2D line;
    line.poses.push_back(pose);
    line.poses.push_back(pose2);
    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    debug_pub_.publish(nav_2d_utils::pathToPath(line));
#endif

    return true;
}

double PathDraggingCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
    size_t i = traj.poses.size() - 1;
    return pow(forwardX_ - traj.poses[i].x, 2.0) + pow(forwardY_ - traj.poses[i].y, 2.0);
}

} /* namespace dwb_critics */
