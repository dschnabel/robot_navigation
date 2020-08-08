#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from locomotor_msgs.msg import NavigateToPoseAction, NavigateToPoseGoal

client = actionlib.SimpleActionClient('/locomotor/navigate', NavigateToPoseAction)

def print_feedback(feedback):
    pose = feedback.state.global_pose.pose
    vel = feedback.state.current_velocity.velocity
    print('%.2f %.2f %.2f | %.2f %.2f' % (pose.x, pose.y, pose.theta,
                                          vel.x, vel.theta))
    print('Global plan: %d poses' % len(feedback.state.global_plan.poses))
    print('%.2f %.2f %.2f' % (feedback.percent_complete,
                              feedback.distance_traveled,
                              feedback.estimated_distance_remaining))

def run(x=0.0, y=0.0, theta=0.0, frame_id='map'):
    print(frame_id)
    goal = NavigateToPoseGoal()
    goal.goal.pose.x = x
    goal.goal.pose.y = y
    goal.goal.pose.theta = theta
    goal.goal.header.frame_id = frame_id
    client.send_goal(goal) #, feedback_cb = print_feedback)

def callback(data):
#    rospy.loginfo("x=%f,y=%f,theta=%f", data.pose.position.x, data.pose.position.y, data.pose.orientation.z)
    run(x=data.pose.position.x, y=data.pose.position.y, theta=data.pose.orientation.z, frame_id=data.header.frame_id)

def listener():
    rospy.init_node('goalListener', anonymous=True)

    client.wait_for_server()
    print('goalListener.py: Goal action server started!')

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
