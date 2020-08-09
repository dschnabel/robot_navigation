#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from locomotor_msgs.msg import NavigateToPoseAction, NavigateToPoseGoal
from tf.transformations import euler_from_quaternion

client = actionlib.SimpleActionClient('/locomotor/navigate', NavigateToPoseAction)

def run(x=0.0, y=0.0, o=0.0, frame_id='map'):
    print(frame_id)
    goal = NavigateToPoseGoal()
    goal.goal.pose.x = x
    goal.goal.pose.y = y
    goal.goal.pose.theta = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
    goal.goal.header.frame_id = frame_id
    client.send_goal(goal)

def callback(data):
    run(x=data.pose.position.x, y=data.pose.position.y, o=data.pose.orientation, frame_id=data.header.frame_id)

def listener():
    rospy.init_node('goalListener', anonymous=True)

    client.wait_for_server()
    print('goalListener.py: Goal action server started!')

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
