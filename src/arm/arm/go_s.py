from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import numpy as np
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy

"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()
    # diy_v = DIY_V()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    navigator.changeMap('/home/tony/ros2_ws/src/map.yaml')

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.27
    goal_pose.pose.position.y = -0.38
    goal_pose.pose.orientation.z =-0.707
    goal_pose.pose.orientation.w =0.707 

   

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal S succeeded!')
        # home_pose_arm = [0.0,np.deg2rad(0.0),np.deg2rad(-75.0),np.deg2rad(0.0)]
        # group_pub = Node.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        # arm_group_command = JointGroupCommand()
        
        # arm_group_command.name = "arm"
        # arm_group_command.cmd = home_pose_arm
        # group_pub.publish(arm_group_command)
        x = navigator.feedback.current_pose.pose.position.x
        y = navigator.feedback.current_pose.pose.position.y
        z = navigator.feedback.current_pose.pose.orientation.z
        w = navigator.feedback.current_pose.pose.orientation.w
        print(f"x is {x},y is {y}, z is {z}, w is {w}")
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    exit(0)


if __name__ == '__main__':
    main()