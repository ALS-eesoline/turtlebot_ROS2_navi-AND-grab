from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy

"""
Basic navigation demo to go to pose.
"""


class DIY_V(Node):

    def __init__(self, node_name='basic_navigator'):
        super().__init__(node_name=node_name)
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self.CURRENTPOSE,
                                                              amcl_pose_qos)
        
    def CURRENTPOSE(self,msg):
        print(f"+++++++++++{msg}++++++++++++++")

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
        # x = navigator.feedback.current_pose.pose.position.x
        # y = navigator.feedback.current_pose.pose.position.y
        # z = navigator.feedback.current_pose.pose.orientation.z
        # w = navigator.feedback.current_pose.pose.orientation.w
        # print(f"x is {x},y is {y}, z is {z}, w is {w}")

        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
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

            # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)

    # Do something depending on the return code
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal S succeeded!')
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

# Go to our demos A goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.20
    goal_pose.pose.position.y = -3.3
    goal_pose.pose.orientation.z =-0.707
    goal_pose.pose.orientation.w =0.707 

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        # x = navigator.feedback.current_pose.pose.position.x
        # y = navigator.feedback.current_pose.pose.position.y
        # z = navigator.feedback.current_pose.pose.orientation.z
        # w = navigator.feedback.current_pose.pose.orientation.w
        # print(f"x is {x},y is {y}, z is {z}, w is {w}")
        # print(f"当前位置为 {navigator.feedback.current_pose.pose.position}")
        # print(f"当前位置为 {navigator.feedback.current_pose.pose.orientation}")
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
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

            # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal A succeeded!')
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


# Go to our demos B goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.7
    goal_pose.pose.position.y = -3.2
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        x = navigator.feedback.current_pose.pose.position.x
        y = navigator.feedback.current_pose.pose.position.y
        z = navigator.feedback.current_pose.pose.orientation.z
        w = navigator.feedback.current_pose.pose.orientation.w
        print(f"x is {x},y is {y}, z is {z}, w is {w}")
    
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
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal BB succeeded!')
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

    # navigator.lifecycleShutdown()
    
    
# # Go back to our demos A goal pose
#     goal_pose = PoseStamped()
#     goal_pose.header.frame_id = 'map'
#     goal_pose.header.stamp = navigator.get_clock().now().to_msg()
#     goal_pose.pose.position.x = 0.32
#     goal_pose.pose.position.y = -3.0
#     goal_pose.pose.orientation.z =0.707
#     goal_pose.pose.orientation.w =0.707 

#     # sanity check a valid path exists
#     # path = navigator.getPath(initial_pose, goal_pose)

#     navigator.goToPose(goal_pose)

#     i = 0
#     while not navigator.isTaskComplete():
#         ################################################
#         #
#         # Implement some code here for your application!
#         #
#         ################################################

#         # Do something with the feedback
#         i = i + 1
#         feedback = navigator.getFeedback()
#         if feedback and i % 5 == 0:
#             print(
#                 'Estimated time of arrival: '
#                 + '{0:.0f}'.format(
#                     Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
#                     / 1e9
#                 )
#                 + ' seconds.'
#             )

#             # Some navigation timeout to demo cancellation
#             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
#                 navigator.cancelTask()

#             # Some navigation request change to demo preemption
#             # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
#             #     goal_pose.pose.position.x = -3.0
#             #     navigator.goToPose(goal_pose)

#     # Do something depending on the return code
#     result = navigator.getResult()
#     if result == TaskResult.SUCCEEDED:
#         print('Goal succeeded!')
#     elif result == TaskResult.CANCELED:
#         print('Goal was canceled!')
#     elif result == TaskResult.FAILED:
#         print('Goal failed!')
#     else:
#         print('Goal has an invalid return status!')
        
    
# Go back to our demos start goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.27
    goal_pose.pose.position.y = -0.38
    goal_pose.pose.orientation.z =0.707
    goal_pose.pose.orientation.w =0.707 

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        
    
        
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
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

            # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    exit(0)


if __name__ == '__main__':
    main()