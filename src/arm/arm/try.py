import math
# a = math.sqrt(2)/2
import rclpy
import sensor_msgs
import threading
import random
import copy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import PoseArray, TransformStamped
import numpy as np
import tf2_geometry_msgs
from pan_tilt_msgs.msg import PanTiltCmdDeg
import time
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
import math
from tf2_ros.buffer import Buffer
import array
from tf2_ros.transform_listener import TransformListener
import tf_transformations
# lib of navigtion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy

# class ArmController(Node):
#     def __init__(self):
#         super().__init__("ArmController")
#         qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
#         self.twi_pub=self.create_publisher(Twist,'/cmd_vel',10)
#         self.timer_ = self.create_timer(0.5, self.timer_cb)
        
#     def timer_cb(self):
#         print('aaaaaaa')
#         msg = Twist()
#         msg.linear.x = 0.5
#         msg.angular.z = -2.0
#         self.twi_pub.publish(msg)
        

# rclpy.init(args=None)
# contoller = ArmController()
# rclpy.spin(contoller)

a = [1,2,3]
b = []
b.append(a)
b.append(a)
print(time.time())
time.sleep(1)
print(12>5)
