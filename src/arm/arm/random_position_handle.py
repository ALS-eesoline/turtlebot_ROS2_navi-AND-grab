import rclpy
import sensor_msgs
import threading
import random
from irobot_create_msgs.msg import HazardDetectionVector
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

# 新添加 对角线碰撞 or 垂直碰撞
# 向前伸改进


class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.cam_view = self.create_subscription(PoseArray,"/aruco_poses",self.cam_pose,10)
        self.hazardDetector = self.create_subscription(HazardDetectionVector,"/hazard_detection",self.hazardDete_callback,qos)
        # self.pub_timer = self.create_timer(0.5, self.timers_cb)
        self.hazard = False
        self.start_grabbing = False
        self.finish_grabing = False
        
        self.go_B_without_grabbing = False
        self.grab_forward =True
        forward_joint_list_fan = []
        
        self.initial_x_2 = 0
        self.initial_x_2 = 0
        self.state_flag = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        self.pantil_deg_cmd = PanTiltCmdDeg()
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg,"/pan_tilt_cmd_deg",10)
        self.marker_target_pose = [0,0,0,0]
        self.four_ele_num = [0,1,2,3]
        self.theta0 = 0
        self.theta1 = 0
        self.cnt = 0
        self.current_theta = 0
        self.current_x = 0
        self.current_y = 0
        self.thred = 0.09
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 4
        self.initial_guesses[1][0] = np.deg2rad(0.0)
        self.initial_guesses[2][0] = np.deg2rad(0.0)
        self.initial_guesses[3][0] = np.deg2rad(0.0)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')
        self.T_final = None
        self.machine_state = "INIT"
        self.sub=self.create_subscription(Odometry,"/odom",self.odom_callback,qos)
        self.twi_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 150
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure*(self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))
        self.br = TransformBroadcaster(self)
        self.target_T_sd = [0.0]*25
        self.T_buffer = None
        self.Gohome = False
        self.B_POINT_release = False
        self.ifShake = False
        self.IFarm_forward = False
        self.skakeCounter = 0
        self.catch_xyz = [0.0]*3
        self.timer_ = self.create_timer(0.5, self.timer_cb)
        self.timer_ = self.create_timer(0.5, self.release_b)
        self.time_flag = 0
        self.time_STAMP = True
        
    def hazardDete_callback(self,msg):
        try:
            if msg.detections[0].type == 1:
                self.hazard = True
        except Exception as e:
            pass
    
    def release_b(self):
        release_pos = [np.deg2rad(-90.0),np.deg2rad(20.),np.deg2rad(30.0),np.deg2rad(-20.0)]
        check_pos = copy.deepcopy(self.joint_pos)
        
        if self.B_POINT_release:
            self.set_group_pos(release_pos)
            if np.abs(check_pos[0]-release_pos[0])<0.1 and np.abs(check_pos[1]-release_pos[1])<0.1 and np.abs(check_pos[2]-release_pos[2])<0.1 and np.abs(check_pos[3]-release_pos[3])<0.1:
                self.set_single_pos("gripper",1.5)
                self.B_POINT_release = False

    def odom_callback(self,msg):
        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        # print(msg)
        self.current_theta = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    
    def timer_cb(self):
        
        self.pantil_deg_cmd.pitch = 12.0
        self.pantil_deg_cmd.yaw = 0.0
        self.pantil_deg_cmd.speed = 10
        self.pantil_pub.publish(self.pantil_deg_cmd)
        thred = self.thred
        check_pos = copy.deepcopy(self.joint_pos)
        
        # 根据start_grabbing true and false, 判断 开始 抓取
        if not self.start_grabbing:
            return 0
        
        if self.cnt%5 == 0:
            # 识别
            print("state 1111")
            try:
                ave_x = 0
                ave_y = 0
                ave_z = 0
                
                sample_list = copy.deepcopy(self.target_T_sd)
                
                for obj in sample_list:
                    
                    ave_x += obj[0][3]   
                    ave_y += obj[1][3]   
                    ave_z += obj[2][3]
              
                
                ave_x = ave_x/25
                ave_y = ave_y/25
                ave_z = ave_z/25
                
                self.catch_xyz[0] = ave_x+0.015
                self.catch_xyz[1] = ave_y
                self.catch_xyz[2] = ave_z+0.015
                
                self.cnt+=1
                
            except Exception as e:
                print(e)
                pass
        elif self.cnt%5 == 1:
            print("state 2222") 
            print(self.catch_xyz) 
            # if not self.IFarm_forward:
            #     initial_pose_arm = [0.0,np.deg2rad(0.0),np.deg2rad(-70.0),np.deg2rad(35.0)]
            #     self.set_group_pos(initial_pose_arm)
            #     self.set_single_pos('gripper',1.5)
            #     if np.abs(0.0 - check_pos[0]) < thred and np.abs(initial_pose_arm[1] - check_pos[1]) < thred and np.abs(initial_pose_arm[2] - check_pos[2]) < thred*10 and np.abs(initial_pose_arm[3] - check_pos[3]) < thred:            
            #         self.IFarm_forward = True
            
            
            # else:
            initial_pose_arm = [check_pos[0],np.deg2rad(-5.0),np.deg2rad(-90.0),np.deg2rad(0.0)]
            self.set_group_pos(initial_pose_arm)
            self.set_single_pos('gripper',1.5)
            if np.abs(check_pos[0] - check_pos[0]) < thred and np.abs(initial_pose_arm[1] - check_pos[1]) < thred and np.abs(initial_pose_arm[2] - check_pos[2]) < thred*10 and np.abs(initial_pose_arm[3] - check_pos[3]) < thred:            
                self.set_group_pos(initial_pose_arm)
                self.cnt+=1
            else:
                pass
            
            
            
            
        elif self.cnt%5 == 2:
            print("state forward ready")
            initial_pose_arm = [0.0,np.deg2rad(-15.0),np.deg2rad(-30.0),np.deg2rad(40.0)]
            self.set_group_pos(initial_pose_arm)
            self.set_single_pos('gripper',1.5)
            if np.abs(0.0 - check_pos[0]) < thred and np.abs(initial_pose_arm[1] - check_pos[1]) < thred and np.abs(initial_pose_arm[2] - check_pos[2]) < thred*10 and np.abs(initial_pose_arm[3] - check_pos[3]) < thred:            
                self.cnt+=1
            
            
        elif self.cnt%5 == 3:
            print("state 3333") 
            m_x = self.catch_xyz[0]
            m_y = self.catch_xyz[1] 
            m_z = self.catch_xyz[2]
            if self.time_STAMP:
                self.time_flag = time.time()
                self.time_STAMP = False   
            
            # self.get_grab_forward_joint_list(m_x ,m_y,m_z)
            
            
            
            m_theta = np.arctan2(m_y, m_x)
            
            m_x = self.catch_xyz[0]
            m_y = self.catch_xyz[1]
            m_z = self.catch_xyz[2] 
            new_T_sd = [
                            [np.cos(m_theta), -np.sin(m_theta), 0.0, m_x],
                            [np.sin(m_theta), np.cos(m_theta), 0.0, m_y],
                            [0.0, 0.0, 1.0, m_z],
                            [0.0, 0.0, 0.0, 1.0]
                        ]
            state,isSolvable = self.matrix_control(new_T_sd)
            
            for i in range(4):
                if np.abs(state[i]) > 4:
                    state[i] = state[i] - 3.1415926*2*np.sign(state[i])
            
             
            if not isSolvable:
                self.target_T_sd = [0.0]*25
                self.cnt = 0
                pose_arm = [np.deg2rad(-90.0),np.deg2rad(-35.0),np.deg2rad(60.0),np.deg2rad(50.0)]
                self.set_group_pos(pose_arm)
                return 0
            
            now_time = time.time()
            delta_time = now_time - self.time_flag
            whether = (delta_time > 30)
            self.set_single_pos("waist",state[0])
            print(f"waist moving {m_x,m_y}")
            if np.abs(check_pos[0]-state[0]) < 0.02:
                # joint_list = [state[0],state[1],state[2], state[3]]  
                # self.set_group_pos(joint_list)
                # self.set_single_pos("wrist_angle",state[3])
                # if np.abs(check_pos[3]-state[3]) < 0.10:
                joint_list = [state[0],state[1],state[2], state[3]]  
                self.set_group_pos(joint_list)
                if ((np.abs(state[1] - check_pos[1]) < thred and np.abs(state[2] - check_pos[2]) < thred and np.abs(state[3] - check_pos[3]) < thred)) or whether:
                    self.set_single_pos('gripper',0.65)
                    self.cnt+=1
                
            
                
            


        else:
            print("state 4444") 
            self.initial_guesses[1][0] = np.deg2rad(0)
            # home_pose_arm = [np.deg2rad(-90.0),np.deg2rad(-5.0),np.deg2rad(50.0),np.deg2rad(50.0)]
            initial_pose_arm = [0.0,np.deg2rad(0.0),np.deg2rad(-75.0),np.deg2rad(0.0)]
            home_pose_arm = [0.0,np.deg2rad(-10.0),np.deg2rad(-55.0),np.deg2rad(0.0)]
            self.set_group_pos(home_pose_arm)
            # if np.abs(home_pose_arm[0] - check_pos[0]) < thred and np.abs(home_pose_arm[1] - check_pos[1]) < thred and np.abs(home_pose_arm[2] - check_pos[2]) < thred and np.abs(home_pose_arm[3] - check_pos[3]) < thred:
                # self.cnt+=1
            print("good try")
            self.Gohome = True
            self.target_T_sd = [0.0]*25
            # self.start_grabbing = False
            # self.set_single_pos('gripper',1.5)
            
            # mssg = Twist()
            # mssg.linear.x = (-2.0)
            # self.twi_pub.publish(mssg)
            
            if self.Gohome:
                # home_pose_arm = [np.deg2rad(-90.0),np.deg2rad(-35.0),np.deg2rad(30.0),np.deg2rad(35.0)]
                
                self.set_group_pos(home_pose_arm)
                # if np.abs(home_pose_arm[0] - check_pos[0]) < 2*thred and np.abs(home_pose_arm[1] - check_pos[1]) < 2*thred and np.abs(home_pose_arm[2] - check_pos[2]) < 2*thred and np.abs(home_pose_arm[3] - check_pos[3]) < 2*thred:
                self.cnt += 1 
                self.Gohome = False
                self.finish_grabing = True
                self.start_grabbing = False
                
                    
    def get_grab_forward_joint_list(self,x,y,z):
        if self.grab_forward:
            print(f"////{x}//////{y}////////{z}///")
            
            
            m_theta = np.arctan2(y,x)
            k = np.tan(m_theta)
            m_x = x - 0.015
            m_y = y - k*0.015
            m_z = z
            
            solvement = []
            
            
            for i in range(10):
                new_T_sd = [
                            [np.cos(m_theta), -np.sin(m_theta), 0.0, m_x],
                            [np.sin(m_theta), np.cos(m_theta), 0.0, m_y],
                            [0.0, 0.0, 1.0, m_z],
                            [0.0, 0.0, 0.0, 1.0]
                        ]
                joint_lis, state_tf = self.matrix_control(new_T_sd)
                if state_tf == False:
                    self.go_B_without_grabbing = True
                    solvement.append([0.0,0.0,-1.5,0.0])
                else:
                    for i in range(4):
                        if np.abs(joint_lis[i]) > 4:
                            joint_lis[i] = joint_lis[i] - 3.1415926*2*np.sign(joint_lis[i])
                    
                    kl = [joint_lis[0],joint_lis[1],joint_lis[2],joint_lis[3]]
                    solvement.append(kl)
                    m_x += 0.003
                    m_y += 0.003*k
            self.forward_joint_list_fan = solvement
            self.grab_forward = False
            
                
            
            
            
                
                 
        
    def cam_pose(self,msg):
        print("------*-*-*-*-*-*-**-*-*---------")
        m_marker = PoseArray()
        m_marker = msg
        cam2maker = TransformStamped()
        cam2maker.header.stamp = self.get_clock().now().to_msg()
        cam2maker.header.frame_id = 'camera_color_optical_frame'
        cam2maker.child_frame_id = 'marker'
        cam2maker.transform.translation.x = m_marker.poses[0].position.x
        cam2maker.transform.translation.y = m_marker.poses[0].position.y
        cam2maker.transform.translation.z = m_marker.poses[0].position.z
        q1 = array.array('f', [0, 0, 0, 0])
        q1[0] = m_marker.poses[0].orientation.x
        q1[1] = m_marker.poses[0].orientation.y
        q1[2] = m_marker.poses[0].orientation.z
        q1[3] = m_marker.poses[0].orientation.w
        cam2maker.transform.rotation.x = q1[0]
        cam2maker.transform.rotation.y = q1[1]
        cam2maker.transform.rotation.z = q1[2]
        cam2maker.transform.rotation.w = q1[3]

        self.br.sendTransform(cam2maker)
        # print(self.marker2arm())
        if self.marker2arm() is not None:
            trans = self.marker2arm()
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            rotation_matrix = tf_transformations.quaternion_matrix([trans.transform.rotation.x,
                                                                   trans.transform.rotation.y,
                                                                   trans.transform.rotation.z,
                                                                   trans.transform.rotation.w])
            T_sd = [
                [rotation_matrix[0][0], rotation_matrix[0]
                    [1], rotation_matrix[0][2], x],
                [rotation_matrix[1][0], rotation_matrix[1]
                    [1], rotation_matrix[1][2], y],
                [rotation_matrix[2][0], rotation_matrix[2]
                    [1], rotation_matrix[2][2], z],
                [0.0, 0.0, 0.0, 1.0]]
            
            
            # self.target_T_sd = T_sd
            # print(self.target_T_sd)
            self.target_T_sd.insert(0, T_sd)  # 在列表的开始位置插入新元素
            self.target_T_sd.pop()
            
            
    def js_cb(self, msg):
        # print(f"js_cb 的 msg为{msg}")
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i]) # 添加新的关节信息

    

    def marker2arm(self):
        '''
        need to write
        :return:  transform matrix between camera and arm
        '''
        to_frame_rel = 'px100/base_link'
        from_frame_rel = 'marker'
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        return trans


    def cam2arm(self):
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
        '''
        need to write
        :return:  transform matrix between camera and arm
        '''
        return None

    def cam_pos_to_arm_pos(self):
        '''
        need to write
        :return:  position from camera coordinate to arm coordinate
        '''
        return None

    def set_single_pos(self, name, pos, blocking=True):
        '''
        ### @param: name: joint name
        ### @param: pos: radian
        ### @param: blocking - whether the arm need to check current position 

        '''
        # print("duty call!!!!!!!!!!!!!!!!!!!!!")
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)
        print('setpos:',pos)

        thred = self.thred
        if blocking:
            check_pos = None
            cal_name = None
            if len(self.joint_pos) == 7:
                match name:
                    case "waist":
                        check_pos = self.joint_pos[0]
                        cal_name = 'joint'
                    case "shoulder":
                        check_pos = self.joint_pos[1]
                        cal_name = 'joint'
                    case "elbow":
                        check_pos = self.joint_pos[2]
                        cal_name = 'joint'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]
                        cal_name = 'joint'
                    case "gripper":
                        check_pos = self.joint_pos[4]
                        print('gripper pos:', check_pos)
                        # cal_name = 'gripper'
                        cal_name = 'joint'
                    case _:
                        print('unvalid name input!')

                match cal_name:
                    case "joint":
                        dis = np.abs(pos-check_pos)
                        if dis < thred:
                            return True
                        else:
                            print('single joint moving...')
                            return False                       
                    case "gripper":
                        return True

        pass

    def set_group_pos(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
        else:
            self.arm_group_command.name = "arm"
            self.arm_group_command.cmd = pos_list
            self.group_pub.publish(self.arm_group_command)
            # print(f"pos_list 是 ++++++ {pos_list}")
            thred = self.thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    print('current group pos:', check_pos)
                    print('target group pos:', pos_list)
                    if np.abs(pos_list[0] - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                        # self.gripper_controller(-0.6)
                        # self.arm_command.name = "gripper"
                        # self.arm_command.cmd = -0.6 #-0.6--0.7---1.5
                        # self.cmd_pub.publish(self.arm_command)
                        return True
                    else:
                        if np.abs(pos_list[0] - check_pos[0]) >= thred:
                            print('waist moving...')
                        if np.abs(pos_list[1] - check_pos[1]) >= thred:
                            print('shoulder moving...')
                        if np.abs(pos_list[2] - check_pos[2]) >= thred:
                            print('elbow moving...')
                        if np.abs(pos_list[3] - check_pos[3]) >= thred:
                            print('wrist moving...')
                            return False            
            pass

    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def go_home_pos(self):
        state = self.set_group_pos([0.0, 0.0, 0.0, 0.0])
        return state

    def go_sleep_pos(self):
        state = self.set_group_pos([-1.4, -0.35, 0.7, 1.0])
        return state


    def matrix_control(self, T_sd, custom_guess: list[float]=None, execute: bool=True):
        if custom_guess is None:
            initial_guesses = self.initial_guesses
            print("use self guess")
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
            # theta_list, success = mr.IKinBody(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=0.05,
                ev=0.02,
            )
            solution_found = True
            print('success',success, solution_found)
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                theta_list = self._wrap_theta_list(theta_list)
                # solution_found = self._check_joint_limits(theta_list)
                # print(f"=========={theta_list}==========")
                solution_found = True
            else:
                solution_found = False

            if solution_found:
                if execute:
                    joint_list = [theta_list[0],theta_list[1],theta_list[2], theta_list[3]]
                    # self.set_group_pos(joint_list)
                    self.T_sb = T_sd
                    # 等待机械臂运动完成
                    # self.wait_for_arm_to_finish()
                return theta_list, success

        # self.core.get_logger().warn('No valid pose could be found. Will not execute')
        return theta_list, False
    

    def waist_control(self, pos):
        """
        lower limit = -1.5
        upper limit = 1.5
        """
        pos = float(pos)
        state = self.set_single_pos('waist', pos)
        return state
    
    def shoulder_control(self, pos):
        """
        lower limit = -0.4
        upper limit = ~0.9
        """
        pos = float(pos)
        state = self.set_single_pos('shoulder', pos)
        return state
    
    def elbow_control(self, pos):
        '''
        lower limit = -1.1
        upper limit = 0.8
        '''
        pos = float(pos)
        state = self.set_single_pos('elbow', pos)
        return state
    
    def wrist_control(self, pos):
        '''
        lower limit = -1.4
        upper limit = 1.8
        '''
        pos = float(pos)
        state = self.set_single_pos('wrist_angle', pos)
        return state


    def gripper_controller(self, effort, delay: float):
        '''
        effort: release = 1.5
        effort: grasp = -0.6
        '''
        name = 'gripper'
        effort = float(effort)
        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state


    def set_pressure(self, pressure: float) -> None:
        """
        Set the amount of pressure that the gripper should use when grasping an object.
        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def release(self, delay: float = 1.0) -> None:
        """
        Open the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(1.5, delay)
        return state

    def grasp(self, pressure: float = 0.5, delay: float = 1.0) -> None:
        """
        Close the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(pressure, delay)
        return state


    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        """
        Wrap an array of joint commands to [-pi, pi) and between the joint limits.

        :param theta_list: array of floats to wrap
        :return: array of floats wrapped between [-pi, pi)
        """
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] += REV
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] -= REV
        return theta_list

def spinTHREAD(node):
    rclpy.spin(node)

def go_dest_by_navigation(x,y,z,w,navi):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navi.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.z = z
    goal_pose.pose.orientation.w = w 
    
    # goal_pose.pose.position.x = 0.27
    # goal_pose.pose.position.y = -0.38
    # goal_pose.pose.orientation.z =0.707
    # goal_pose.pose.orientation.w =-0.707 

    navi.goToPose(goal_pose)

    i = 0
    while not navi.isTaskComplete():
        i = i + 1
        feedback = navi.getFeedback()
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
                navi.cancelTask()
    result = navi.getResult()
    if result == TaskResult.SUCCEEDED:
        print('S Point succeeded!')
    elif result == TaskResult.CANCELED:
        print('S Point was canceled!')
    elif result == TaskResult.FAILED:
        print('S Point failed!')
    else:
        print('Goal has an invalid return status!')
        
    return result    
    
def move2Face(x, y, controller,initial_theta,initial_x,initial_y):
    msg = Twist()
    
    controller.twi_pub.publish(msg)


def main():
    rclpy.init(args=None)
    contoller = ArmController()
    # rclpy.spin(contoller)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(contoller)

    # 在另一个线程中执行旋转
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    
    
    
    face_y = math.sqrt(2)/2
    # spin_thread.join()
    
    # navigation backbone 
    print("start navigation threadd")
    navigator = BasicNavigator()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.27
    initial_pose.pose.position.y = -0.38
    initial_pose.pose.orientation.z = -0.707
    initial_pose.pose.orientation.w = 0.707
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    navigator.changeMap('/home/tony/ros2_ws/src/map.yaml')
    
    home_pose_arm = [np.deg2rad(-90.0),np.deg2rad(-35.0),np.deg2rad(60.0),np.deg2rad(50.0)]
    contoller.set_group_pos(home_pose_arm)
    

    # go to near A position
    # a_result = go_dest_by_navigation(0.6, -1.5, -face_y, face_y,navigator)
    a_result = go_dest_by_navigation(0.20, -3.2, -face_y, face_y,navigator)

    # a_result = TaskResult.SUCCEEDED
    if a_result == TaskResult.SUCCEEDED:
        # print(f"到达A点,A is ({navigator.feedback.current_pose.pose.position.x,navigator.feedback.current_pose.pose.position.y})")
        modify_y = -3.0
        fail_counter = 0
        go_face_falg = True
        tar_x = 0
        tar_y = 0
        init_flag = True  #旋转找到目标
        initial_theta = 80
        
        
         
     # + 90 ~ -90; - -90 ~ 90
        
        

        while init_flag:
            msg = Twist()
            msg.angular.z = 1.5
            contoller.twi_pub.publish(msg)
            time.sleep(0.3)
            contoller.target_T_sd = [0.0]*25
            time.sleep(2)
            print("spiiiiiiiiiiiiiiiiiiiiiiiinnnn")
            if contoller.target_T_sd[7] != 0.0:
                init_flag = False
                time.sleep(0.1)
        
        while True:
            time.sleep(0.2)
            goal = copy.deepcopy(contoller.target_T_sd[0][1][3])
            mssg = Twist()
            mssg.angular.z = 0.1*np.sign(goal)
            contoller.twi_pub.publish(mssg)
            
            print(f"........{mssg.angular.z}....... {goal}")
            time.sleep(1)
            goal = copy.deepcopy(contoller.target_T_sd[0][1][3])
            if np.abs(goal) < 0.05:
                break
            
                
        while not contoller.hazard:
            print("ready to go")
            msssg = Twist()
            msssg.linear.x = 0.3
            contoller.twi_pub.publish(msssg)
            time.sleep(0.2)
            
        time.sleep(0.5)
        vector = copy.deepcopy(contoller.target_T_sd[0])
        
        sweep_flag_init = np.sign(vector[1][2])
        if_sweep = vector[1][2]
        how_to_sweep = np.arctan2(-1*vector[1][2], -1*vector[0][2])
        tar_pos_list = []
        if np.abs(how_to_sweep) < 0.5:
            tar_pos_list = [0.0,np.deg2rad(10.0),np.deg2rad(70.0),np.deg2rad(-80.0)]
            pass
        else:
            tar_pos_list = [0.0,np.deg2rad(15.0),np.deg2rad(60.0),np.deg2rad(-80.0)]
            pass
        
        # sweep_ready = [sweep_flag_init*1.5,np.deg2rad(-10.0),np.deg2rad(70.0),np.deg2rad(0.0)]
        # contoller.set_group_pos([sweep_ready[0],sweep_ready[1],sweep_ready[2],sweep_ready[3]])
        # time.sleep(2.5)
        
        # rise
        sweep_ready = [contoller.joint_pos[0],np.deg2rad(5.0),np.deg2rad(-90.0),np.deg2rad(0.0)]
        contoller.set_group_pos([sweep_ready[0],sweep_ready[1],sweep_ready[2],sweep_ready[3]])
        time.sleep(2.5)
        contoller.set_single_pos("waist",sweep_flag_init*1.5)
        time.sleep(2)
        sweep_ready = [sweep_flag_init*1.5,0.0,np.deg2rad(50.0),np.deg2rad(-20.0)]
        contoller.set_group_pos([sweep_ready[0],sweep_ready[1],sweep_ready[2],sweep_ready[3]])
        time.sleep(1)
        sweep_path = -1
        unit_sweep = 0.1
        while True:
            if np.abs(if_sweep) < 0.05:
                contoller.target_T_sd = [0.0] * 25
                time.sleep(2)
                break
            time.sleep(0.5)
            # sweep_flag = np.sign(contoller.target_T_sd[0][1][2])
            sweep_flag = sweep_flag_init
            sweep_path += 1
            waist_pose = sweep_flag*1.5 -sweep_flag*sweep_path*unit_sweep
            if np.abs(waist_pose) > 1.5:
                time.sleep(0.5)
                contoller.target_T_sd = [0.0] * 25
                time.sleep(2)
                break
            
            # tar_pos_list = [waist_pose,np.deg2rad(10.0),np.deg2rad(80.0),np.deg2rad(-80.0)]
            # tar_pos_list = [waist_pose,np.deg2rad(10.0),np.deg2rad(80.0),np.deg2rad(-80.0)]
            
            kk = [waist_pose,tar_pos_list[1],tar_pos_list[2],tar_pos_list[3]]
            contoller.set_group_pos(kk)
            
        contoller.start_grabbing = True
        while contoller.start_grabbing and (not contoller.go_B_without_grabbing):
            time.sleep(1)
            # kl = copy.deepcopy(contoller.forward_joint_list_fan)
            if not contoller.grab_forward:
                for obj in contoller.forward_joint_list_fan:
                    contoller.set_group_pos(obj)
                    time.sleep(0.5)
                contoller.cnt += 1
        time.sleep(1.5)
        
        home_pose_arm_kk = [contoller.joint_pos[0],np.deg2rad(5.0),np.deg2rad(-90.0),np.deg2rad(0.0)]
        contoller.set_group_pos(home_pose_arm_kk)        
        time.sleep(1)
        
        home_pose_arm_kk = [-1.5,np.deg2rad(-15.0),np.deg2rad(60.0),np.deg2rad(0.0)]
        contoller.set_group_pos(home_pose_arm_kk)
        time.sleep(1)
    go_dest_by_navigation(navigator.feedback.current_pose.pose.position.x,navigator.feedback.current_pose.pose.position.y,0.707,0.707,navi=navigator)
    # go_dest_by_navigation(navigator.feedback.current_pose.pose.position.x,-2.65,0.0,1.0,navi=navigator)
    
    
    
    result = go_dest_by_navigation(2.7,-3.15,0.0,1.0,navigator)  
              
    # result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('B Point succeeded!')
        contoller.B_POINT_release = True
        time.sleep(5)
    elif result == TaskResult.CANCELED:
        print('B Point was canceled!')
    elif result == TaskResult.FAILED:
        print('B Point failed!')
    else:
        print('Goal has an invalid return status!')
        
    contoller.go_sleep_pos()        
    go_dest_by_navigation(0.27,-0.38,0.707,0.707,navigator)          
    
    
    spin_thread.join()
    contoller.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# 矫正位置， 进行抓取
                # ave_x = 0
                # ave_y = 0
                # ave_z = 0
                # goal_matrix = copy.deepcopy(contoller.target_T_sd)
                # for obj in goal_matrix:
                #     ave_x += obj[0][3]                    
                #     ave_y += obj[1][3]                    
                #     ave_z += obj[2][3]                    
                # ave_x = ave_x / 10
                # ave_y = ave_y / 10
                # ave_z = ave_z / 10
                
                # isCatchable = np.abs(ave_x)