import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
from tf2_ros import TransformException, TransformBroadcaster
from geometry_msgs.msg import PoseArray, TransformStamped
import numpy as np
import tf2_geometry_msgs
import time
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
import math
from tf2_ros.buffer import Buffer
import array
from tf2_ros.transform_listener import TransformListener
import tf_transformations


class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.cam_view = self.create_subscription(PoseArray,"/aruco_poses",self.cam_pose,10)
        self.pub_timer = self.create_timer(0.5, self.timers_cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        self.four_ele_num = [0,1,2,3]
        self.cnt = 0
        self.thred = 0.1
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 4
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)
        self.initial_guesses[0][0] = np.deg2rad(30)
        self.initial_guesses[3][0] = np.deg2rad(30)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')

        self.machine_state = "INIT"

        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 150
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure*(self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))
        self.br = TransformBroadcaster(self)
        
        pass
    
    def cam_pose(self,msg):
        # now = rclpy.time.Time()
        # trans = self.tf_buffer.lookup_transform( 
        #     'camera_link',
        #     'px100/base_link',
        #     now)
        # print(msg)
        # kk = msg.header.orientation
        # print(kk)
        # self.four_ele_num = [kk.x,kk.y,kk.z,kk.w]
        # print("+++++++++++++++++")
        m_marker = PoseArray()
        m_marker = msg
        # print(m_marker)
        cam2maker = TransformStamped()
        cam2maker.header.stamp = self.get_clock().now().to_msg()
        cam2maker.header.frame_id = 'camera_color_optical_frame'
        cam2maker.child_frame_id = 'marker'
        cam2maker.transform.translation.x = m_marker.poses[0].position.x
        cam2maker.transform.translation.y = m_marker.poses[0].position.y
        cam2maker.transform.translation.z = m_marker.poses[0].position.z
        temp_rotation_matrix = np.array([
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]])
        # 将旋转矩阵转四元数
        quat = tf_transformations.quaternion_from_matrix(temp_rotation_matrix)
        q1 = array.array('f', [0, 0, 0, 0])
        q1[0] = m_marker.poses[0].orientation.x
        q1[1] = m_marker.poses[0].orientation.y
        q1[2] = m_marker.poses[0].orientation.z
        q1[3] = m_marker.poses[0].orientation.w
        q2 = array.array('f', [0, 0, 0, 0])
        q2[0] = quat[0]
        q2[1] = quat[1]
        q2[2] = quat[2]
        q2[3] = quat[3]
        # 将两个四元数的旋转叠加
        q = tf_transformations.quaternion_multiply(q1, q2)
        cam2maker.transform.rotation.x = q[0]
        cam2maker.transform.rotation.y = q[1]
        cam2maker.transform.rotation.z = q[2]
        cam2maker.transform.rotation.w = q[3]

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
            T_sd = np.array([
                [rotation_matrix[0][0], rotation_matrix[0]
                    [1], rotation_matrix[0][2], x],
                [rotation_matrix[1][0], rotation_matrix[1]
                    [1], rotation_matrix[1][2], y],
                [rotation_matrix[2][0], rotation_matrix[2]
                    [1], rotation_matrix[2][2], z],
                [0.0, 0.0, 0.0, 1.0]])
            mlist, mflag = self.matrix_control(T_sd)
            self.fk = self.joint_to_pose(mlist)
            # print('T_sd', T_sd)
            # print('mflag:', mflag)
            # print('fk', self.fk)
            # print('mlist:', mlist)
        pass




    def js_cb(self, msg):
        # print(f"js_cb 的 msg为{msg.name}")
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i]) # 添加新的关节信息

    # demo
    def timers_cb(self):
        home_pose_arm = [np.deg2rad(+90.0),np.deg2rad(-35.0),np.deg2rad(60.0),np.deg2rad(50.0)]
        self.set_group_pos(home_pose_arm)
        self.set_single_pos("gripper",1.5)
        # if len(self.joint_pos) == 7:
        #     print(self.machine_state)
        #     match self.machine_state:
        #         case "INIT":
        #             if self.go_home_pos() == True and self.release():
        #                 print('go home pos done!')
        #                 self.machine_state = "NEXT1"
        #                 time.sleep(3.0)
        #         case "NEXT1":
        #             if self.go_sleep_pos() == True :
        #                 print('go sleep pos done!')
        #                 T_fk = self.joint_to_pose([-1.4, -0.35, 0.7, 1.0])
        #                 print('fk', T_fk)
        #                 self.machine_state = "NEXT2"
        #                 time.sleep(3.0)
        #         case "NEXT2":
        #             if self.set_group_pos([1.0, 0.0, 0.5, -0.6]) == True and self.grasp(-0.5):
        #                 print('NEXT2 control done!')
        #                 self.machine_state = "NEXT3"
        #                 time.sleep(1.0)
        #             pass
        #         case "NEXT3":
        #             T_sd = np.array([
        #                             [0.03, 0.99, 0.17, 0.02],
        #                             [-0.22, 0.17, -0.96, -0.1],
        #                             [-0.98,  0.0, 0.22, 0.05],
        #                             [ 0.0, 0.0, 0.0, 1.0]])
        #             mlist, mflag = self.matrix_control(T_sd)
        #             self.fk = self.joint_to_pose(mlist)
        #             print('fk', self.fk)
        #             print('mlist:', mlist)
        #             if mflag == True and self.release():
        #                 print('matrix control done!')
        #                 self.machine_state = "NEXT4"
        #                 time.sleep(3.0)
        pass

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
        print("duty call!!!!!!!!!!!!!!!!!!!!!")
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)

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
                        cal_name = 'gripper'
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
     
            thred = self.thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    print('current group pos:', check_pos)
                    print('target group pos:', pos_list)
                    if np.abs(pos_list[0] - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
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
        else:
            initial_guesses = [custom_guess]
        # print(f"self.robot_des is {self.robot_des} +++++++++++++")
        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
            # theta_list, success = mr.IKinBody(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=0.5,
                ev=0.2,
            )
            solution_found = True
            print('success',success, solution_found)
            
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                print('success',success,len(theta_list))
                theta_list = self._wrap_theta_list(theta_list)
                print(len(theta_list))
                # solution_found = self._check_joint_limits(theta_list)
                solution_found = True
            else:
                solution_found = False
                print(theta_list)

            if solution_found:
                if execute:
                    joint_list = [theta_list[0],theta_list[1],theta_list[2], theta_list[3]]
                    self.set_group_pos(joint_list)
                    self.T_sb = T_sd
                    # 等待机械臂运动完成
                    # self.wait_for_arm_to_finish()
                return theta_list, True

        # self.core.get_logger().warn('No valid pose could be found. Will not execute')
        return theta_list, False
    # # 我写的 可以删
    # def wait_for_arm_to_finish(self):
    #     while self.robot.is_moving():
    #         time.sleep(2)

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



def main():
    rclpy.init(args=None)
    contoller = ArmController()
    rclpy.spin(contoller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
