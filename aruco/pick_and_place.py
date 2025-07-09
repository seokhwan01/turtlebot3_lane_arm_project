#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Frame Capture and JPEG Compression

This script captures frames from a webcam using OpenCV,
retrieves the raw image's width and height, compresses
the image to JPEG format, and also determines the size
of the decoded (compressed) image.

Author: Rujin Kim
Date: 2025-05-17
"""
import rclpy
from rclpy.time import Time

from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import getkey
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist, Pose, PoseArray
from turtlebot_cosmo_interface.srv import MoveitControl
from aruco_yolo.moveit_client import TurtlebotArmClient
import time
import ast
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose_stamped
import tf2_ros
import math
from std_srvs.srv import Trigger
# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')
        #추가
        self.start_client = self.create_client(Trigger,'start_tracing')
        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        # Change this to the desired marker ID from pick_n_place.launch.py file, Declare parameter with default integer value
        self.markerid = self.declare_parameter('markerid', 0).get_parameter_value().integer_value

        self.target_marker_id = self.markerid 

        self.subscription = self.create_subscription(
            MarkerArray,
            '/detected_markers',
            self.aruco_listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)

        self.twist = Twist()
        self.finish_move = False


        self.subscription_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription_  # prevent unused variable warning

        self.get_joint = False
        self.marker = []

        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.trajectory_msg = JointTrajectory()

        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    

        self.point = JointTrajectoryPoint()
        self.point.velocities = [0.0] * 4
        self.point.accelerations = [0.0] * 4
        self.point.time_from_start.sec = 0
        self.point.time_from_start.nanosec = 500

        #sample_pkg/src/simple_manager_node.py
        # 상태 변수
        
        self.aruco_position_x =None
        self.aruco_position_y =None
        self.aruco_position_z =None        



        self.arm_triggered = False          # arm 동작 여부
        self.marker_id = None
        self.state = 'START'  

        
        
        self.aruco_pose_saved = False  # __init__에 추가

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def euler_to_quaternion(self,roll, pitch, yaw):
        """
        roll, pitch, yaw (radians) -> quaternion (x, y, z, w)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return (qx, qy, qz, qw)
##################################################################################
##################################################################################
    def append_pose_init(self, x_cam, y_cam, z_cam):
        try:
            # 1. ArUco로부터 받아온 위치를 camera_link 기준 PoseStamped로 설정
            marker_pose_cam = PoseStamped()
            marker_pose_cam.header.frame_id = "camera_link"  # 카메라 좌표계 이름
            marker_pose_cam.header.stamp = self.get_clock().now().to_msg()
            marker_pose_cam.pose.position.x = x_cam
            marker_pose_cam.pose.position.y = y_cam
            marker_pose_cam.pose.position.z = z_cam
            yaw = math.atan2(y_cam, x_cam)  # z축 기준 회전 (단위: radian)
            q = self.euler_to_quaternion(0, 0, yaw)
            marker_pose_cam.pose.orientation.x = q[0]
            marker_pose_cam.pose.orientation.y = q[1]
            marker_pose_cam.pose.orientation.z = q[2]
            marker_pose_cam.pose.orientation.w = q[3]
            self.get_logger().info(
                f"{BLUE}[DEBUG] Marker Pose (Camera): x={x_cam:.3f}, y={y_cam:.3f}, z={z_cam:.3f}{RESET}"
            )
            # 2. camera_link → base_link로 변환
            transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame='link5',
                time=rclpy.time.Time()
            )
            pose_in_base = do_transform_pose_stamped(marker_pose_cam, transform)
            # pose_in_base.pose.orientation.w = 1.0  # orientation 유지 or 수정 필요시 조정
            self.get_logger().info(
                f"{MAGENTA}[DEBUG] Transformed Pose (Base): x={pose_in_base.pose.position.x:.3f}, "
                f"y={pose_in_base.pose.position.y:.3f}, z={pose_in_base.pose.position.z:.3f}{RESET}"
            )
            # x_original = pose_in_base.pose.position.x
            # y_original = pose_in_base.pose.position.y
            # pose_in_base.pose.position.x = -y_original
            # pose_in_base.pose.position.y = -x_original
            # 3. PoseArray로 래핑
            pose_array = PoseArray()
            pose_array.poses.append(pose_in_base.pose)
            return pose_array
        except Exception as e:
            self.get_logger().error(f"{RED}[ERROR] append_pose_init failed: {e}{RESET}")
            return None

    def joint_states_callback(self, msg):

        if self.get_joint == False:
            return
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    def aruco_listener_callback(self, msg):
        # time.sleep(2)
        # print("Gripper Open")
        # response = arm_client.send_request(2, "open")
        # arm_client.get_logger().info(f'Response: {response.response}')
        self.get_logger().info(f"{BLUE}[CALLBACK] MarkerArray received with {len(msg.markers)} markers{RESET}")
    
        for marker in msg.markers:
            self.get_logger().info(f"{CYAN} Checking Marker ID: {marker.id}{RESET}")
        
            print("markers")
            if marker.id == self.target_marker_id:
                self.get_logger().info(f"{GREEN}[MATCH] Found Marker ID {marker.id}{RESET}")
                self.get_logger().info(f'First Marker Detected - Position: x={marker.pose.pose.position.x}, y={marker.pose.pose.position.y}, z={marker.pose.pose.position.z}')
                
                # 처음에 들어온 값만 저장
                if not self.aruco_pose_saved:
                    self.aruco_position_x = marker.pose.pose.position.x
                    self.aruco_position_y = marker.pose.pose.position.y
                    self.aruco_position_z = marker.pose.pose.position.z
                    self.aruco_pose_saved = True  # 이후 덮어쓰기 방지
                    self.get_logger().info(f"{YELLOW}[INFO] Saved marker position for first time{RESET}")
                    self.aruco_arm_controll()
                else:
                    self.get_logger().info(f"[SKIP] Marker already saved → skipping duplicate processing.")

               
                     
    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)  
    
    def aruco_arm_controll(self):
        if None in (self.aruco_position_x,self.aruco_position_y,self.aruco_position_z):
            print('aruco not initialized')
            return
        print("Impossible Mission Start")        
        arm_client = TurtlebotArmClient()
        
        print(f"Mission Aruco marker Locaion coordinates: {self.aruco_position_x}, {self.aruco_position_y}, {self.aruco_position_z}")
        
        time.sleep(2)
        # pose_array = self.append_pose_init(0.137496 - self.aruco_position_y + 0.05,0.00 - self.aruco_position_x ,0.122354 )
        pose_array=self.append_pose_init(self.aruco_position_x,self.aruco_position_y,self.aruco_position_z)

        response = arm_client.send_request(0, "", pose_array)
        # response = arm_client.send_request(4, "movel", pose_array)
        arm_client.get_logger().info(f'Response: {response.response}')

        ###추가
        req = Trigger.Request()
        result = self.start_client.call(req)
        self.get_logger().info(f"Success: {result.success}, Message: {result.message}")
        ##

        
       

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()
 

    joint_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    trajectory_msg = JointTrajectory()

    trajectory_msg.header = Header()
    trajectory_msg.header.frame_id = ''
    trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 4
    point.accelerations = [0.0] * 4
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 500


  
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()