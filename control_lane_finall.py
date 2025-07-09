 #!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import math
import numpy as np
# MAX_ANGULAR_Z = 1.0
MAX_ANGULAR_Z = 1.0

class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')
        self.start_service = self.create_service(Trigger, 'start_tracing', self.start_service_callback)
        self.tracing_flag= False
        
        #'/control/lane', 리매핑 ->'/detect/lane'
        #:카메라 프레임상 중앙차선 x좌표 구독
        self.sub_lane = self.create_subscription(
            Float64,
            '/detect/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64, #현재 발행중인 놈 없음
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel', #퍼블리시 
            1
        )

        # PD control related variables
        self.last_error = 0
        # self.MAX_VEL = 0.2 #0.1->2 #지금은 안씀
        self.MAX_VEL=0.3

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()
        self.last_center=160.0

    def start_service_callback(self, request, response):
        self.get_logger().info("트레이싱 시작")
        response.success = True
        response.message = "tracing_start"
        self.tracing_flag= True
        return response
        # 동작 실행

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data
        """
        로봇의 속도를 상황에 따라 유동적으로 조정 가능
        """
        #self.MAX_VEL: 이 값을 callback_follow_lane()에서 선속도 계산 시 사용

    def callback_follow_lane(self, desired_center):
        if self.tracing_flag:
            pass
        else:#flag = Flase
            return
        if abs(self.last_center - desired_center.data)>=200:
            self.last_center=desired_center.data
            print("너무 차이커서 리턴")
            return
        center = desired_center.data #center: 현재 감지된 차선 중심 위치
        
        #현재 카메라에서 500이 중심점?
        #error > 0: 오른쪽으로 치우침
        #error < 0: 왼쪽으로 치우침
        """
        원본
        error = center - 500
        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error
        #오차가 크면 많이 돌고, 오차가 작으면 미세하게 조정

        twist = Twist()# ROS2의 기본 속도 메시지 객체 생성
        # Linear velocity: adjust speed based on error (maximum 0.05 limit)
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
        """
        error = (center - 160)*2 #error: 중앙(500픽셀 기준)과의 차이
        self.last_center=desired_center.data

        # Kp = 0.0025
        Kp = 0.003

        # Kd = 0.007
        Kd=0.01

        zk=1.2
        # angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error
        scaled_error = np.sign(error) * (abs(error) ** zk) / (160 ** zk) * 160
        angular_z = Kp * scaled_error + Kd * (scaled_error - self.last_error)
        #오차가 크면 많이 돌고, 오차가 작으면 미세하게 조정
        twist = Twist()# ROS2의 기본 속도 메시지 객체 생성
        # Linear velocity: adjust speed based on error (maximum 0.05 limit)
        twist.angular.z = (-max(angular_z, -MAX_ANGULAR_Z) if angular_z < 0 else -min(angular_z, MAX_ANGULAR_Z))
        # twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 160, 0) ** 2.2), 0.3) #오차가 작을수록 빠르게, 클수록 느리게 이동
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 160, 0) ** 0.7), 0.26) #오차가
        #0.05->0.1
        print(f"선 속도 : {twist.linear.x} 각속도 : {twist.angular.z}")
        print(f"중심좌표 :{desired_center.data}")
        self.pub_cmd_vel.publish(twist) #속도 발송



    def shut_down(self):
        #shut_down()은 노드가 종료될 때 로봇을 정지시키기 위해 호출되는 함수
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
