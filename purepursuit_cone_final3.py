#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, pi, sqrt, pow, atan2, hypot
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import numpy as np
from visualization_msgs.msg import Marker

from morai_msgs.msg import CtrlCmd

class pure_pursuit:
    def __init__(self):
        rospy.init_node('purepursuit_cone', anonymous=True)
        # rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/selected_path", Path, self.path_callback)
        # rospy.Subscriber("/speed", Bool, self.speed_callback)
        # self.ctrl_cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)

        # self.ctrl_cmd_msg = Twist()
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2
        self.forward_point = Point()
        self.is_look_forward_point = False

        self.vehicle_length = 2.0
        self.lfd = 4.0   # 기본 Ld
        self.path = Path()
        # self.slow = False
        self.target_vel = 20  # 기본 vel 80
        self.vel_penalty = 0.0
        self.BASE_LD = 2.0
        self.MIN_LD = 1.0
        self.MAX_LD = 3.0

    def path_callback(self, msg):
        self.path = msg

    # def speed_callback(self, msg):
    #     self.slow = msg.data

    def calculate_curvature(self, path):
        if len(path.poses) < 3:
            return 0.0

        # local_path의 첫점, p3의 인덱스 절반 지점, MAX_LD 점을 사용
        p1 = path.poses[0].pose.position
        
        # p3 인덱스 찾기 (MAX_LD 거리에 해당하는 포인트)
        p3_index = len(path.poses) - 1  # 기본값은 마지막 포인트
        
        for i, pose in enumerate(path.poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            dis = sqrt(x**2 + y**2)
            if dis >= self.MAX_LD:
                p3_index = i
                break
        
        # p2를 p3 인덱스의 절반에 해당하는 포인트로 설정
        p2_index = p3_index // 2
        p2 = path.poses[p2_index].pose.position
        p3 = path.poses[p3_index].pose.position
        

        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x3, y3 = p3.x, p3.y

        area = abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)) / 2.0
        d1 = hypot(x2-x1, y2-y1)
        d2 = hypot(x3-x2, y3-y2)
        d3 = hypot(x1-x3, y1-y3)

        if d1*d2*d3 == 0:
            return 0.0
        return 4*area/(d1*d2*d3)

    def speed_control(self, curvature):
        MAX_SPEED = 20.0 #직선일 때 최대속도 80.0
        CURV_TH = 0.02 #곡률이 CURV_TH(0.02) 보다 크면 감속(낮추면 조금만 커브여도 감속)
        CURV_FACTOR = 70.0 #곡률에 따른 감속 강도(크면 코너에서 감속 많이함)

        if curvature > CURV_TH:
            self.vel_penalty = min(MAX_SPEED*0.4, curvature * CURV_FACTOR)
        else:
            self.vel_penalty = 0.0

        target_vel = max(MAX_SPEED - self.vel_penalty, 10.0)

        # if self.slow:
        #     target_vel = 60.0

        return target_vel

    def update_lookahead(self, curvature):
        self.BASE_LD = 2.0
        self.MIN_LD = 1.0
        self.MAX_LD = 3.0

        if curvature > 0.02:
            self.lfd = self.MIN_LD
        else:
            self.lfd = self.BASE_LD

P = pure_pursuit()

class marker():
    def __init__(self):
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.marker = Marker()
        self.marker.header.frame_id = 'velodyne'
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.color.a = 100.0
        self.marker.color.g = 1.0
        self.marker.pose.orientation.w = 1.0
        self.num_points = 360

    def marker_publish(self):
        self.radius = P.lfd
        self.marker.action = Marker.DELETE
        self.marker.points = []
        self.marker_pub.publish(self.marker)

        self.marker.action = Marker.ADD
        for i in range(self.num_points+1):
            point = Point()
            point.x = self.radius * np.cos(2*np.pi*i/self.num_points)
            point.y = self.radius * np.sin(2*np.pi*i/self.num_points)
            point.z = 0.0
            self.marker.points.append(point)
        self.marker_pub.publish(self.marker)

M = marker()

def main():
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if len(P.path.poses) > 0:
            curvature = P.calculate_curvature(P.path)
            P.target_vel = P.speed_control(curvature)
            P.update_lookahead(curvature)

            P.is_look_forward_point = False

            for i in P.path.poses:
                x = i.pose.position.x
                y = i.pose.position.y
                dis = hypot(x, y)
                if dis >= P.lfd:
                    P.forward_point = i.pose.position
                    P.is_look_forward_point = True
                    break

            if P.is_look_forward_point:
                alpha = atan2(P.forward_point.y, P.forward_point.x)
                steer = atan2((2*P.vehicle_length*sin(alpha)), P.lfd)

                steer = max(-30*pi/180, min(steer, 30*pi/180)) #코너링 한계 각도

                # P.ctrl_cmd_msg.linear.x = P.target_vel
                P.ctrl_cmd_msg.velocity = P.target_vel 
                # P.ctrl_cmd_msg.angular.z = steer * -1 * 0.55 * 1 #0.55 (크면 핸들 반응 빠름, 작으면 부드럽고 느림)
                P.ctrl_cmd_msg.steering = steer * 0.55 #0.55 (크면 핸들 반응 빠름, 작으면 부드럽고 느림)
            else:
                # P.ctrl_cmd_msg.linear.x = 0.0
                P.ctrl_cmd_msg.velocity = 0.0
                # P.ctrl_cmd_msg.angular.z = 0.0
                P.ctrl_cmd_msg.steering = 0.0

            P.ctrl_cmd_pub.publish(P.ctrl_cmd_msg)

            print("Curvature: {:.3f} | Steering [deg]: {:.1f} | Velocity [km/h]: {:.1f} | Penalty [km/h]: {:.2f} | Lfd: {:.2f}".format(
                curvature, steer*180/pi, P.target_vel, P.vel_penalty, P.lfd))

        M.marker_publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
