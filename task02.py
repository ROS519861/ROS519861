#!/usr/bin/python
# -*- coding:utf8 -*-
# 示例代码框架（ROS + OpenCV）
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class DualLineFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cam", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.kp = 0.1  # PID 参数
        self.ki = 0.01
        self.kd = 0.05

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        # 检测双线并计算中线偏差
        left_line, right_line = detect_dual_lines(binary)
        center_line = (left_line + right_line) // 2
        error = center_line - cv_image.shape[1] // 2
        # PID 控制
        steering = self.kp * error + self.ki * self.integral + self.kd * (error - self.last_error)
        self.twist.angular.z = steering
        self.cmd_vel_pub.publish(self.twist)

def detect_dual_lines(image):
    # 实现双线检测逻辑（示例：滑动窗口法）
    pass

if __name__ == "__main__":
    rospy.init_node("dual_line_follower")
    follower = DualLineFollower()
    rospy.spin()
