#!/usr/bin/python
# -*- coding:utf8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from robot_task.cfg import pid_paramsConfig

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# 全局变量
bridge = CvBridge()
current_image = None
pid = PIDController(Kp=0.02, Ki=0.001, Kd=0.005)

def image_callback(msg):
    global current_image
    current_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def process_image(image):
    height, width = image.shape[:2]
    roi = image[int(height*0.5):height, :]  # 仅处理下半部分
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    binary = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 11, 2
    )
    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # 补偿ROI偏移
            cy_global = cy + int(height*0.5)
            # 绘制中心点
            cv2.circle(image, (cx, cy_global), 5, (0, 0, 255), -1)
            cv2.line(image, (width//2, 0), (width//2, height), (0, 255, 0), 2)
            cv2.imshow("Debug", image)
            cv2.waitKey(1)
            return cx, cy_global
    return None, None

def dynamic_reconfigure_callback(config, level):
    pid.Kp = config.Kp
    pid.Ki = config.Ki
    pid.Kd = config.Kd
    return config

def line_following():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    max_speed = 0.3
    min_speed = 0.1
    error_threshold = 50

    while not rospy.is_shutdown():
        global current_image
        if current_image is not None:
            cx, cy = process_image(current_image)
            if cx is not None:
                error = cx - current_image.shape[1] // 2
                dt = 0.1
                control = pid.compute(error, dt)
                # 动态调整速度
                if abs(error) > error_threshold:
                    speed = min_speed
                else:
                    speed = max_speed - (abs(error) / error_threshold) * (max_speed - min_speed)
                twist = Twist()
                twist.linear.x = speed
                twist.angular.z = -control
                pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('line_follower')
        rospy.Subscriber('/cam', Image, image_callback)
        srv = Server(pid_paramsConfig, dynamic_reconfigure_callback)
        line_following()
    except rospy.ROSInterruptException:
        pass
