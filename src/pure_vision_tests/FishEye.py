import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 
import numpy as np 

fx = 381.3469774171306
fy = 381.3469774171306

cx = 320.0
cy = 240.0

K = np.array([
    [fx,0.0,cx  ],
    [0.0,fy,cy  ],
    [0.0,0.0,1.0]
])

d = np.array([1.0, 1.0, 1.0, 1.0])

FORWARD_VEL = 0.0
ANGULAR_VEL = 0.0

def change_vel(new_fv:float, new_av:float):
    global FORWARD_VEL, ANGULAR_VEL

    FORWARD_VEL = new_fv
    ANGULAR_VEL = new_av

def openCV_main(frame:np.ndarray):
    h, w = frame.shape[:2]

    mask = np.zeros(frame.shape[:2], dtype="uint8")

    cv2.rectangle(mask, (100, 100), (400, 400), 255, -1)

    frame = cv2.bitwise_and(mask,frame)
    
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, d, np.eye(3), K, (w, h), cv2.CV_32FC1
    )
    
    distorted_img = cv2.remap(
        frame, map1, map2, 
        interpolation=cv2.INTER_LINEAR, 
        borderMode=cv2.BORDER_CONSTANT
    )
    
    return distorted_img

class movementSender(Node):
    def __init__(self):
        super().__init__('cmdvel_publisher')
        self.cmvpub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer =  self.create_timer(0.5,self.timer_callback)

        self.imgSub = self.create_subscription(Image, '/lane_camera/image', self.imageCallback, 10)
        self.br = CvBridge()

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = FORWARD_VEL
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = ANGULAR_VEL

        self.cmvpub.publish(msg)

    def imageCallback(self, msg:Image):
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8') 

        mod_frame = openCV_main(current_frame)
        
        cv2.imshow('Webcam Display', mod_frame)
        cv2.waitKey(1)    

def main():
    rclpy.init()
    moveSend = movementSender()
    rclpy.spin(moveSend)
    moveSend.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
