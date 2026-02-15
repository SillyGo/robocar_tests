import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 
import numpy as np 

from sklearn.cluster import KMeans
from sklearn.cluster import MiniBatchKMeans

import time

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
    time_start = time.time()
    k = 5

    h,w = frame.shape[0], frame.shape[1]

    for i in range(w):
        for j in range(int(h/3)):
            frame[j][i][0] = 0
            frame[j][i][1] = 0
            frame[j][i][2] = 0
    
    X = frame.reshape(-1,3) 
    #km = KMeans(n_clusters=k, n_init=10)
    km = MiniBatchKMeans(n_clusters=k, n_init=1,batch_size=100,max_iter=50)

    km.fit(X)
    segmented_image = km.cluster_centers_[km.labels_]
    segmented_image = segmented_image.reshape(frame.shape)

    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    threshold1 = 100
    threshold2 = 150
    edges = cv2.Canny(gray_image,threshold1,threshold2)

    print(f"time diff: {time.time() - time_start}")

    return edges

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
