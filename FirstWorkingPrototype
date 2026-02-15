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

def relu(x):
    if x <= 0:
        return 0
    return x

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
LAST_TIME = time.time()

def change_vel(new_fv:float, new_av:float):
    global FORWARD_VEL, ANGULAR_VEL

    FORWARD_VEL = new_fv
    ANGULAR_VEL = new_av

def openCV_main(frame:np.ndarray):
    time_start = time.time()

    h,w = frame.shape[0], frame.shape[1]

    frame[0:int(h/3), :] = 0

    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    threshold1 = 100
    threshold2 = 150
    edges = cv2.Canny(gray_image,threshold1,threshold2)

    histogram_count = 15
    d0 = np.pi / histogram_count #diferença de ângulo entre dois histogramas

    bitmap = [1 for i in range(histogram_count)] #1 = perfeitamente limpo, 0 = obstáculo iminente

    factor = 3

    resized_edges = cv2.resize(edges, (int(w/factor), int(h/factor)), interpolation=cv2.INTER_AREA)

    w,h = int(w/factor), int(h/factor)

    max_d = np.sqrt(h**2 + (w/2)**2)

    edge_points = np.argwhere(resized_edges > 0)
    
    for v, u in edge_points:
        x = u - int(w/2)
        y = h - v
        x_factor = -x
        
        d = np.sqrt(x**2 + y**2)
        
        if d > 5:
            theta = np.arcsin(x_factor/d)
            index = int(theta / d0)
            
            points = max(0, d - 50) / (max_d - 50)
            
            if points <= bitmap[index]:
                bitmap[index] = points

    sum = 0
    div_by = 0
    for i in range(len(bitmap)):

        if bitmap[i] < 1:
            bitmap[i] = bitmap[i] * 0.5

        sum += i * bitmap[i]
        div_by += bitmap[i]

    if div_by == 0:
        change_vel(0.0,0.0)
    else:
        print(int(sum/div_by) - int(len(bitmap)/2))
        angle = (sum / div_by) - (len(bitmap)/2)

        print(angle)
        lin_vel = 1.5 / (abs(angle)+0.1)

        change_vel(1.5, -angle)
    
    #daqui para frente é só coisa de visualização

    vis_size = 400
    vis_center = vis_size // 2
    vis_radius = vis_size // 2 - 20
    histogram_vis = np.zeros((vis_size, vis_size, 3), dtype=np.uint8)

    rot = np.pi

    for i in range(histogram_count):
        angle_start = i * d0 + rot
        angle_end = (i + 1) * d0 + rot
        
        color_value = int((1 - bitmap[i]) * 255)
        color = (0, color_value, 0)
        
        pts = []
        pts.append([vis_center, vis_center])
        
        num_points = 20
        for j in range(num_points + 1):
            angle = angle_start + (angle_end - angle_start) * j / num_points
            x = int(vis_center + vis_radius * np.cos(angle))
            y = int(vis_center + vis_radius * np.sin(angle))
            pts.append([x, y])
        
        pts = np.array(pts, np.int32)
        cv2.fillPoly(histogram_vis, [pts], color)
        cv2.polylines(histogram_vis, [pts], True, (255, 255, 255), 1)
    
    histogram_vis = cv2.applyColorMap(histogram_vis, cv2.COLORMAP_JET)

    #print(f"time diff: {time.time() - time_start}")

    return edges, histogram_vis

class movementSender(Node):
    def __init__(self):
        super().__init__('cmdvel_publisher')
        self.cmvpub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer =  self.create_timer(0.05,self.timer_callback)

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
        global LAST_TIME
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8') 

        mod_frame, histogram_vis = openCV_main(current_frame)
        
        cv2.imshow('Webcam Display', mod_frame)
        cv2.imshow('Radial Histogram', histogram_vis)
        cv2.waitKey(1)    

        cur_time = time.time()

        print(f"FPS: {1/(cur_time - LAST_TIME)}")

        LAST_TIME = cur_time

def main():
    rclpy.init()
    moveSend = movementSender()
    rclpy.spin(moveSend)
    moveSend.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
