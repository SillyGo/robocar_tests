from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import rclpy 
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2 
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

#parâmetros da câmera: obti esses valores fazendo ros2 topic echo /lane_camera/camera_info e guardando os resultados.
fx = 381.3469774171306
fy = 381.3469774171306
cx = 320.0
cy = 240.0

class PointCloudPub(Node):
    def __init__(self):
        super().__init__('camFramePointcloudPub')
        self.depthSub = self.create_subscription(Image, '/lane_camera/depth_image', self.depthCallback, 10)
        self.rgbSub = self.create_subscription(Image, '/lane_camera/image', self.rgbCallback, 10)

        #ps. quanto ao depth e rgb imagens, terei que sincronizar os tópicos aqui, oque pode causar alguns efeitos indesejados;
        #porém, vale ressaltar que essa sincronização não terá que ser feita numa situação real, logo que isso vai tudo ser resolvido
        #pela lib da realsense. Além disso, Poderei ler as imagens diretamente no código que envia a pointcloud, diretamente do sensor.

        self.pcPub = self.create_publisher(PointCloud2, '/camFramePC', 10)
        self.br = CvBridge()
        
        self.last_depth_image = None

    def depthCallback(self, msg:Image):
        self.last_depth_image = msg                 #salva a última imagem de profundidade recebida em um buffer. TODO: transformar esse buffer em um buffer de 3 elementos depois
        return
    
    def rgbCallback(self, msg:Image):

        #PSEUDOCODIGO:
        #1. garante que a imagem RGB recebida aqui (msg) e a imagem de profundidade do buffer estão sincronizadas
        #2. garante que as imagens tem as mesmas dimensões e que estas estão, portanto, alinhadas.
        #3. caso todos os testes de segurança dêem certo, prosseguimos.
        
        #4. faz todo o processo antes feito para encontrar as bordas na imagem RGB
        #5. pega os pixels das bordas e salva
        #6. vê o valor de profundidade destes pixels na imagem de profundidade, e salva esses valores em uma 3-upla (pixel_x, pixel_y, z)
        #7. sua a 3-upla e a fórmula de conversão pixel-cartesiano para encontrar a 3-upla (x,y,z) do ponto
        #8. salva todas essas 3-uplas (x,y,z) em um grande array
        #9. converte esse array para pointcloud 
        #10. publica a pointcloud no frame 'Camera Link'

        DepthMsg = self.last_depth_image # evita a mudança desse valor durante a execução!

        timeMono = msg.header.stamp.nanosec
        timeDepth = DepthMsg.header.stamp.nanosec

        precision = 500

        if abs(timeMono - timeDepth) >= precision:
            self.get_logger().warn("ERRO: mensagens não sincronizadas!")
            return
        else:
            self.get_logger().info("INFO: mensagens recebidas estão sincronizadas!")

        MonoImage = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
        DepthImage = self.br.imgmsg_to_cv2(DepthMsg, desired_encoding='passthrough') 

        #VI E GARANTO: MonoImage e DepthImage tem as mesmas dimensões.

        #Com isso, todos os testes de segurança deram certo. Daremos prosseguimento à execução do código.

        h, w = MonoImage.shape[:2]

        MonoImage = MonoImage[0:int(5*h/5), :]

        h, w = MonoImage.shape[:2]

        gray = cv2.cvtColor(MonoImage, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 100)

        edge_points = np.argwhere(edges > 0)

        rows = edge_points[:, 0]
        cols = edge_points[:, 1]

        Z = DepthImage[rows, cols]

        X = Z * (cols - cx) / fx
        Y = Z * (rows - cy) / fy

        global_points = np.stack((X, Y, Z), axis=1)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'lane_camera_optical_frame'

        cloud_msg = pc2.create_cloud_xyz32(header, global_points)
        self.pcPub.publish(cloud_msg)

        cv2.imshow('Webcam Display', MonoImage)
        cv2.waitKey(1)    

        return

if __name__ == '__main__':
    rclpy.init()
    pubnode = PointCloudPub()
    rclpy.spin(pubnode)
    pubnode.destroy_node()
    rclpy.shutdown()
