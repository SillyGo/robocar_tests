from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import rclpy 
from rclpy.node import Node
from cv_bridge import CvBridge

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

        return

if __name__ == '__main__':
    rclpy.init()
    pubnode = PointCloudPub()
    rclpy.spin(pubnode)
    pubnode.destroy_node()
    rclpy.shutdown()
