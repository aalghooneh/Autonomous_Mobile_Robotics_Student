

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2



class ImageVisualizationNode(Node):
    def __init__(self):
        super().__init__('image_visualization_node')

        self.subscription_ = self.create_subscription(
            CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.image_callback, 10)


        self.bridge = CvBridge()

    def image_callback(self, msg):
        
        # Decompress the compressed image
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Image', 800, 600)


        # Visualize the image in an OpenCV window
        cv2.imshow('Image', img)
        cv2.waitKey(1)





def main(args=None):
    rclpy.init(args=args)
    image_visualization_node = ImageVisualizationNode()
    rclpy.spin(image_visualization_node)
    image_visualization_node.destroy_node()
    rclpy.shutdown()







if __name__ == '__main__':
    main()

