import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self, name, image_topic='/zed2i/zed_node/left_raw/image_raw_color'):
        # initiate the node and give it a name
        super().__init__(name)

        # create the subscriber, receive the image
        self.img_subscription = self.create_subscription(
            msg_type = Image,
            topic = image_topic,
            callback = self.img_callback,
            qos_profile = 10
        )  
        # to convert between ROS and Opencv images
        self.br = CvBridge()

    def img_callback(self, msg):
        # display the messages on the console
        self.get_logger().info('Receiveing video frames...')

        # convet the ROS image to Opencv bgr image
        current_frame = self.br.imgmsg_to_cv2(msg)
        # current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        cv2.imshow('camera', current_frame)
        cv2.waitKey(500)


def main():
    parser = argparse.ArgumentParser()
    # 添加命令行参数
    parser.add_argument("--img", type=str, default='/zed2i/zed_node/left_raw/image_raw_color', help="发布图像topic")

    # 解析命令行参数
    args = parser.parse_args()
    rclpy.init()

    
    image_subscriber = ImageSubscriber(
        'image_subscriber',
        args.img
        )
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    # destory the node explicitly
    image_subscriber.destroy_node()
    # shutdown the ROS client library for python
    rclpy.shutdown()

if __name__ == '__main__':
    main()