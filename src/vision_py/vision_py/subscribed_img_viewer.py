import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self, name):
        # initiate the node and give it a name
        super().__init__(name)
        self.get_logger().info(f"{name} node has activated!")
        # declare parameter
        self.declare_parameter('image_topic', '/my_zed2i/left_image_raw')
        # get parameter
        image_topic = self.get_parameter("image_topic").value

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
    rclpy.init()
    image_subscriber = ImageSubscriber(
        'image_subscriber'
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