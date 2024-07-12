import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pyzbar.pyzbar import decode

class ExampleNode(Node):

    def __init__(self):
        super().__init__('example_node')
        self.pose_subscriber = self.create_subscription(PoseStamped, "/orb_slam2_stereo_node/pose", self.coordinates_callback, 10)
        self.slam_debug_image_subscriber = self.create_subscription(Image, "/orb_slam2_stereo_node/debug_image", self.slam_debug_img_callback, 10)
        self.stereo_left_image_subscriber = self.create_subscription(Image, "/stereo_left", self.stereo_left_img_callback, 10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, 'qrFound', 10)

    def coordinates_callback(self, msg: PoseStamped):
        self.get_logger().info("{: .2f}, {: .2f}, {: .2f}".format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    def slam_debug_img_callback(self, imageMsg: Image):
        pass
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(imageMsg, "bgr8")
        # except CvBridgeError as e:
        #     print(e)

        # (rows, cols, channels) = cv_image.shape
        # print("r {} x c {} x ch {} {}".format(rows, cols, channels, imageMsg.encoding))
        # cv2.imshow("SLAM debug", cv_image)
        # cv2.waitKey(3)

    def stereo_left_img_callback(self, imageMsg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(imageMsg, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        print("r {} x c {} x ch {} {}".format(rows, cols, channels, imageMsg.encoding))
        cv2.imshow("Stereo left", cv_image)
        cv2.waitKey(3)

        qr_result = decode(cv_image)

        msg = String()
        if qr_result:
            print(qr_result[0].data)
            msg.data = "found it!"
        else:
            msg.data = "nope"

        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    subscriber = ExampleNode()

    rclpy.spin(subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
