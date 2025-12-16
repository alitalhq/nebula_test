#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class ImageRawSubscriber(Node):
    def __init__(self):
        super().__init__("image_raw_subscriber")

        # ---- CAMERA QoS'larını birebir aynı oluştur ----
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        cam_info_qos = QoSProfile(depth=1)
        cam_info_qos.reliability = QoSReliabilityPolicy.RELIABLE

        # ---- Subscribers ----
        self.image_sub = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            image_qos,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            "camera/camera_info",
            self.info_callback,
            cam_info_qos,
        )

        self.bridge = CvBridge()
        self.latest_info = None

        self.get_logger().info("Image raw subscriber hazır.")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.imshow("camera/image_raw", frame)
        cv2.waitKey(1)

    def info_callback(self, msg: CameraInfo):
        self.latest_info = msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageRawSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

