#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # 创建图像发布者
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        # 创建CvBridge用于OpenCV和ROS消息之间的转换
        self.bridge = CvBridge()
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            rclpy.shutdown()

        # 设置帧率
        self.timer = self.create_timer(0.033, self.publish_images)  # 30Hz

    def publish_images(self):
        # 从摄像头捕获一帧
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("无法获取图像")
            return

        # 将OpenCV图像转换为ROS消息
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")
            return

        # 设置ROS图像消息的时间戳
        ros_image.header.stamp = self.get_clock().now().to_msg()

        # 发布图像消息
        self.image_pub.publish(ros_image)

        # 发布相机信息
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = ros_image.header.stamp
        self.camera_info_pub.publish(camera_info_msg)

    def shutdown(self):
        # 关闭摄像头
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
