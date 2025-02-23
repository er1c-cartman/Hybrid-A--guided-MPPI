#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

class TfToPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_to_pose_publisher')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'odom')

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.pose_topic = self.target_frame + '_pose'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.timer = self.create_timer(0.05, self.callback)

    def callback(self):
        try:
            # Use the latest available transform with a small timeout
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('Transform not available.')
            return

        pose = PoseWithCovarianceStamped()
        # Use the timestamp from the transform to ensure synchronization
        pose.header.stamp = trans.header.stamp
        pose.header.frame_id = self.source_frame
        pose.pose.pose.position.x = trans.transform.translation.x
        pose.pose.pose.position.y = trans.transform.translation.y
        pose.pose.pose.position.z = trans.transform.translation.z
        pose.pose.pose.orientation.x = trans.transform.rotation.x
        pose.pose.pose.orientation.y = trans.transform.rotation.y
        pose.pose.pose.orientation.z = trans.transform.rotation.z
        pose.pose.pose.orientation.w = trans.transform.rotation.w

        # Set a dummy covariance for the example
        pose.pose.covariance = [0.0] * 36

        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = TfToPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

