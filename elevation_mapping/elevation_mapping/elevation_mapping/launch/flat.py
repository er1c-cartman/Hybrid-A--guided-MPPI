#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from grid_map_msgs.msg import GridMapInfo
import numpy as np

class FlatSurfacePublisher(Node):

    def __init__(self):
        super().__init__('flat_surface_publisher')
        self.declare_parameter('radius', 1.0)
        self.radius = self.get_parameter('radius').value

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'base_link_pose',  # The robot pose topic
            self.pose_callback,
            10)

        self.elevation_subscriber = self.create_subscription(
            GridMap,
            '/elevation_map',  # The elevation map topic
            self.elevation_callback,
            10)

        self.elevation_publisher = self.create_publisher(
            GridMap,
            'flat_elevation_map',
            10)

        self.robot_pose = None
        self.elevation_map = None

    def pose_callback(self, msg):
        self.robot_pose = msg.pose
        self.publish_flat_surface()

    def elevation_callback(self, msg):
        self.elevation_map = msg
        self.publish_flat_surface()

    def publish_flat_surface(self):
        if self.robot_pose is None or self.elevation_map is None:
            return

        # Extract robot position
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        # Modify the elevation map to flatten the circular area
        resolution = self.elevation_map.info.resolution
        origin_x = self.elevation_map.info.pose.position.x
        origin_y = self.elevation_map.info.pose.position.y

        flat_radius = self.radius

        for layer_name in self.elevation_map.layers:
            layer_idx = self.elevation_map.layers.index(layer_name)
            data = np.array(self.elevation_map.data[layer_idx].data).reshape((self.elevation_map.info.length_y, self.elevation_map.info.length_x))

            for i in range(self.elevation_map.info.length_y):
                for j in range(self.elevation_map.info.length_x):
                    x = origin_x + j * resolution
                    y = origin_y + i * resolution
                    distance = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
                    if distance <= flat_radius:
                        data[i, j] = 0  # Set to 0 for a flat surface

            self.elevation_map.data[layer_idx].data = data.flatten().tolist()

        self.elevation_publisher.publish(self.elevation_map)

def main(args=None):
    rclpy.init(args=args)
    flat_surface_publisher = FlatSurfacePublisher()
    rclpy.spin(flat_surface_publisher)
    flat_surface_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

