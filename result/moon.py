import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import json
import os
import re
from datetime import datetime
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from scipy.spatial.transform import Rotation as R
import pandas as pd


class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define navigation goals
        self.goals = [
            {'x': 1.51, 'y': -6.52, 'z': -0.87, 'w': 0.47}, 
            {'x': -7.34, 'y': -10.36, 'z': -0.98, 'w': -0.15}, 
            {'x': -15.52, 'y': -11.9, 'z': 0.96, 'w': 0.25}, 
            {'x': -23.59, 'y': -5.32, 'z': 0.81, 'w': 0.57},
            {'x': -27.42, 'y': 3.57, 'z': 0.81, 'w': 0.57}, 
            {'x': -35.40, 'y': 4.37, 'z': 0.81, 'w': 0.58}  # Example goals
        ]
        self.current_goal_index = 0
        self.execution_folder = self.create_execution_folder()
        self.send_next_goal()

        # Initialize attributes
        self.data = []
        self.odom_data = None
        self.last_position = None
        self.last_pose = None
        self.total_path_length = 0.0
        self.total_heading_change = 0.0
        self.start_time = self.get_clock().now()

        # New attributes for roll and pitch
        self.rolling_angles = []
        self.pitching_angles = []

        # ROS2 Timers and Subscriptions
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 100)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        self.latest_linear_velocity = msg.linear.x
        self.latest_angular_velocity = msg.angular.z

    def send_next_goal(self):
        if self.current_goal_index < len(self.goals):
            goal_dict = self.goals[self.current_goal_index]
            goal_msg = NavigateToPose.Goal()

            # Create a PoseStamped for the goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "odom"  # Use "map" or another appropriate frame_id
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_dict['x']
            goal_pose.pose.position.y = goal_dict['y']
            goal_pose.pose.orientation.z = goal_dict['z']
            goal_pose.pose.orientation.w = goal_dict['w']

            goal_msg.pose = goal_pose

            self.action_client.wait_for_server()
            self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)

            self.current_goal_index += 1
        else:
            self.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal succeeded!')
            if self.current_goal_index >= len(self.goals):
                self.shutdown()
            else:
                self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        pass

    def timer_callback(self):
        if self.odom_data is not None:
            current_time = self.get_clock().now()
            elapsed_time = current_time - self.start_time
            elapsed_time_sec = elapsed_time.nanoseconds / 1e9

            current_pose = self.odom_data.pose.pose

            # Calculate path length and heading change
            if self.last_position is not None:
                current_position = self.odom_data.pose.pose.position
                distance = self.calculate_distance(self.last_position, current_position)
                self.total_path_length += distance
                heading_change = self.calculate_heading_change(self.last_pose, current_pose)
                self.total_heading_change += heading_change

            # Calculate roll, pitch, and yaw
            roll, pitch, yaw = self.quaternion_to_euler(current_pose.orientation)
            self.rolling_angles.append(roll)
            self.pitching_angles.append(pitch)

            # Calculate AOL
            aol = self.total_heading_change / self.total_path_length if self.total_path_length > 0 else 0.0

            data_point = {
                'Goal Number': self.current_goal_index,
                'Time': elapsed_time_sec,
                'X': current_pose.position.x,
                'Y': current_pose.position.y,
                'Z': current_pose.position.z,
                'linear_velocity': self.odom_data.twist.twist.linear.x,
                'angular_velocity': self.odom_data.twist.twist.angular.z,
                'Total Path Length': self.total_path_length,
                'AOL': aol,
                'Roll': roll,
                'Pitch': pitch
            }
            self.data.append(data_point)

            self.last_position = current_pose.position
            self.last_pose = current_pose

    def odom_callback(self, msg: Odometry):
        self.odom_data = msg

    def quaternion_to_euler(self, quaternion):
        """Convert quaternion to roll, pitch, and yaw."""
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        return roll, pitch, yaw

    def calculate_distance(self, last_position, current_position):
        """Calculate Euclidean distance between two points."""
        dx = current_position.x - last_position.x
        dy = current_position.y - last_position.y
        dz = current_position.z - last_position.z
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def calculate_heading_change(self, pose1, pose2):
        """Calculate heading change between two poses."""
        yaw1 = self.quaternion_to_euler(pose1.orientation)[2]
        yaw2 = self.quaternion_to_euler(pose2.orientation)[2]
        heading_change = np.abs(yaw2 - yaw1)
        if heading_change > np.pi:
            heading_change = (2 * np.pi) - heading_change
        return heading_change

    def create_execution_folder(self):
        dirs = [d for d in os.listdir(os.getcwd()) if os.path.isdir(d) and d.startswith('test')]
        max_num = max([int(re.search(r'test(\d+)', d).group(1)) for d in dirs if re.search(r'test(\d+)', d)], default=0)
        new_dir_name = f"test{max_num + 1}"
        os.makedirs(new_dir_name, exist_ok=True)
        return new_dir_name

    def shutdown(self):
        avg_roll = np.mean(self.rolling_angles) if self.rolling_angles else 0.0
        var_roll = np.var(self.rolling_angles) if self.rolling_angles else 0.0
        avg_pitch = np.mean(self.pitching_angles) if self.pitching_angles else 0.0
        var_pitch = np.var(self.pitching_angles) if self.pitching_angles else 0.0

        # Calculate RMS for roll and pitch
        rms_roll = np.sqrt(np.mean(np.square(self.rolling_angles))) if self.rolling_angles else 0.0
        rms_pitch = np.sqrt(np.mean(np.square(self.pitching_angles))) if self.pitching_angles else 0.0

        self.get_logger().info(f"Average Roll: {avg_roll}, Variance Roll: {var_roll}, RMS Roll: {rms_roll}")
        self.get_logger().info(f"Average Pitch: {avg_pitch}, Variance Pitch: {var_pitch}, RMS Pitch: {rms_pitch}")

        # Pass rms_roll and rms_pitch to the save methods
        self.save_data_to_excel(rms_roll, rms_pitch)
        self.save_data_to_json(rms_roll, rms_pitch)
        self.get_logger().info('Shutting down...')
        rclpy.shutdown()


    def save_data_to_excel(self, rms_roll, rms_pitch):
        filename = os.path.join(self.execution_folder, 'data.xlsx')
        df = pd.DataFrame(self.data)
        df['Average Roll'] = [np.mean(self.rolling_angles)] * len(self.data)
        df['Variance Roll'] = [np.var(self.rolling_angles)] * len(self.data)
        df['RMS Roll'] = [rms_roll] * len(self.data)
        df['Average Pitch'] = [np.mean(self.pitching_angles)] * len(self.data)
        df['Variance Pitch'] = [np.var(self.pitching_angles)] * len(self.data)
        df['RMS Pitch'] = [rms_pitch] * len(self.data)
        df.to_excel(filename, index=False)


    def save_data_to_json(self, rms_roll, rms_pitch):
        json_path = os.path.join(self.execution_folder, 'data.json')
        summary = {
            'Average Roll': np.mean(self.rolling_angles),
            'Variance Roll': np.var(self.rolling_angles),
            'RMS Roll': rms_roll,
            'Average Pitch': np.mean(self.pitching_angles),
            'Variance Pitch': np.var(self.pitching_angles),
            'RMS Pitch': rms_pitch
        }
        with open(json_path, 'w') as json_file:
            json.dump({'data': self.data, 'summary': summary}, json_file, indent=4)



def main(args=None):
    rclpy.init(args=args)
    navigator = RobotNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

