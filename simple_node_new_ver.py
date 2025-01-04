# it read data from a Json file
# output of json like: {"timestamp": 1736010539.8562458, "detected": false, "tracking": false, "distance": null, "x_center": 0.0, "search_command": 0.0}

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
from pathlib import Path

class SimpleFollowerNode(Node):
    def __init__(self):
        super().__init__('simple_follower')
        
        # Create publisher for robot velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create timer for checking detection data
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # Parameters
        self.target_distance = 1.0  # meters
        self.max_linear_speed = 0.26  # m/s
        self.max_angular_speed = 1.82  # rad/s
        self.search_angular_speed = 0.5  # rad/s for search mode
        
        # Update file path to use absolute path in Docker container
        self.detection_file_path = Path('/root/shared_folder/tmp/person_detection.json')
        
        self.get_logger().info(f'Simple follower node initialized. Watching file: {self.detection_file_path}')
        
        # Verify file exists at startup
        if self.detection_file_path.exists():
            self.get_logger().info('Detection file found successfully!')
        else:
            self.get_logger().error(f'Detection file not found at startup. Please check path: {self.detection_file_path}')

    def timer_callback(self):
        try:
            # Check file existence
            if not self.detection_file_path.exists():
                self.get_logger().warn(f'Detection file not found at: {self.detection_file_path}')
                self.stop_robot()
                return
                
            # Read detection data from file
            with open(self.detection_file_path, 'r') as f:
                data = json.load(f)
            
            # Log raw data occasionally
            self.get_logger().info(f'Detection data: {data}')
            
            msg = Twist()
            
            # Check if person is detected with valid distance
            if data['detected'] and data['distance'] is not None:
                # Calculate linear velocity
                distance_error = data['distance'] - self.target_distance
                
                if abs(distance_error) > 0.1:  # 10cm deadband
                    linear_speed = self.max_linear_speed * min(abs(distance_error), 1.0)
                    msg.linear.x = linear_speed if distance_error > 0 else -linear_speed
                
                # Calculate angular velocity based on person position
                if abs(data['x_center']) > 0.1:
                    msg.angular.z = -self.max_angular_speed * data['x_center']
                
            else:
                # No person detected, use search_command if available
                if 'search_command' in data:
                    # search_command ranges from -1 to 1
                    search_velocity = data['search_command'] * self.search_angular_speed
                    msg.angular.z = search_velocity
                    self.get_logger().info(f'Searching with angular velocity: {search_velocity:.2f}')
                else:
                    self.get_logger().info('No valid detection and no search command, stopping robot')
                    msg.angular.z = 0.0
                
                # Stop forward motion when searching
                msg.linear.x = 0.0
            
            # Publish command
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
            )
                
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
            self.stop_robot()

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
