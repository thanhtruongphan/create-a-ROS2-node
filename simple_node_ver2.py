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
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.search_angular_speed = 1.0  # rad/s for search mode
        
        # Acceleration limits (units per second)
        self.max_linear_accel = 0.5  # m/s²
        self.max_angular_accel = 1.0  # rad/s²
        
        # Current velocity state
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_update_time = time.time()
        
        # Smoothing parameters
        self.velocity_smoothing = 0.3  # Lower = smoother but slower response
        
        # Update file path to use absolute path in Docker container
        self.detection_file_path = Path('/root/shared_folder/person_detection.json')
        
        self.get_logger().info(f'Simple follower node initialized. Watching file: {self.detection_file_path}')
        
        # Verify file exists at startup
        if self.detection_file_path.exists():
            self.get_logger().info('Detection file found successfully!')
        else:
            self.get_logger().error(f'Detection file not found at startup. Please check path: {self.detection_file_path}')

    def apply_acceleration_limits(self, target_vel, current_vel, max_accel, dt):
        """Apply acceleration limits to velocity changes"""
        max_vel_change = max_accel * dt
        return current_vel + max(min(target_vel - current_vel, max_vel_change), -max_vel_change)

    def smooth_velocity(self, target_vel, current_vel):
        """Apply exponential smoothing to velocity changes"""
        return current_vel + self.velocity_smoothing * (target_vel - current_vel)

    def timer_callback(self):
        try:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time

            # Check file existence
            if not self.detection_file_path.exists():
                self.get_logger().warn(f'Detection file not found at: {self.detection_file_path}')
                self.stop_robot()
                return
                
            # Read detection data from file
            with open(self.detection_file_path, 'r') as f:
                data = json.load(f)
            
            # Calculate target velocities
            target_linear_vel = 0.0
            target_angular_vel = 0.0
            
            # Check if person is detected with valid distance
            if data['detected'] and data['distance'] is not None:
                # Calculate linear velocity
                distance_error = data['distance'] - self.target_distance
                
                if abs(distance_error) > 0.1:  # 10cm deadband
                    target_linear_vel = self.max_linear_speed * min(abs(distance_error), 1.0)
                    target_linear_vel = target_linear_vel if distance_error > 0 else -target_linear_vel
                
                # Calculate angular velocity based on person position
                if abs(data['x_center']) > 0.1:
                    target_angular_vel = -self.max_angular_speed * data['x_center']
                
            else:
                # No person detected, use search_command if available
                if 'search_command' in data:
                    target_angular_vel = data['search_command'] * self.search_angular_speed
                
            # Apply smoothing and acceleration limits
            smooth_linear_vel = self.smooth_velocity(target_linear_vel, self.current_linear_vel)
            smooth_angular_vel = self.smooth_velocity(target_angular_vel, self.current_angular_vel)
            
            # Apply acceleration limits
            self.current_linear_vel = self.apply_acceleration_limits(
                smooth_linear_vel, 
                self.current_linear_vel, 
                self.max_linear_accel, 
                dt
            )
            self.current_angular_vel = self.apply_acceleration_limits(
                smooth_angular_vel, 
                self.current_angular_vel, 
                self.max_angular_accel, 
                dt
            )
            
            # Create and publish command
            msg = Twist()
            msg.linear.x = self.current_linear_vel
            msg.angular.z = self.current_angular_vel
            self.publisher_.publish(msg)
            
            self.get_logger().info(
                f'cmd_vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
            )
                
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
            self.stop_robot()

    def stop_robot(self):
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
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
