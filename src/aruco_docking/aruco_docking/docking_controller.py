import cv2
import numpy as np
import rclpy
import signal
import sys
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge
from interfaces.msg import MarkerPose


class MarkerFollowerNode(Node):
    def __init__(self):
        super().__init__('marker_follower_node')
        self.get_logger().info("Marker follower node started with 3D pose control...")

        # Subscribe to marker poses from pose estimation node
        self.pose_subscription = self.create_subscription(
            MarkerPose,
            '/marker_poses',
            self.pose_callback,
            10
        )
        
        # Maintain subscription to output image for visualization
        self.image_subscription = self.create_subscription(
            Image,
            '/pose_estimation/output_image',
            self.image_callback,
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # Target pose parameters (in meters)
        self.target_x = 0.0        # Target x position (centered laterally)
        self.target_y = 0.0        # Target y position (centered vertically)
        self.target_z = 0.5        # Target distance from camera
        
        # PID controller parameters (tuned for meter-based errors)
        self.x_pid = PIDController(kp=1.0, ki=0.05, kd=0.1)  # For lateral (x) positioning
        self.y_pid = PIDController(kp=0.5, ki=0.02, kd=0.05) # For vertical (y) positioning - lower gain as most robots can't move vertically
        self.z_pid = PIDController(kp=0.8, ki=0.05, kd=0.1)  # For distance (z) control

        # Robot movement parameters
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.3

        # Search mode parameters
        self.search_mode = False
        self.last_marker_time = self.get_clock().now()
        self.marker_timeout = 5.0
        self.marker_found = False
        self.was_tracking = False
        
        # Current marker pose data
        self.current_tvec = None
        self.current_rvec = None
        self.current_marker_id = None
        
        # Timer for handling marker timeout
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Store frame for visualization
        self.current_frame = None

    def pose_callback(self, msg):
        # Extract pose data from the MarkerPose message
        self.current_marker_id = msg.marker_id
        
        # Extract translation vector
        tvec_x = msg.translation.x
        tvec_y = msg.translation.y
        tvec_z = msg.translation.z
        self.current_tvec = np.array([tvec_x, tvec_y, tvec_z])
        
        # Extract rotation vector
        rvec_x = msg.rotation.x
        rvec_y = msg.rotation.y
        rvec_z = msg.rotation.z
        self.current_rvec = np.array([rvec_x, rvec_y, rvec_z])
        
        # Update tracking state
        if self.search_mode:
            self.get_logger().info("Marker found after searching")
            self.x_pid.reset()
            self.y_pid.reset()
            self.z_pid.reset()
            self.search_mode = False
            
        self.last_marker_time = self.get_clock().now()
        self.marker_found = True
        self.was_tracking = True
        
        # Calculate errors in meters
        x_error = self.target_x - tvec_x  # Lateral error
        y_error = self.target_y - tvec_y  # Vertical error 
        z_error = self.target_z - tvec_z  # Distance error
        
        # Update PID controllers
        x_correction = self.x_pid.update(x_error)
        y_correction = self.y_pid.update(y_error)
        z_correction = self.z_pid.update(z_error)
        
        # Create and fill Twist message
        twist = Twist()
        
        # Map PID outputs to robot movements:
        # - Use z_correction for forward/backward movement (linear.x)
        # - Use x_correction for rotation (angular.z)
        # - y_correction could be used for vertical movement if supported
        twist.linear.x = z_correction  # Forward/backward based on distance
        twist.angular.z = x_correction  # Rotation based on lateral position
        
        # Apply speed limits
        twist.linear.x = max(min(twist.linear.x, self.max_linear_speed), -self.max_linear_speed)
        twist.angular.z = max(min(twist.angular.z, self.max_angular_speed), -self.max_angular_speed)
        
        # Apply deadband to avoid small oscillations
        twist.linear.x = 0.0 if abs(twist.linear.x) < 0.01 else twist.linear.x
        twist.angular.z = 0.0 if abs(twist.angular.z) < 0.01 else twist.angular.z
        
        # Log information
        self.get_logger().info(f"Marker ID: {self.current_marker_id}")
        self.get_logger().info(f"Position (meters): X:{tvec_x:.4f}, Y:{tvec_y:.4f}, Z:{tvec_z:.4f}")
        self.get_logger().info(f"Errors (meters): X:{x_error:.4f}, Y:{y_error:.4f}, Z:{z_error:.4f}")
        self.get_logger().info(f"Commands: LinearX:{twist.linear.x:.3f}, AngularZ:{twist.angular.z:.3f}")
        
        # Publish control commands
        self.cmd_vel_publisher.publish(twist)

    def image_callback(self, msg):
        # Just store the frame for visualization
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Add status text to image
        if self.current_frame is not None:
            status_text = "Searching" if self.search_mode else "Tracking" if self.marker_found and not self.search_mode else "Waiting"
            cv2.putText(self.current_frame, f"Status: {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # Add pose information if available
            if self.current_tvec is not None:
                pos_text = f"Pos: X:{self.current_tvec[0]:.2f}, Y:{self.current_tvec[1]:.2f}, Z:{self.current_tvec[2]:.2f}m"
                cv2.putText(self.current_frame, pos_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Calculate errors
                x_error = self.target_x - self.current_tvec[0]
                y_error = self.target_y - self.current_tvec[1]
                z_error = self.target_z - self.current_tvec[2]
                err_text = f"Err: X:{x_error:.2f}, Y:{y_error:.2f}, Z:{z_error:.2f}m"
                cv2.putText(self.current_frame, err_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow("Marker Follower", self.current_frame)
            cv2.waitKey(1)

    def timer_callback(self):
        # Check if marker detection has timed out
        if self.marker_found:
            time_since_last_marker = (self.get_clock().now() - self.last_marker_time).nanoseconds / 1e9
            
            if time_since_last_marker > self.marker_timeout:
                if not self.search_mode:
                    self.get_logger().warning(f"No marker detected for {self.marker_timeout} seconds - starting search")
                    self.search_mode = True
                
                # Create search pattern (rotate in place)
                twist = Twist()
                twist.angular.z = 0.1
                twist.linear.x = 0.0
                self.cmd_vel_publisher.publish(twist)
            
        # If no marker has ever been detected, publish zero velocity
        if not self.marker_found and not self.search_mode:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)

    def destroy_node(self):
        self.get_logger().info("Shutting down marker follower node...")
        stop_twist = Twist()
        for _ in range(3):  # Send stop command multiple times to ensure it's received
            self.cmd_vel_publisher.publish(stop_twist)
            time.sleep(0.1)
        cv2.destroyAllWindows()
        self.get_logger().info("Node shutdown complete")
        super().destroy_node()


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0
        self.integral = 0

    def update(self, error):
        # Calculate P term
        p_term = self.kp * error

        # Calculate I term with windup prevention
        self.integral = max(min(self.integral + error, 1.0), -1.0)  # Limit for meter-based control
        i_term = self.ki * self.integral

        # Calculate D term
        d_term = self.kd * (error - self.previous_error)
        self.previous_error = error

        return p_term + i_term + d_term

    def reset(self):
        self.previous_error = 0
        self.integral = 0


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollowerNode()

    def signal_handler(sig, frame):
        node.get_logger().info("Received termination signal")
        stop_twist = Twist()
        for _ in range(3):
            node.cmd_vel_publisher.publish(stop_twist)
            time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected")
    finally:
        stop_twist = Twist()
        for _ in range(3):
            node.cmd_vel_publisher.publish(stop_twist)
            time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()