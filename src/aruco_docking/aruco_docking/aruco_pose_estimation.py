import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from interfaces.msg import MarkerPose


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        self.get_logger().info("Pose estimation node started...")

        # ROS2 subscription and publishers
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Keep the image publisher for visualization
        self.image_publisher = self.create_publisher(Image, '/pose_estimation/output_image', 10)
        # Add a new publisher for marker pose data
        self.pose_publisher = self.create_publisher(MarkerPose, '/marker_poses', 10)
        self.bridge = CvBridge()

        # ArUco configuration
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters()

        # Camera intrinsic parameters
        self.intrinsic_camera = np.array([
            [933.15867, 0, 657.59],
            [0, 933.1586, 400.36993],
            [0, 0, 1]
        ])

        # Camera distortion coefficients
        self.distortion = np.array([-0.43948, 0.18514, 0, 0])

        # Marker size (in meters)
        self.marker_size = 0.02
        self.obj_points = np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
        ], dtype=np.float32)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        output = self.pose_estimation(frame)
        output_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
        self.image_publisher.publish(output_msg)

    def pose_estimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                img_points = corners[i][0]
                success, rvec, tvec = cv2.solvePnP(
                    self.obj_points,
                    img_points,
                    self.intrinsic_camera,
                    self.distortion
                )

                if success:
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.drawFrameAxes(frame, self.intrinsic_camera, self.distortion, rvec, tvec, 0.01)
                    
                    # Log the pose information
                    self.get_logger().info(f"Marker ID: {ids[i]}")
                    self.get_logger().info(
                        f"Translation (tvec): x={tvec[0][0]:.4f}, y={tvec[1][0]:.4f}, z={tvec[2][0]:.4f} meters"
                    )
                    self.get_logger().info(
                        f"Rotation (rvec): x={rvec[0][0]:.4f}, y={rvec[1][0]:.4f}, z={rvec[2][0]:.4f} radians"
                    )
                    
                    # Publish marker pose
                    marker_pose = MarkerPose()
                    marker_pose.marker_id = int(ids[i][0])
                    
                    # Fill translation vector
                    translation = Vector3()
                    translation.x = float(tvec[0][0])
                    translation.y = float(tvec[1][0])
                    translation.z = float(tvec[2][0])
                    marker_pose.translation = translation
                    
                    # Fill rotation vector
                    rotation = Vector3()
                    rotation.x = float(rvec[0][0])
                    rotation.y = float(rvec[1][0])
                    rotation.z = float(rvec[2][0])
                    marker_pose.rotation = rotation
                    
                    # Publish the pose message
                    self.pose_publisher.publish(marker_pose)
                else:
                    self.get_logger().warning(f"Pose estimation failed for Marker ID: {ids[i]}")

        return frame

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()