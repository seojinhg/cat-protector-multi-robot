import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from ultralytics import YOLO
from cv_bridge import CvBridge
from std_msgs.msg import String, Float64
import numpy as np
import math
import cv2
import threading
import time
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from message_filters import Subscriber, ApproximateTimeSynchronizer

class YoloNavTracker(Node):
    def __init__(self):
        super().__init__('yolo_nav_tracker')

        # Model path set as ROS2 parameter for environment flexibility
        self.declare_parameter('model_path', '/home/rokey/Downloads/0630_cat/best.pt')
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        self.target_class = "cat"

        # Initialize state variables
        self.bridge = CvBridge()
        self.K = None
        self.depth_image = None
        self.camera_frame = None
        self.tf_ready = False
        self.camera_ready_flag = False
        self.lock = threading.Lock()

        self.last_goal_time = self.get_clock().now()
        # Tuned values from real robot testing:
        # goal_interval=0.3s — prevents Nav2 overload and motion instability from too-frequent goal updates
        # If too short → robot wobbles by constantly switching targets
        # If too long → robot loses track of the cat
        self.goal_interval = Duration(seconds=0.3)
        # min_goal_distance=0.3m — ignores small cat movements to reduce unnecessary robot motion
        # Only sends new goal if cat moved more than 0.3m from last goal position
        self.min_goal_distance = 0.3
        self.last_position = None

        self.frame_count = 0

        # Set topic names based on namespace
        ns = self.get_namespace().rstrip('/')
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.info_topic = f'{ns}/oakd/stereo/camera_info'
        # YOLO model for object detection
        # OpenCV visualization (bounding box, class name, center point)
        # Extract depth value at detected object center (cx, cy) -> 3D position calculation

        # Synchronized subscribers for RGB, Depth, CameraInfo
        self.rgb_sub = Subscriber(self, CompressedImage, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.info_sub = Subscriber(self, CameraInfo, self.info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1  # Time tolerance in seconds
        )
        # Publisher for 2D position
        self.pos_pub = self.create_publisher(
            Float64,
            'pose_xy',
            10
        )
        self.sync.registerCallback(self.synced_callback)

        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = TurtleBot4Navigator()
        # Fixed initial pose based on self-generated SLAM map coordinate
        # Future improvement: replace with AMCL auto-localization for dynamic environments
        initial_pose = self.navigator.getPoseStamped([0.982, -0.16], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        time.sleep(2.0)
        self.navigator.waitUntilNav2Active()

        self.get_logger().info("Waiting 5 seconds for TF tree to stabilize...")
        self.create_timer(5.0, self.enable_tf)

    # Synchronized callback for RGB + Depth + CameraInfo
    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        if not self.tf_ready:
            return

        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self.get_logger().info(f"RGB image size: {frame.shape}")
            self.get_logger().info(f"Depth image size: {depth.shape}")

        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        with self.lock:
            self.K = np.array(info_msg.k).reshape(3, 3)
            self.depth_image = depth
            self.camera_frame = info_msg.header.frame_id
         
        # YOLO tracking with persist=True to maintain track IDs across frames
        for track in self.model.track(
            frame,
            stream=True,
            conf=0.5,
            persist=True
        ):
            boxes = track.boxes

            if boxes.id is None:
                self.get_logger().warn("Track ID is None. Skipping this detection.")
                continue

            # Extract track ID
            track_id = int(boxes.id[0])

            # Extract class, confidence, bounding box coordinates
            cls_id = int(boxes.cls[0])
            conf = float(boxes.conf[0])
            x1, y1, x2, y2 = map(int, boxes.xyxy[0])

            # Filter by target class only
            if self.model.names[cls_id] != self.target_class:
                continue

            # Calculate center coordinates and process movement
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            self.process_target_center(cx, cy, track_id, rgb_msg.header.stamp)

            # Track only the first detected object
            break

    def enable_tf(self):
        if not self.tf_ready:
            self.tf_ready = True
            self.get_logger().info("TF tree ready. Starting object tracking...")

    def check_camera_ready(self):
        if self.K is not None and self.depth_image is not None and not self.camera_ready_flag:
            self.camera_ready_flag = True
            self.get_logger().info("Camera info and depth image received. System ready.")

    def camera_info_callback(self, msg):
        # Store camera intrinsic matrix K for depth calculation
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info("Camera intrinsics received.")
        self.check_camera_ready()

    def depth_callback(self, msg):
        # Convert depth ROS message to OpenCV image
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.lock:
                self.depth_image = depth
            self.check_camera_ready()
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    # image_callback is disabled in favor of synced_callback
    # synced_callback provides better RGB/Depth synchronization and YOLO tracking with Track ID
    # def image_callback(self, msg):
    #     # Convert compressed RGB ROS message to OpenCV image
    #     if not self.camera_ready_flag:
    #         self.get_logger().warn("Camera not ready, skipping image frame.")
    #         return

    #     if not self.tf_ready:
    #         return

    #     try:
    #         frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #     except Exception as e:
    #         self.get_logger().error(f"Image conversion failed: {e}")
    #         return

    #     results = self.model.predict(frame, verbose=False)[0]
    #     boxes = results.boxes

    #     if not boxes:
    #         return

    #     # Use YOLO model to detect objects
    #     for box in boxes:
    #         # Filter by target class
    #         cls_id = int(box.cls[0])
    #         cls_name = self.model.names[cls_id]

    #         # Calculate bounding box center coordinates
    #         if cls_name == self.target_class:
    #             x1, y1, x2, y2 = map(int, box.xyxy[0])
    #             cx = (x1 + x2) // 2
    #             cy = (y1 + y2) // 2

    #             # Move to detected center position
    #             self.process_target_center(cx, cy)
    #             break

    def process_target_center(self, x, y, track_id=None, header_stamp=None):
        # Extract average depth Z value at center coordinate (x, y)
        # Use camera intrinsics K to compute (X, Y, Z) 3D coordinates
        # Transform from camera_frame to map frame via TF
        # Ignore if position is too close to last goal
        # Send goToPose() command via Nav2
        # Publish RViz Marker to visualize target position
        with self.lock:
            depth = self.depth_image.copy() if self.depth_image is not None else None
            K = self.K.copy() if self.K is not None else None
            frame_id = self.camera_frame

        if depth is None or K is None:
            self.get_logger().warn("Missing depth or intrinsics.")
            return

        if not (0 <= x < depth.shape[1] and 0 <= y < depth.shape[0]):
            self.get_logger().warn(f"Invalid pixel coordinate: ({x}, {y}) out of image bounds.")
            return

        # Average depth over 3x3 region around center pixel (mm -> m)
        z = np.nanmean(depth[y-1:y+2, x-1:x+2]) / 1000.0
        if z == 0.0 or np.isnan(z) or z <= 0.2 or z >= 5.0:
            self.get_logger().warn(f"Invalid depth at ({x}, {y}): {z:.2f}m")
            return

        # Compute 3D position using camera intrinsics and depth
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        Z = z

        pt_camera = PointStamped()
        # Represents a 3D point in a specific coordinate frame
        # Used for TF transform (e.g. do_transform_point to another frame)
        # Used to specify target position for robot
        # Represents sensor-detected position
        # pt_camera.header.stamp must be set correctly for accurate coordinate transform

        self.get_logger().info(f"{header_stamp}")
        if header_stamp is None:
            pt_camera.header.stamp = self.get_clock().now().to_msg()
        else:
            pt_camera.header.stamp = header_stamp

        pt_camera.header.frame_id = frame_id
        pt_camera.point.x = X
        pt_camera.point.y = Y
        pt_camera.point.z = Z

        if not self.tf_buffer.can_transform('map', frame_id, Time(), timeout=Duration(seconds=1.0)):
            self.get_logger().warn("TF transform not ready.")
            return

        try:
            # Transform 3D point from camera frame to map frame
            transform = self.tf_buffer.lookup_transform('map', frame_id, Time(), timeout=Duration(seconds=1.0))
            pt_map = do_transform_point(pt_camera, transform)

            now = self.get_clock().now()
            if now - self.last_goal_time < self.goal_interval:
                return

            if self.last_position:
                dx = pt_map.point.x - self.last_position[0]
                dy = pt_map.point.y - self.last_position[1]

                # Ignore if target is too close to last position
                if math.hypot(dx, dy) < self.min_goal_distance:
                    return
            
            # Get robot's current orientation from the transform
            robot_orientation = transform.transform.rotation

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pt_map.point.x
            goal_pose.pose.position.y = pt_map.point.y
            # Keep current robot orientation (do not force north)
            goal_pose.pose.orientation = robot_orientation

            # Send navigation goal — only update if moved more than min_goal_distance
            self.navigator.goToPose(goal_pose)
            self.last_goal_time = now
            self.last_position = (pt_map.point.x, pt_map.point.y)
            self.get_logger().info(f"Navigating to: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")

            # FIX 2: Removed invalid .type attribute calls on tuple and float
            # self.last_position.type -> invalid (tuple has no .type)
            # pt_map.point.x.type -> invalid (float has no .type)

            # FIX 3: last_positions -> last_position (correct variable name)
            msg_pub = Float64()
            msg_pub.data = float(self.last_position[0])  # publish x coordinate as example
            # FIX 4: pos_pub.publisheri -> pos_pub.publish
            self.pos_pub.publish(msg_pub)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goal_marker"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = goal_pose.pose.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2

            # Publish marker to visualize target position in RViz
            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f"TF transform error: {e}")


def main():
    rclpy.init()
    node = YoloNavTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Summary of pipeline:
# 1. Receive RGB + Depth + CameraInfo -> system ready
# 2. Detect target class via YOLO -> get bounding box
# 3. Compute depth at center coordinate (cx, cy)
# 4. Use camera intrinsics + TF to extract 3D map coordinates
# 5. Send navigation command via Nav2
# 6. Visualize detection with OpenCV, show marker in RViz
