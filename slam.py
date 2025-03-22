import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from laser_geometry import LaserProjection
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
import threading
from scipy.spatial import cKDTree

class PauseAndCapture(Node):
    def __init__(self):
        super().__init__('pause_and_capture')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.laser_projector = LaserProjection()

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile)

        self.pc_pub = self.create_publisher(PointCloud2, '/accumulated_cloud', 10)
        self.icp_pub = self.create_publisher(PointCloud2, '/icp_merged_cloud', 10)

        self.accumulated_points = []
        self.icp_accumulated_points = []

        self.capture_enabled = False
        self.latest_scan = None
        self.delay_timer = None

        self.input_thread = threading.Thread(target=self.key_press_listener, daemon=True)
        self.input_thread.start()

        self.get_logger().info("PauseAndCapture node started. Press Enter to capture a scan.")

    def key_press_listener(self):
        while True:
            input(">> Press Enter to capture scan: ")
            self.capture_enabled = True

    def scan_callback(self, scan_msg):
        if not self.capture_enabled:
            return

        self.latest_scan = scan_msg
        self.capture_enabled = False

        if self.delay_timer:
            self.delay_timer.cancel()
        self.delay_timer = self.create_timer(0.1, self.delayed_transform_lookup)

    def delayed_transform_lookup(self):
        self.delay_timer.cancel()
        scan_msg = self.latest_scan
        self.latest_scan = None

        try:
            cloud_in_laser = self.laser_projector.projectLaser(scan_msg)

            #TODO:
            # Perform a lookup to transform the point cloud from its original frame to the 'odom' frame
            transform = self.tf_buffer.lookup_transform(
                ..., # Target frame
                ... ,# Source frame (the point cloud's original frame)
                ...,  # Timestamp of the scan message to ensure proper time synchronization
                timeout=rclpy.duration.Duration(seconds=0.5)  # Timeout of 0.5 seconds to wait for the transform
            )

            transformed_points = self.transform_pointcloud2(cloud_in_laser, transform)

            if self.icp_accumulated_points:
                icp_aligned = self.perform_icp(transformed_points, self.icp_accumulated_points)
                self.icp_accumulated_points.extend(icp_aligned)
                self.publish_icp_merged_cloud(scan_msg.header.stamp)
                self.get_logger().info(f"ICP-aligned and merged {len(icp_aligned)} points.")
            else:
                self.icp_accumulated_points.extend(transformed_points)
                self.publish_icp_merged_cloud(scan_msg.header.stamp)
                self.get_logger().info(f"Initialized ICP merged cloud with {len(transformed_points)} points.")

            self.accumulated_points.extend(transformed_points)
            self.publish_accumulated_cloud(scan_msg.header.stamp)
            self.get_logger().info(f"Captured and transformed {len(transformed_points)} points.")

        except TransformException as ex:
            self.get_logger().warn(f"Transform failed after delay: {str(ex)}")




    #TODO: Complete the rotate_point_euler in transform_pointcloud2 functios


    def transform_pointcloud2(self, cloud_msg, transform):
        """Transform a point cloud using Euler angles from a given quaternion."""
        
        # Helper function to convert quaternion to Euler angles (roll, pitch, yaw)
        def quaternion_to_euler(q):
            """Convert quaternion to Euler angles (roll, pitch, yaw)."""
            x, y, z, w = q.x, q.y, q.z, q.w
            roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            pitch = math.asin(2.0 * (w * y - z * x))
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            return [roll, pitch, yaw]

        def rotate_point_euler(x, y, z, roll, pitch, yaw):
            """Rotate a point (x, y, z) using Euler angles (roll, pitch, yaw)."""

            #TODO:
            #using the roll,pitch and yaw construct the Rx , Ry, Rz matrix


            # Roll (rotation around X-axis)
            Rx = ...

            # Pitch (rotation around Y-axis)
            Ry = ...

            # Yaw (rotation around Z-axis)
            Rz = ...

            #TODO:
            # Combined rotation matrix (first yaw, then pitch, then roll)
            R = ...

            #TODO:
            # Apply the rotation to the point
            rotated_point = ...

            return rotated_point[0], rotated_point[1], rotated_point[2]
        


        # Extract translation and rotation (quaternion) from the transform
        t = transform.transform.translation
        q = transform.transform.rotation

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = quaternion_to_euler(q)

        # Transform the point cloud using Euler rotation
        transformed_points = []
        for pt in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = pt
            
            #TODO:
            # Apply rotation to the point using Euler angles use the rotate point euler function
            rx, ry, rz = ...
            
            #TODO:
            # Apply translation to the rotated point using the variavble t
            rx += ...
            ry += ...
            rz += ...
            
            # Append transformed point
            transformed_points.append((rx, ry, rz))

        return transformed_points

    def perform_icp(self, source_points, target_points, max_iterations=20, tolerance=1e-4):
        src = np.array(source_points)
        tgt = np.array(target_points)

        # prev_error = float('inf')
        ERROR = []
        prev_error = float(10000.0)
        counter_ = 0
        for _ in range(max_iterations):
            tree = cKDTree(tgt)
            distances, indices = tree.query(src)
            matched_tgt = tgt[indices]


            R,t = self.svd_estimation(matched_tgt.T,src.T)


            src = (R @ src.T).T + t

            mean_error = np.mean(distances)
            print('iterations and error: ', counter_, mean_error)
            if abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error
            ERROR.append(prev_error)
            counter_ += 1
        print(ERROR)

        return src.tolist()
    

    #TODO: copy your svd_estimation function from hw4 A

    def svd_estimation(self, previous_points, current_points):
        #TODO;
        pass


    def publish_accumulated_cloud(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, self.accumulated_points)
        self.pc_pub.publish(cloud_msg)
        self.get_logger().info("Published accumulated cloud.")

    def publish_icp_merged_cloud(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, self.icp_accumulated_points)
        self.icp_pub.publish(cloud_msg)
        self.get_logger().info("Published ICP merged cloud.")

def main(args=None):
    rclpy.init(args=args)
    node = PauseAndCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()