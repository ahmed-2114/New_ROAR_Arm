#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class ReachabilityMap(Node):
    def __init__(self):
        super().__init__('reachability_mapper')
        
        # 1. Setup Service Client & Publisher
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.marker_pub = self.create_publisher(MarkerArray, 'reachability_markers', 10)
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt compute_ik service...')

        # 2. Configuration (Match your SRDF/URDF)
        self.group_name = "arm_controller"
        self.base_frame = "base_link"
        
        # 3. Define Workspace Scan Volume (in Meters)
        # Adjust these limits based on your robot's physical reach
        self.x_min, self.x_max = -0.8, 0.8
        self.y_min, self.y_max = -0.8, 0.8
        self.z_min, self.z_max =  0.0, 1.0
        self.resolution = 0.1  # 10cm voxel size

    def generate(self):
        self.get_logger().info("Starting Reachability Scan...")
        marker_array = MarkerArray()
        id_counter = 0
        
        # Create grid
        x_range = np.arange(self.x_min, self.x_max, self.resolution)
        y_range = np.arange(self.y_min, self.y_max, self.resolution)
        z_range = np.arange(self.z_min, self.z_max, self.resolution)
        
        total = len(x_range) * len(y_range) * len(z_range)
        count = 0

        for x in x_range:
            for y in y_range:
                for z in z_range:
                    # Construct IK Request
                    req = GetPositionIK.Request()
                    req.ik_request.group_name = self.group_name
                    req.ik_request.robot_state = RobotState() # Use default/current state
                    req.ik_request.avoid_collisions = True
                    req.ik_request.timeout.sec = 0
                    req.ik_request.timeout.nanosec = 50000000 # 0.05s timeout
                    
                    # Define Target Pose
                    ps = PoseStamped()
                    ps.header.frame_id = self.base_frame
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.position.z = z
                    # Orientation: Pointing FORWARD (modify if your tool points Z-up)
                    ps.pose.orientation.w = 1.0 
                    
                    req.ik_request.pose_stamped = ps

                    # Call Service
                    future = self.ik_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    result = future.result()

                    # Process Result
                    if result.error_code.val == 1: # SUCCESS
                        marker = Marker()
                        marker.header.frame_id = self.base_frame
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.id = id_counter
                        marker.pose = ps.pose
                        marker.scale.x = self.resolution * 0.6
                        marker.scale.y = self.resolution * 0.6
                        marker.scale.z = self.resolution * 0.6
                        marker.color.a = 0.6
                        marker.color.g = 1.0 # Green
                        
                        marker_array.markers.append(marker)
                        id_counter += 1
                    
                    count += 1
                    if count % 100 == 0:
                        self.get_logger().info(f"Scanned {count}/{total} voxels...")

        self.get_logger().info(f"Scan Complete. Publishing {len(marker_array.markers)} reachable points.")
        
        # Publish Loop
        rate = self.create_rate(1)
        while rclpy.ok():
            self.marker_pub.publish(marker_array)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityMap()
    node.generate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()