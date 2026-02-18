import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class ReachabilityGenerator(Node):
    def __init__(self):
        super().__init__('reachability_generator')
        
        # 1. Setup Clients
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.marker_pub = self.create_publisher(MarkerArray, 'reachability_markers', 10)
        
        # Wait for service
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for compute_ik service...')

        # 2. Configuration for New_ROAR_Arm
        self.group_name = "arm_controller"  # From your SRDF
        self.base_frame = "base_link"       # From your URDF
        
        # Workspace settings (meters)
        self.resolution = 0.1  # 10cm voxels (decrease to 0.05 for higher detail)
        self.x_range = np.arange(-0.8, 0.8, self.resolution)
        self.y_range = np.arange(-0.8, 0.8, self.resolution)
        self.z_range = np.arange(0.0, 1.0, self.resolution)

    def generate_map(self):
        self.get_logger().info("Starting map generation...")
        marker_array = MarkerArray()
        id_counter = 0
        
        total_points = len(self.x_range) * len(self.y_range) * len(self.z_range)
        processed = 0

        for x in self.x_range:
            for y in self.y_range:
                for z in self.z_range:
                    processed += 1
                    if processed % 100 == 0:
                        self.get_logger().info(f"Processed {processed}/{total_points}")

                    # 3. Create IK Request
                    req = GetPositionIK.Request()
                    req.ik_request.group_name = self.group_name
                    req.ik_request.robot_state = RobotState() # Empty = current state
                    req.ik_request.avoid_collisions = True
                    
                    # Target Pose
                    ps = PoseStamped()
                    ps.header.frame_id = self.base_frame
                    ps.pose.position.x = x
                    ps.pose.position.y = y
                    ps.pose.position.z = z
                    # Orientation: Facing forward/down (standard grasp)
                    ps.pose.orientation.w = 1.0 
                    
                    req.ik_request.pose_stamped = ps
                    req.ik_request.timeout.sec = 0
                    req.ik_request.timeout.nanosec = 50000000 # 0.05s timeout

                    # 4. Call Service (Synchronous for simplicity)
                    future = self.ik_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    response = future.result()

                    # 5. Create Marker based on result
                    if response.error_code.val == 1: # SUCCESS
                        marker = Marker()
                        marker.header.frame_id = self.base_frame
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.id = id_counter
                        id_counter += 1
                        
                        marker.scale.x = self.resolution * 0.8
                        marker.scale.y = self.resolution * 0.8
                        marker.scale.z = self.resolution * 0.8
                        
                        marker.color.a = 0.5
                        marker.color.g = 1.0 # Green for reachable
                        
                        marker.pose = ps.pose
                        marker_array.markers.append(marker)
        
        # Publish all at once
        self.get_logger().info(f"Publishing {len(marker_array.markers)} reachable points")
        while rclpy.ok():
            self.marker_pub.publish(marker_array)
            rclpy.spin_once(self, timeout_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityGenerator()
    node.generate_map()
    rclpy.shutdown()

if __name__ == '__main__':
    main()