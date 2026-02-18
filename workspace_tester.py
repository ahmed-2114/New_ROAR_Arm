#!/usr/bin/env python3
"""
Fixed Workspace Tester - Properly seeds IK requests
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
import math
from rclpy.parameter import Parameter
import time

GROUP_NAME = "arm_controller"
BASE_FRAME = "base_link"
TARGET_LINK = "Link_5"

class WorkspaceTester(Node):
    def __init__(self):
        super().__init__('workspace_tester',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.marker_pub = self.create_publisher(MarkerArray, '/workspace_markers', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.current_joint_state = None
        
        self.get_logger().info("🔍 Workspace Tester Starting...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK service not available!")
            return
        
        # Wait for joint states
        self.get_logger().info("⏳ Waiting for joint states...")
        start = time.time()
        while self.current_joint_state is None and time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_joint_state is None:
            self.get_logger().error("❌ No joint states received! Is the robot running?")
            return
            
        self.get_logger().info("✅ IK service ready and joint states received")
    
    def joint_state_callback(self, msg):
        """Store latest joint state for IK seeding"""
        self.current_joint_state = msg
    
    def get_seed_state(self):
        """Get current robot state for IK seeding"""
        if self.current_joint_state is None:
            # Try to get joint state
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.current_joint_state is not None:
                    break
        
        if self.current_joint_state is None:
            # Return a default safe configuration
            seed = RobotState()
            seed.joint_state.name = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_EE']
            seed.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            return seed
        
        # Use actual current state
        seed = RobotState()
        seed.joint_state = self.current_joint_state
        return seed
    
    def test_position(self, x, y, z, timeout=1.0):
        """Test if a position is reachable with proper seeding"""
        # Refresh joint state
        rclpy.spin_once(self, timeout_sec=0.01)
        
        # Try multiple orientations
        orientations = [
            (1.0, 0.0, 0.0, 0.0),  # No rotation
            (0.707, 0.707, 0.0, 0.0),  # 90° roll
            (0.707, 0.0, 0.707, 0.0),  # 90° pitch
            (0.707, 0.0, 0.0, 0.707),  # 90° yaw
            (0.0, 1.0, 0.0, 0.0),  # 180° roll
        ]
        
        for qw, qx, qy, qz in orientations:
            ik_req = PositionIKRequest()
            ik_req.group_name = GROUP_NAME
            ik_req.ik_link_name = TARGET_LINK
            ik_req.pose_stamped.header.frame_id = BASE_FRAME
            ik_req.pose_stamped.pose.position.x = x
            ik_req.pose_stamped.pose.position.y = y
            ik_req.pose_stamped.pose.position.z = z
            ik_req.pose_stamped.pose.orientation.w = qw
            ik_req.pose_stamped.pose.orientation.x = qx
            ik_req.pose_stamped.pose.orientation.y = qy
            ik_req.pose_stamped.pose.orientation.z = qz
            ik_req.timeout.sec = 0
            ik_req.timeout.nanosec = int(timeout * 1e9)
            
            # CRITICAL: Seed with current robot state
            ik_req.robot_state = self.get_seed_state()
            
            req = GetPositionIK.Request()
            req.ik_request = ik_req
            
            try:
                fut = self.ik_client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout + 0.5)
                
                if fut.done() and fut.result() and fut.result().error_code.val == 1:
                    return True
            except Exception as e:
                continue
        
        return False
    
    def quick_test(self):
        """Quick test of common positions"""
        self.get_logger().info("\n🧪 Quick Reachability Test\n")
        
        test_points = [
            ("Current position", None, None, None),  # Special case
            ("Front close", 0.3, 0.0, 0.3),
            ("Front mid", 0.4, 0.0, 0.3),
            ("Front far", 0.6, 0.0, 0.3),
            ("Left side", 0.3, 0.3, 0.3),
            ("Right side", 0.3, -0.3, 0.3),
            ("High up", 0.3, 0.0, 0.6),
            ("Low down", 0.3, 0.0, 0.15),
            ("Behind (near)", -0.1, 0.0, 0.3),
            ("Behind (far)", -0.5, 0.0, 0.3),
        ]
        
        results = []
        for name, x, y, z in test_points:
            if name == "Current position":
                # Always reachable - it's where we are!
                self.get_logger().info(f"✅ {name:25s} (starting configuration)")
                results.append((name, 0, 0, 0, True))
                continue
                
            reachable = self.test_position(x, y, z, timeout=1.5)
            status = "✅" if reachable else "❌"
            self.get_logger().info(f"{status} {name:25s} [{x:+.2f}, {y:+.2f}, {z:+.2f}]")
            results.append((name, x, y, z, reachable))
        
        # Count successes
        successes = sum(1 for _, _, _, _, reachable in results if reachable)
        self.get_logger().info(f"\n📊 Results: {successes}/{len(results)} positions reachable\n")
        
        if successes <= 2:
            self.get_logger().warn("⚠️ Very low success rate! Possible issues:")
            self.get_logger().warn("   1. Robot model might have issues")
            self.get_logger().warn("   2. Joint limits might be too restrictive")
            self.get_logger().warn("   3. Check if robot is in collision")
            return results
        
        # Find reachable limits
        self.get_logger().info("🎯 Finding workspace boundaries...\n")
        
        # Max forward reach
        max_forward = 0.0
        for dist in [i * 0.05 for i in range(1, 20)]:
            if self.test_position(dist, 0.0, 0.3, timeout=1.0):
                max_forward = dist
            else:
                break
        self.get_logger().info(f"Max forward reach (+X): ~{max_forward:.2f}m")
        
        # Max sideways reach
        max_side = 0.0
        for dist in [i * 0.05 for i in range(1, 15)]:
            if self.test_position(0.3, dist, 0.3, timeout=1.0):
                max_side = dist
            else:
                break
        self.get_logger().info(f"Max sideways reach (±Y): ~{max_side:.2f}m")
        
        # Max height
        max_height = 0.0
        for height in [i * 0.05 for i in range(1, 20)]:
            if self.test_position(0.3, 0.0, height, timeout=1.0):
                max_height = height
            else:
                break
        self.get_logger().info(f"Max height (Z): ~{max_height:.2f}m")
        
        # Show recommended safe zone
        self.get_logger().info("\n💡 Recommended Safe Operating Zone:")
        self.get_logger().info(f"   X: 0.15m to {max_forward-0.1:.2f}m")
        self.get_logger().info(f"   Y: -{max_side-0.05:.2f}m to +{max_side-0.05:.2f}m")
        self.get_logger().info(f"   Z: 0.10m to {max_height-0.1:.2f}m")
        
        return results

def main():
    rclpy.init()
    node = WorkspaceTester()
    
    if node.current_joint_state is None:
        print("\n❌ ERROR: Cannot get joint states!")
        print("Make sure your robot is running:")
        print("  ros2 launch ROAR_MoveIT complete.launch.py")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    print("\n" + "="*50)
    print("  5-DOF Workspace Tester (Fixed)")
    print("="*50)
    print("\nOptions:")
    print("  1) Quick test (common positions)")
    print("  2) Test specific position")
    print("\nChoice: ", end='')
    
    choice = input().strip()
    
    if choice == '1':
        node.quick_test()
    elif choice == '2':
        print("\nEnter position to test:")
        x = float(input("  X (forward/back): "))
        y = float(input("  Y (left/right): "))
        z = float(input("  Z (up/down): "))
        
        print(f"\n🔍 Testing position [{x:.2f}, {y:.2f}, {z:.2f}]...")
        reachable = node.test_position(x, y, z, timeout=2.0)
        if reachable:
            print(f"\n✅ Position [{x:.2f}, {y:.2f}, {z:.2f}] is REACHABLE")
        else:
            print(f"\n❌ Position [{x:.2f}, {y:.2f}, {z:.2f}] is UNREACHABLE")
            print("\n💡 Tips:")
            print("   - Try positions in front of the robot (positive X)")
            print("   - Keep distances reasonable (<0.8m)")
            print("   - Avoid positions behind the base (negative X)")
    else:
        print("Invalid choice")
    
    print("\n✅ Done!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
