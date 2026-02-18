#!/usr/bin/env python3
"""
5-DOF XYZ Teleop with Workspace Checking
Prevents unreachable goal commands
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    PositionConstraint,
    BoundingVolume,
    PlanningOptions,
    JointConstraint
)
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
import sys, select, termios, tty
from rclpy.parameter import Parameter
import math

# --- SETTINGS ---
GROUP_NAME = "arm_controller"
BASE_FRAME = "base_link"
TARGET_LINK = "Link_5"
STEP_SIZE = 0.05        # 5cm steps
PLANNING_TIME = 5.0
# ----------------

msg = """
╔════════════════════════════════════════════════╗
║   ROAR 5-DOF XYZ Teleop (Workspace-Aware)      ║
╚════════════════════════════════════════════════╝

Controls:
  w/s : Forward/Backward (+X/-X)  
  a/d : Left/Right      (+Y/-Y)
  q/e : Up/Down         (+Z/-Z)
  
  r   : Return to home position
  p   : Print current position
  x   : Exit

💡 Tip: The arm works best FORWARD of the base!
    Positions behind the robot are usually unreachable.
"""

class SmartXYZMover(Node):
    def __init__(self):
        super().__init__('smart_xyz_mover',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, MoveGroup, 'move_action')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        self.get_logger().info("🔧 Connecting to MoveIt...")
        self.action_client.wait_for_server()
        self.ik_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("✅ Connected! Smart XYZ mode active.")
        
        # Workspace limits (empirical - adjust for your robot)
        self.workspace = {
            'x_min': -0.2,   # Don't go far behind base
            'x_max': 0.8,    # Max forward reach
            'y_min': -0.5,   # Max left
            'y_max': 0.5,    # Max right
            'z_min': 0.05,   # Don't go below table
            'z_max': 0.8,    # Max height
        }

    def get_current_pose(self):
        """Get current end-effector position"""
        try:
            trans = self.tf_buffer.lookup_transform(
                BASE_FRAME, TARGET_LINK, rclpy.time.Time()
            )
            return trans.transform
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
    
    def check_workspace_limits(self, x, y, z):
        """Check if position is within known workspace limits"""
        if x < self.workspace['x_min'] or x > self.workspace['x_max']:
            return False, f"X={x:.2f} outside range [{self.workspace['x_min']:.2f}, {self.workspace['x_max']:.2f}]"
        if y < self.workspace['y_min'] or y > self.workspace['y_max']:
            return False, f"Y={y:.2f} outside range [{self.workspace['y_min']:.2f}, {self.workspace['y_max']:.2f}]"
        if z < self.workspace['z_min'] or z > self.workspace['z_max']:
            return False, f"Z={z:.2f} outside range [{self.workspace['z_min']:.2f}, {self.workspace['z_max']:.2f}]"
        return True, "OK"
    
    def quick_ik_check(self, x, y, z, timeout=1.0):
        """Quick IK feasibility check before planning"""
        # Try a few different orientations
        orientations = [
            (1.0, 0.0, 0.0, 0.0),
            (0.707, 0.707, 0.0, 0.0),
            (0.707, 0.0, 0.707, 0.0),
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
            
            req = GetPositionIK.Request()
            req.ik_request = ik_req
            
            try:
                fut = self.ik_client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout + 0.2)
                
                if fut.done() and fut.result() and fut.result().error_code.val == 1:
                    return True, "IK solution found"
            except:
                continue
        
        return False, "No IK solution"

    def create_position_goal(self, target_x, target_y, target_z):
        """Create position-only constraint goal"""
        goal = MoveGroup.Goal()
        
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.num_planning_attempts = 3
        req.allowed_planning_time = PLANNING_TIME
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = BASE_FRAME
        position_constraint.link_name = TARGET_LINK
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # Small sphere around target
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerance
        bounding_volume.primitives = [sphere]
        
        sphere_pose = PoseStamped()
        sphere_pose.header.frame_id = BASE_FRAME
        sphere_pose.pose.position.x = target_x
        sphere_pose.pose.position.y = target_y
        sphere_pose.pose.position.z = target_z
        sphere_pose.pose.orientation.w = 1.0
        bounding_volume.primitive_poses = [sphere_pose.pose]
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints = [position_constraint]
        req.goal_constraints = [constraints]
        
        # Workspace bounds
        req.workspace_parameters.header.frame_id = BASE_FRAME
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -0.2
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.5
        
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        
        return goal
    
    def move_to_home(self):
        """Return to home position"""
        self.get_logger().info("🏠 Returning to home...")
        
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        
        constraints = Constraints()
        joint_names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
        joint_values = [1.241, -1.2419, 1.4493, 0.0955, 1.5014]
        
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        req.goal_constraints = [constraints]
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        
        return self.action_client.send_goal_async(goal)
    
    def print_position(self):
        """Display current position with workspace info"""
        current_tf = self.get_current_pose()
        if not current_tf:
            print("❌ Cannot read position")
            return
        
        x = current_tf.translation.x
        y = current_tf.translation.y
        z = current_tf.translation.z
        
        print("\n" + "="*50)
        print("📍 Current End-Effector Position")
        print("="*50)
        print(f"  X (forward/back): {x:+.3f}m")
        print(f"  Y (left/right):   {y:+.3f}m")
        print(f"  Z (up/down):      {z:+.3f}m")
        print("\n💡 Workspace Limits:")
        print(f"  X: [{self.workspace['x_min']:+.2f}, {self.workspace['x_max']:+.2f}]")
        print(f"  Y: [{self.workspace['y_min']:+.2f}, {self.workspace['y_max']:+.2f}]")
        print(f"  Z: [{self.workspace['z_min']:+.2f}, {self.workspace['z_max']:+.2f}]")
        
        # Check if current position is good
        in_workspace, msg = self.check_workspace_limits(x, y, z)
        if in_workspace:
            print("✅ Currently in safe workspace")
        else:
            print(f"⚠️  {msg}")
        print("="*50 + "\n")

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = SmartXYZMover()
    print(msg)
    
    # Show initial position
    node.print_position()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = getKey(settings)
            
            if key == 'x': 
                break
            
            if key == 'p':
                node.print_position()
                continue
            
            if key == 'r':
                node.get_logger().info("🏠 Going home...")
                fut = node.move_to_home()
                rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
                if fut.done():
                    gh = fut.result()
                    if gh.accepted:
                        node.get_logger().info("✅ Home move accepted")
                        result_fut = gh.get_result_async()
                        rclpy.spin_until_future_complete(node, result_fut, timeout_sec=10.0)
                continue
                
            if key == '': 
                continue

            # Get current position
            current_tf = node.get_current_pose()
            if not current_tf:
                node.get_logger().warn("⚠️ Cannot get current position")
                continue

            # Calculate target
            target_x = current_tf.translation.x
            target_y = current_tf.translation.y
            target_z = current_tf.translation.z

            if key == 'w': 
                target_x += STEP_SIZE
                direction = "Forward (+X)"
            elif key == 's': 
                target_x -= STEP_SIZE
                direction = "Backward (-X)"
            elif key == 'a': 
                target_y += STEP_SIZE
                direction = "Left (+Y)"
            elif key == 'd': 
                target_y -= STEP_SIZE
                direction = "Right (-Y)"
            elif key == 'q': 
                target_z += STEP_SIZE
                direction = "Up (+Z)"
            elif key == 'e': 
                target_z -= STEP_SIZE
                direction = "Down (-Z)"
            else: 
                continue

            # Check workspace limits first
            in_workspace, limit_msg = node.check_workspace_limits(target_x, target_y, target_z)
            if not in_workspace:
                node.get_logger().warn(f"⚠️ Target outside workspace: {limit_msg}")
                print(f"   💡 Try moving in a different direction")
                continue
            
            node.get_logger().info(f"📍 Target: [{target_x:.3f}, {target_y:.3f}, {target_z:.3f}] {direction}")
            
            # Quick IK check
            node.get_logger().info("🔍 Checking reachability...")
            reachable, ik_msg = node.quick_ik_check(target_x, target_y, target_z, timeout=1.0)
            
            if not reachable:
                node.get_logger().warn(f"⚠️ Position appears unreachable: {ik_msg}")
                node.get_logger().warn("   Try a smaller step or different direction")
                continue
            
            node.get_logger().info("✅ Position is reachable, planning...")
            
            # Create and send goal
            goal = node.create_position_goal(target_x, target_y, target_z)
            fut = node.action_client.send_goal_async(goal)
            
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            
            if not fut.done():
                node.get_logger().error("❌ Goal send timeout")
                continue
                
            goal_handle = fut.result()
            if not goal_handle.accepted:
                node.get_logger().warn("⚠️ Goal rejected by MoveIt")
                continue
            
            node.get_logger().info("⏳ Executing motion...")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=PLANNING_TIME + 5.0)
            
            if result_future.done():
                result = result_future.result()
                if result.result.error_code.val == 1:
                    node.get_logger().info("✅ Move successful!")
                else:
                    node.get_logger().warn(f"⚠️ Move failed: error code {result.result.error_code.val}")
            else:
                node.get_logger().error("❌ Motion timeout")
            
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
