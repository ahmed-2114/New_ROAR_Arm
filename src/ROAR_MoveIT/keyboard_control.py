#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import math
import sys, select, termios, tty

# --- SETTINGS ---
GROUP_NAME = "arm_controller"
BASE_FRAME = "base_link"
TARGET_LINK = "Link_5"  # Moving the wrist center point
STEP_SIZE = 0.05        # 5cm steps
# ----------------

msg = """
---------------------------------------
   ROAR XYZ Teleop (5-DOF Optimized)
---------------------------------------
   w / s : Forward / Backward (+X / -X)
   a / d : Left / Right      (+Y / -Y)
   q / e : Up / Down        (+Z / -Z)

   x     : Exit
---------------------------------------
"""

class XYZMover(Node):
    def __init__(self):
        super().__init__('xyz_mover', 
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, MoveGroup, 'move_action')
        # Trajectory action client for direct joint control
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/arm_controller_controller/follow_joint_trajectory')
        # IK service client (MoveIt provides /compute_ik)
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        self.get_logger().info("Connecting to MoveIt Brain...")
        self.action_client.wait_for_server()
        self.get_logger().info("Connected! XYZ mode active.")

    def get_link_pos(self):
        try:
            # Current coordinates of Link_5 relative to base_link
            trans = self.tf_buffer.lookup_transform(BASE_FRAME, TARGET_LINK, rclpy.time.Time())
            return trans.transform.translation
        except:
            return None

    def send_goal(self, target_xyz):
        # Robust approach: compute IK using MoveIt's /compute_ik service,
        # then send a FollowJointTrajectory goal directly to the controller.
        # 1) Wait for IK service
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('IK service /compute_ik not available')
            return None
        # Try several wrist orientations to increase IK success chance
        # We'll sweep yaw angles and small pitch offsets
        yaw_samples = [0.0, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, math.pi]
        pitch_samples = [0.0, math.pi/12, -math.pi/12]
        roll = 0.0
        ik_resp = None
        last_err = None
        for yaw in yaw_samples:
            for pitch in pitch_samples:
                # construct quaternion from roll,pitch,yaw
                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)
                q_w = cr * cp * cy + sr * sp * sy
                q_x = sr * cp * cy - cr * sp * sy
                q_y = cr * sp * cy + sr * cp * sy
                q_z = cr * cp * sy - sr * sp * cy

                ik_req = PositionIKRequest()
                ik_req.group_name = GROUP_NAME
                ik_req.ik_link_name = TARGET_LINK
                ik_req.pose_stamped.header.frame_id = BASE_FRAME
                ik_req.pose_stamped.pose.position.x = target_xyz.x
                ik_req.pose_stamped.pose.position.y = target_xyz.y
                ik_req.pose_stamped.pose.position.z = target_xyz.z
                ik_req.pose_stamped.pose.orientation.x = q_x
                ik_req.pose_stamped.pose.orientation.y = q_y
                ik_req.pose_stamped.pose.orientation.z = q_z
                ik_req.pose_stamped.pose.orientation.w = q_w
                ik_req.timeout.sec = 3

                req = GetPositionIK.Request()
                req.ik_request = ik_req

                fut = self.ik_client.call_async(req)
                rclpy.spin_until_future_complete(self, fut)
                if not fut.done() or fut.result() is None:
                    self.get_logger().warn('IK call failed (no response)')
                    continue
                resp = fut.result()
                # Log the full response (error code + any message) for debugging
                try:
                    self.get_logger().info(f'IK response error_code={resp.error_code.val}')
                except Exception:
                    pass

                if resp.error_code.val == 1:
                    ik_resp = resp
                    break
                else:
                    last_err = resp.error_code.val
            if ik_resp:
                break

        if ik_resp is None:
            self.get_logger().warn(f'IK solver failed after orientation sweep, trying small position perturbations')
            # Try small position perturbations (±1-2 cm) to help solver
            deltas = [0.0, 0.01, -0.01, 0.02, -0.02]
            for dx in deltas:
                for dy in deltas:
                    for dz in deltas:
                        if dx == 0.0 and dy == 0.0 and dz == 0.0:
                            continue
                        tx = target_xyz.x + dx
                        ty = target_xyz.y + dy
                        tz = target_xyz.z + dz
                        self.get_logger().info(f'Trying perturbed target: {tx:.3f},{ty:.3f},{tz:.3f}')
                        # orientation sweep for this perturbed position
                        for yaw in yaw_samples:
                            for pitch in pitch_samples:
                                cy = math.cos(yaw * 0.5)
                                sy = math.sin(yaw * 0.5)
                                cp = math.cos(pitch * 0.5)
                                sp = math.sin(pitch * 0.5)
                                cr = math.cos(0.0 * 0.5)
                                sr = math.sin(0.0 * 0.5)
                                q_w = cr * cp * cy + sr * sp * sy
                                q_x = sr * cp * cy - cr * sp * sy
                                q_y = cr * sp * cy + sr * cp * sy
                                q_z = cr * cp * sy - sr * sp * cy

                                ik_req = PositionIKRequest()
                                ik_req.group_name = GROUP_NAME
                                ik_req.ik_link_name = TARGET_LINK
                                ik_req.pose_stamped.header.frame_id = BASE_FRAME
                                ik_req.pose_stamped.pose.position.x = tx
                                ik_req.pose_stamped.pose.position.y = ty
                                ik_req.pose_stamped.pose.position.z = tz
                                ik_req.pose_stamped.pose.orientation.x = q_x
                                ik_req.pose_stamped.pose.orientation.y = q_y
                                ik_req.pose_stamped.pose.orientation.z = q_z
                                ik_req.pose_stamped.pose.orientation.w = q_w
                                ik_req.timeout.sec = 3

                                req = GetPositionIK.Request()
                                req.ik_request = ik_req

                                fut = self.ik_client.call_async(req)
                                rclpy.spin_until_future_complete(self, fut)
                                if not fut.done() or fut.result() is None:
                                    continue
                                resp = fut.result()
                                self.get_logger().info(f'IK response error_code={resp.error_code.val}')
                                if resp.error_code.val == 1:
                                    ik_resp = resp
                                    break
                            if ik_resp:
                                break
                        if ik_resp:
                            break
                    if ik_resp:
                        break
                if ik_resp:
                    break

            if ik_resp is None:
                self.get_logger().warn(f'IK solver failed after perturbation attempts, last_error={last_err}')
                return None

        # Extract joint positions for the arm joints
        js = ik_resp.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        joint_names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
        positions = []
        for j in joint_names:
            if j in name_to_pos:
                positions.append(name_to_pos[j])
            else:
                self.get_logger().error(f'IK result missing joint {j}')
                return None

        # 2) Send FollowJointTrajectory goal
        if not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Trajectory action server not available')
            return None

        from control_msgs.action import FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = 1
        traj.points = [pt]
        goal.trajectory = traj

        send_fut = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        if not send_fut.done():
            self.get_logger().error('Failed to send trajectory goal')
            return None
        gh = send_fut.result()
        if not gh.accepted:
            self.get_logger().warn('Controller rejected trajectory goal')
            return None
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return gh

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = XYZMover()
    print(msg)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = getKey(settings)
            if key == 'x': break
            if key == '': continue

            current_pos = node.get_link_pos()
            if not current_pos: continue

            target = Point()
            target.x, target.y, target.z = current_pos.x, current_pos.y, current_pos.z

            if key == 'w': target.x += STEP_SIZE
            elif key == 's': target.x -= STEP_SIZE
            elif key == 'a': target.y += STEP_SIZE
            elif key == 'd': target.y -= STEP_SIZE
            elif key == 'q': target.z += STEP_SIZE
            elif key == 'e': target.z -= STEP_SIZE
            else: continue

            node.get_logger().info(f"XYZ Request: {target.x:.2f}, {target.y:.2f}, {target.z:.2f}")
            res = node.send_goal(target)
            # send_goal may return None (failure), a goal handle, or a future-like object.
            if res is None:
                # send_goal already logged the error
                continue

            # If send_goal returned a goal handle (we already waited inside), just report
            if hasattr(res, 'accepted'):
                try:
                    if res.accepted:
                        node.get_logger().info('Trajectory goal accepted (sent to controller)')
                    else:
                        node.get_logger().warn('Trajectory goal was rejected by controller')
                except Exception:
                    node.get_logger().info('Trajectory goal sent')
                continue

            # Otherwise assume it's a future: wait for it and process like before
            try:
                rclpy.spin_until_future_complete(node, res)
            except Exception as e:
                node.get_logger().error(f'Error waiting for future: {e}')
                continue

            try:
                goal_handle = res.result()
            except Exception as e:
                node.get_logger().error(f"Failed to send goal: {e}")
                continue

            if not goal_handle.accepted:
                node.get_logger().warn("MoveIt rejected the goal")
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            try:
                result = result_future.result()
                node.get_logger().info(f"Move result: {result}")
            except Exception as e:
                node.get_logger().error(f"Error getting result: {e}")
                continue
            
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()