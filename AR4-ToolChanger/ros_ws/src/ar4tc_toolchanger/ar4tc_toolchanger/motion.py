"""Thin synchronous wrapper around the MoveIt 2 move_group services and actions."""
import time

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState

SUCCESS = MoveItErrorCodes.SUCCESS


def se3_to_pose(M: pin.SE3) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in M.translation)
    q = pin.Quaternion(M.rotation)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q.x, q.y, q.z, q.w
    return p


class MoveItClient:
    def __init__(self, node, cfg):
        self.node = node
        self.group = cfg["group"]
        self.tip = cfg["tip_link"]
        self.joints = cfg["arm_joints"]
        self.base = cfg["base_frame"]
        self.vel = cfg["velocity_scaling"]
        self.acc = cfg["acceleration_scaling"]
        self._js = {}
        node.create_subscription(JointState, "/joint_states", lambda m: self._js.update(zip(m.name, m.position)), 10)
        self.ik_srv = node.create_client(GetPositionIK, "/compute_ik")
        self.cart_srv = node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.scene_srv = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_scene_srv = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.move_ac = ActionClient(node, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        self.grip_ac = ActionClient(node, GripperCommand, cfg.get("gripper_action", "/gripper_controller/gripper_cmd"))
        self.traj_ac_name = cfg["trajectory_action"]
        self.last_trajectory = None   # joint trajectory of the last executed motion (for recording)

    # ---- plumbing -------------------------------------------------------
    def _wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("ROS call timed out")
        return future.result()

    def _action(self, client, goal, timeout=60.0):
        handle = self._wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError(f"goal rejected by {client._action_name}")
        return self._wait(handle.get_result_async(), timeout).result

    def wait_ready(self, timeout=120.0):
        """Block until move_group and the arm trajectory controller are up."""
        from control_msgs.action import FollowJointTrajectory
        traj = ActionClient(self.node, FollowJointTrajectory, self.traj_ac_name)
        deadline = time.time() + timeout
        for what, ok in [("compute_ik", lambda: self.ik_srv.wait_for_service(1.0)),
                         ("move_action", lambda: self.move_ac.wait_for_server(1.0)),
                         ("trajectory controller", lambda: traj.wait_for_server(1.0)),
                         ("joint_states", lambda: self.joint_positions(1.0) is not None)]:
            while not ok():
                if time.time() > deadline:
                    raise TimeoutError(f"{what} not available")
            self.node.get_logger().info(f"ready: {what}")

    def robot_description(self, node_name="/move_group") -> str:
        cli = self.node.create_client(GetParameters, f"{node_name}/get_parameters")
        cli.wait_for_service(10.0)
        res = self._wait(cli.call_async(GetParameters.Request(names=["robot_description"])))
        return res.values[0].string_value

    def joint_positions(self, settle=0.3):
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not all(j in self._js for j in self.joints):
            return None
        return np.array([self._js[j] for j in self.joints])

    # ---- kinematics / motion -------------------------------------------
    def compute_ik(self, flange: pin.SE3, seed, avoid_collisions=True, attempts=5):
        """MoveIt IK for the tip link; retries from perturbed seeds. Returns joint array or None."""
        rng = np.random.default_rng(0)
        for i in range(attempts):
            s = np.asarray(seed) + (rng.normal(0, 0.3, len(seed)) if i else 0.0)
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name, r.ik_link_name, r.avoid_collisions = self.group, self.tip, avoid_collisions
            r.robot_state.joint_state = JointState(name=self.joints, position=[float(v) for v in s])
            r.pose_stamped.header.frame_id = self.base
            r.pose_stamped.pose = se3_to_pose(flange)
            r.timeout.nanosec = 200_000_000
            res = self._wait(self.ik_srv.call_async(req))
            if res.error_code.val == SUCCESS:
                js = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([js[j] for j in self.joints])
        return None

    def move_to_joints(self, q, tol=1e-3):
        g = MoveGroup.Goal()
        rq = g.request
        rq.group_name, rq.num_planning_attempts, rq.allowed_planning_time = self.group, 5, 5.0
        rq.max_velocity_scaling_factor, rq.max_acceleration_scaling_factor = self.vel, self.acc
        rq.start_state.is_diff = True
        rq.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(v), tolerance_above=tol, tolerance_below=tol, weight=1.0)
            for j, v in zip(self.joints, q)])]
        g.planning_options.plan_only = False
        res = self._action(self.move_ac, g, 90.0)
        self.last_trajectory = res.planned_trajectory.joint_trajectory
        return res.error_code.val == SUCCESS, f"MoveGroup code {res.error_code.val}"

    def move_linear(self, flange: pin.SE3, avoid_collisions=True, step=0.005):
        """Straight-line TCP move (Cartesian path), then execute. Returns (ok, info)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base
        req.start_state.is_diff = True
        req.group_name, req.link_name = self.group, self.tip
        req.waypoints = [se3_to_pose(flange)]
        req.max_step, req.avoid_collisions = step, avoid_collisions
        for name, val in [("jump_threshold", 0.0), ("max_velocity_scaling_factor", self.vel),
                          ("max_acceleration_scaling_factor", self.acc)]:
            if hasattr(req, name):
                setattr(req, name, val)
        res = self._wait(self.cart_srv.call_async(req))
        if res.error_code.val != SUCCESS or res.fraction < 0.999:
            return False, f"cartesian path fraction {res.fraction:.3f}, code {res.error_code.val}"
        pts = res.solution.joint_trajectory.points
        if not pts or (pts[-1].time_from_start.sec == 0 and pts[-1].time_from_start.nanosec == 0):
            return False, "cartesian path not time-parameterized"
        g = ExecuteTrajectory.Goal(trajectory=res.solution)
        self.last_trajectory = res.solution.joint_trajectory
        out = self._action(self.exec_ac, g, 60.0)
        return out.error_code.val == SUCCESS, f"{len(pts)} pts, exec code {out.error_code.val}"

    def gripper(self, position, effort=10.0, settle_timeout=3.0, tol=1e-3):
        g = GripperCommand.Goal()
        g.command.position, g.command.max_effort = float(position), effort
        self.grip_ac.wait_for_server(10.0)
        self._action(self.grip_ac, g, 15.0)
        # The action can return before a physical jaw (Gazebo, real servo) has finished moving.
        deadline, pos = time.time() + settle_timeout, None
        while time.time() < deadline:
            self.joint_positions(0.1)
            pos = self._js.get("gripper_jaw1_joint")
            if pos is not None and abs(pos - position) < tol:
                return True, f"jaw at {pos:.4f} m"
        return False, f"jaw at {pos:.4f} m after {settle_timeout} s" if pos is not None else "no jaw state"

    # ---- planning scene -------------------------------------------------
    def apply_scene(self, scene: PlanningScene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        self.scene_srv.wait_for_service(10.0)
        return self._wait(self.scene_srv.call_async(ApplyPlanningScene.Request(scene=scene))).success

    def get_scene(self):
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                     | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                     | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        self.get_scene_srv.wait_for_service(10.0)
        return self._wait(self.get_scene_srv.call_async(req)).scene
