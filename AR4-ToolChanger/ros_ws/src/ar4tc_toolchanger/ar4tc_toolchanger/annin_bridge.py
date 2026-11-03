"""ROS 2 <-> Annin AR4 (Teensy sketch v6.0) bridge. Firmware and DIP switches stay unchanged.

Provides what MoveIt expects from ros2_control:
  /joint_states                                               (sensor_msgs/JointState)
  /joint_trajectory_controller/follow_joint_trajectory        (control_msgs/FollowJointTrajectory)
plus
  ~/enable   (std_srvs/SetBool)  motion gate, OFF at start; turned off again by any fault
  ~/status   (std_msgs/String)   last event, latched

See docs/ANNIN_BRIDGE.md.
"""
import threading
import time

import numpy as np
import rclpy
import serial
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from . import annin_protocol as ap

JOINTS = [f"joint_{i}" for i in range(1, 7)]
R = FollowJointTrajectory.Result


class AnninBridge(Node):
    def __init__(self):
        super().__init__("annin_bridge")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyACM0").value
        self.baud = p("baud", 9600).value
        self.enabled = p("start_enabled", False).value
        self.max_speed = p("max_joint_speed_deg_s", 20.0).value
        self.min_seg = p("min_segment_seconds", 0.4).value
        self.path_tol = p("path_tolerance_deg", 0.2).value
        self.start_tol = p("start_tolerance_deg", 1.0).value
        self.goal_tol = p("goal_tolerance_deg", 0.5).value
        self.limit_margin = p("limit_margin_deg", 1.0).value
        p("max_travel_deg", 0.0)  # 0 = off; read per goal, so `ros2 param set` works while running
        self.acc, self.dcc, self.ramp = p("acc_pct", 20.0).value, p("dcc_pct", 20.0).value, p("ramp_pct", 20.0).value
        self.poll_hz = p("poll_hz", 10.0).value
        self.reply_timeout = p("reply_timeout_s", 2.0).value

        self.ser = None
        self.lock = threading.Lock()          # one serial transaction at a time
        self.q_deg = None                     # last measured Annin joints
        self.ext_deg = (0.0, 0.0, 0.0)
        self.moving = False
        self.estimate_deg = None              # interpolated estimate published while a blocking RJ runs
        self.last_known_deg = None            # survives disconnects, to detect position jumps / Teensy resets
        self.suspect = ""                     # non-empty = enabling refused until the bridge is restarted

        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "~/status", latched)
        self.create_service(SetBool, "~/enable", self._on_enable, callback_group=cb)
        self.action = ActionServer(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory",
                                   execute_callback=self._execute, goal_callback=self._on_goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        self.create_timer(1.0 / self.poll_hz, self._poll, callback_group=cb)
        self.create_timer(0.05, self._publish, callback_group=cb)
        self.create_timer(3.0, self._check_duplicates, callback_group=cb)
        self._status(f"starting; port {self.port}; motion {'ENABLED' if self.enabled else 'DISABLED'}")

    def _check_duplicates(self):
        """Two bridges both answer MoveIt's trajectory goals (and would fight over the port, though the
        exclusive lock prevents that). Lock motion in every instance that sees a twin."""
        if not self.suspect and [n for n, _ in self.get_node_names_and_namespaces()].count(self.get_name()) > 1:
            self.enabled, self.suspect = False, "another annin_bridge node is running"
            self._status("MOTION LOCKED: another annin_bridge is running. Stop ALL launches (Ctrl+C in every "
                         "window), then start exactly one.", True)

    # ---- serial ---------------------------------------------------------
    def _status(self, text, error=False):
        # rclpy forbids changing severity at one call site, so keep two separate calls.
        if error:
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _open(self):
        if self.ser is None:
            # exclusive: a second process (another bridge, ModemManager, ...) can't open the port meanwhile
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1, exclusive=True)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self._status(f"serial open: {self.port}")

    def _transact(self, cmd, timeout):
        """Send one command, return the first non-empty reply line (or raise TimeoutError)."""
        self._open()
        self.ser.reset_input_buffer()
        self.ser.write(cmd.encode("ascii"))
        deadline, buf = time.time() + timeout, b""
        while time.time() < deadline:
            buf += self.ser.read(256)
            if b"\n" in buf:
                line, _, _ = buf.partition(b"\n")
                if line.strip():
                    return line.decode("ascii", "replace")
                buf = buf.split(b"\n", 1)[1]
        raise TimeoutError(f"no reply to {cmd.strip()[:2]} within {timeout:.1f} s")

    def _fault(self, text):
        self.enabled = False
        self._status(f"FAULT: {text} -> motion DISABLED", error=True)

    # ---- state ----------------------------------------------------------
    def _poll(self):
        if self.moving or not self.lock.acquire(blocking=False):
            return
        try:
            # E-stop sends an unsolicited position line flagged EB; check for it first.
            if self.ser is not None and self.ser.in_waiting:
                r = ap.parse_reply(self.ser.readline().decode("ascii", "replace"))
                if r.estop:
                    self._fault("E-stop pressed")
            r = ap.parse_reply(self._transact("RP\n", self.reply_timeout))
            if r.joints_deg is not None and r.raw.startswith("A"):
                if self.q_deg is None:  # first reading after (re)connecting: compare with the HMI display
                    self._status("robot position, Annin/HMI degrees J1..J6: "
                                 + " ".join(f"{v:.3f}" for v in r.joints_deg))
                    self._check_reconnect(r.joints_deg)
                self.q_deg, self.ext_deg = r.joints_deg, r.ext_deg
                self.last_known_deg = r.joints_deg
            if not r.ok and self.enabled:
                self._fault(r.alarm)
        except (serial.SerialException, OSError, TimeoutError) as e:
            if self.ser is not None:
                self.ser.close()
            self.ser, self.q_deg = None, None
            if self.enabled:
                self._fault(f"connection lost ({e})")
            else:
                self._status(f"no connection: {e}", error=True)
        finally:
            self.lock.release()

    def _check_reconnect(self, q):
        # Sketch v6.0 starts with every joint at exactly 0.000 after power-up/reset (StepM = zeroStep),
        # i.e. uncalibrated. A real calibrated pose of all zeros to 3 decimals is practically impossible.
        if np.all(np.abs(q) < 0.0005):
            self.suspect = "all joints read 0.000: Teensy was reset/power-cycled and is NOT calibrated"
        elif self.last_known_deg is not None and np.max(np.abs(q - self.last_known_deg)) > 1.0:
            self.suspect = (f"position jumped {np.max(np.abs(q - self.last_known_deg)):.2f} deg across a "
                            "reconnect: controller state is not trustworthy")
        if self.suspect:
            self.enabled = False
            self._status(f"MOTION LOCKED: {self.suspect}. Recalibrate in the HMI, then restart the bridge.", True)

    def _publish(self):
        q = self.estimate_deg if self.moving and self.estimate_deg is not None else self.q_deg
        if q is None:
            return
        js = JointState(name=JOINTS, position=ap.annin_to_ros(q).tolist())
        js.header.stamp = self.get_clock().now().to_msg()
        self.js_pub.publish(js)

    def _on_enable(self, req, res):
        if req.data and self.suspect:
            res.success, res.message = False, f"locked: {self.suspect}"
            return res
        if req.data and self.q_deg is None:
            res.success, res.message = False, "no position from robot yet"
            return res
        self.enabled = req.data
        res.success, res.message = True, f"motion {'ENABLED' if req.data else 'DISABLED'}"
        self._status(res.message)
        return res

    # ---- trajectory execution ------------------------------------------
    def _on_goal(self, goal):
        if not self.enabled:
            self._status("goal rejected: motion disabled (ros2 service call /annin_bridge/enable std_srvs/srv/SetBool '{data: true}')", True)
            return GoalResponse.REJECT
        if self.moving or self.q_deg is None:
            self._status("goal rejected: busy or no robot position", True)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _abort(self, handle, code, text):
        self._fault(text)
        handle.abort()
        return R(error_code=code, error_string=text)

    def _execute(self, handle):
        traj = handle.request.trajectory
        if sorted(traj.joint_names) != sorted(JOINTS) or not traj.points:
            return self._abort(handle, R.INVALID_JOINTS, f"unexpected joints {traj.joint_names}")
        order = [traj.joint_names.index(j) for j in JOINTS]
        times = [ap.duration_from_msg(pt.time_from_start) for pt in traj.points]
        pts = np.array([ap.ros_to_annin([pt.positions[i] for i in order]) for pt in traj.points])

        # Every point inside the firmware limits. Joints that move must also end >= limit_margin inside.
        # (A joint that stays put may sit inside the margin: the HMI rest pose has J2 = -41.8 next to -42.)
        for k, q in enumerate(pts):
            if not ap.within_limits(q):
                return self._abort(handle, R.INVALID_GOAL, f"point {k} outside firmware limits: {np.round(q, 2).tolist()}")
        moved = np.abs(pts[-1] - self.q_deg) > 0.05
        in_margin = (pts[-1] < -ap.LIM_NEG + self.limit_margin) | (pts[-1] > ap.LIM_POS - self.limit_margin)
        if np.any(moved & in_margin):
            return self._abort(handle, R.INVALID_GOAL, f"target within {self.limit_margin} deg of a limit: "
                                                       f"{np.round(pts[-1], 2).tolist()}")
        dev = float(np.max(np.abs(pts[0] - self.q_deg)))
        if dev > self.start_tol:
            return self._abort(handle, R.INVALID_GOAL, f"trajectory starts {dev:.2f} deg from the measured position")

        max_travel = self.get_parameter("max_travel_deg").value
        travel = np.max(np.abs(pts - self.q_deg), axis=0)
        if max_travel > 0 and travel.max() > max_travel:
            j = int(travel.argmax())
            return self._abort(handle, R.INVALID_GOAL,
                               f"J{j + 1} would travel {travel[j]:.1f} deg > max_travel_deg {max_travel:g} "
                               f"(travel per joint: {np.round(travel, 1).tolist()})")

        times = [t - times[0] for t in times]
        times, pts = ap.simplify(times, np.vstack([self.q_deg, pts[1:]]), self.path_tol)
        secs = ap.segment_durations(times, pts, self.max_speed, self.min_seg)
        self.get_logger().info(f"executing {len(traj.points)} points as {len(secs)} RJ segments, {sum(secs):.1f} s")

        with self.lock:
            self.moving = True
            try:
                for k, sec in enumerate(secs):
                    if handle.is_cancel_requested:
                        handle.canceled()
                        self._status(f"canceled after segment {k}/{len(secs)}")
                        return R(error_code=R.SUCCESSFUL, error_string="canceled")
                    if not self.enabled:
                        return self._abort(handle, R.PATH_TOLERANCE_VIOLATED, "disabled during motion")
                    r = self._move(pts[k], pts[k + 1], sec)
                    if not r.ok:
                        return self._abort(handle, R.PATH_TOLERANCE_VIOLATED, f"segment {k + 1}/{len(secs)}: {r.alarm}")
                    self.q_deg, self.ext_deg = r.joints_deg, r.ext_deg
                    if r.speed_violation:
                        self.get_logger().warn(f"segment {k + 1}: firmware capped speed (move took longer than {sec:.2f} s)")
                    fb = FollowJointTrajectory.Feedback(joint_names=JOINTS)
                    fb.actual.positions = ap.annin_to_ros(self.q_deg).tolist()
                    handle.publish_feedback(fb)
            except (serial.SerialException, OSError, TimeoutError) as e:
                return self._abort(handle, R.PATH_TOLERANCE_VIOLATED, f"serial: {e}")
            finally:
                self.moving, self.estimate_deg = False, None

        err = float(np.max(np.abs(self.q_deg - pts[-1])))
        if err > self.goal_tol:
            return self._abort(handle, R.GOAL_TOLERANCE_VIOLATED, f"final position off by {err:.3f} deg")
        handle.succeed()
        self._status(f"trajectory done, final error {err:.3f} deg")
        return R(error_code=R.SUCCESSFUL)

    def _move(self, q_from, q_to, sec):
        """One blocking RJ. Publishes a linear estimate meanwhile (the firmware can't report mid-move)."""
        cmd = ap.format_rj(q_to, sec, self.ext_deg, self.acc, self.dcc, self.ramp)
        result, t0 = {}, time.time()

        def worker():
            try:
                # Firmware 's' timing is about sec * 1.2; allow generous slack before declaring a timeout.
                result["line"] = self._transact(cmd, sec * 3.0 + 5.0)
            except Exception as e:  # noqa: BLE001 - reported below
                result["err"] = e

        th = threading.Thread(target=worker, daemon=True)
        th.start()
        while th.is_alive():
            a = min((time.time() - t0) / (sec * 1.2), 1.0)
            self.estimate_deg = q_from + a * (q_to - q_from)
            time.sleep(0.05)
        if "err" in result:
            raise result["err"]
        return ap.parse_reply(result["line"])


def main():
    rclpy.init()
    node = AnninBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()  # just close the port; never send CL (it calls Serial.end() on the Teensy)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
