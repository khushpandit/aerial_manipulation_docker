#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import termios
import tty
import select
import math
import subprocess


class SmoothCenterSafeDescent(Node):

    def __init__(self):
        super().__init__("smooth_center_safe_descent")

        # -------------------------------------------------------------
        # PX4
        # -------------------------------------------------------------
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        self.image_pub = self.create_publisher(
            Image,
            "/drone/tracking_feed",
            10,
        )

        self.rgb_sub = self.create_subscription(
            Image,
            "/gripper/camera",
            self.rgb_callback,
            10,
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/gripper/depth",
            self.depth_callback,
            10,
        )

        self.loc_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.loc_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.bridge = CvBridge()

        # -------------------------------------------------------------
        # ArUco -- KEEPING THE ORIGINAL, GOOD DETECTOR
        # -------------------------------------------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Close-range fallback detector. The original detector above remains
        # the PRIMARY detector. This one is only used after a miss.
        self.close_aruco_params = cv2.aruco.DetectorParameters()
        self.close_aruco_params.minMarkerPerimeterRate = 0.003
        self.close_aruco_params.maxMarkerPerimeterRate = 8.0
        self.close_aruco_params.adaptiveThreshWinSizeMin = 3
        self.close_aruco_params.adaptiveThreshWinSizeMax = 53
        self.close_aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.close_aruco_params,
        )

        self.vision_lost_frames = 0
        self.max_vision_lost_frames = 8
        self.close_detection_used = False

        self.last_valid_u = 400.0
        self.last_valid_v = 400.0

        self.cx = 400.0
        self.cy = 400.0

        self.target_found = False
        self.target_u = 400.0
        self.target_v = 400.0

        self.vision_lost_frames = 0
        self.max_vision_lost_frames = 5

        self.x_error_px = 0.0
        self.y_error_px = 0.0

        # Filtered normalized visual error.
        self.err_x = 0.0
        self.err_y = 0.0
        self.last_err_x = 0.0
        self.last_err_y = 0.0

        self.track_alpha = 0.35

        # -------------------------------------------------------------
        # Depth -- DISPLAY / MONITOR ONLY FOR THIS TEST
        # -------------------------------------------------------------
        self.z_distance = 99.9

        # -------------------------------------------------------------
        # Drone state
        # -------------------------------------------------------------
        self.drone_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -2.0

        self.yaw_offset = 0.0
        self.offboard_set = False
        self.track_mode = False

        # -------------------------------------------------------------
        # Smooth XY controller
        # -------------------------------------------------------------
        # Deliberately conservative because your priority is:
        # smoothness > speed.
        self.track_kp = 1.15
        self.track_kd = 0.08

        self.max_track_speed = 1.00
        self.max_track_accel = 1.00

        self.track_vn = 0.0
        self.track_ve = 0.0

        # -------------------------------------------------------------
        # Safe Z descent
        # -------------------------------------------------------------
        # From the Gazebo scene you supplied:
        # payload top ~= 0.681 m above ground.
        self.payload_top_height_m = 0.681

        # Deliberately stop before the payload for this stage.
        self.safe_clearance_m = 0.20
        self.safe_stop_altitude_m = (
            self.payload_top_height_m + self.safe_clearance_m
        )

        # Original working idea was:
        # target_z += small positive amount every cycle.
        #
        # 0.004 m / 0.05 s ~= 0.08 m/s downward.
        # This is intentionally slow.
        self.descent_step_m = 0.004

        self.center_lock_time = 0.0
        self.required_center_time = 0.10

        self.descent_active = False
        self.safe_height_reached = False

        # Last PX4 local position at which ArUco was confirmed centered.
        # Used for a precise position hold if vision is subsequently lost.
        self.last_center_x = None
        self.last_center_y = None
        self.position_hold_active = False

        self.best_center_error = float("inf")
        self.best_center_x = None
        self.best_center_y = None
        self.best_center_u = None
        self.best_center_v = None

        # Final autonomous task: descend -> grip -> ascend.
        self.mission_phase = "ALIGN"
        self.final_descent_step_m = 0.0025   # ~0.05 m/s at 50 ms
        self.final_floor_clearance_m = 0.03
        self.final_floor_altitude_m = self.payload_top_height_m + self.final_floor_clearance_m
        self.grip_depth_m = 0.15
        self.grip_hold_time = 0.25
        self.grip_stable_time = 0.0

        # -------------------------------------------------------------
        # HYBRID FINAL APPROACH
        # -------------------------------------------------------------
        # At the staging height, switch from visual servoing to:
        #   - saved best X/Y position hold
        #   - depth-controlled Z descent
        #
        # This makes ArUco optional in the final few centimeters.
        self.final_depth_entry_m = 0.35
        self.final_depth_slow_m = 0.20
        self.final_depth_stop_m = 0.15

        # Final descent speeds (NED position setpoint increments).
        # 0.00175 / 0.05 s ~= 0.035 m/s.
        self.final_step_normal_m = 0.00175
        self.final_step_slow_m = 0.00100

        # Final approach still uses visual XY correction when ArUco is
        # visible. It is deliberately weaker than the main tracker.
        self.final_track_speed = 0.25
        self.final_track_accel = 0.40
        # Grip -> settle -> tiny lift -> verify -> full ascent.
        self.grip_settle_time = 0.60
        self.grip_settle_elapsed = 0.0

        # A tiny lift is used as a physical grasp verification:
        # if the payload is attached, camera-to-payload depth stays nearly
        # constant; if it is not attached, depth increases by ~micro-lift.
        self.micro_lift_distance_m = 0.03
        self.micro_lift_step_m = 0.003
        self.micro_lift_target_z = None
        self.pre_lift_depth = float("nan")
        self.verify_timer = 0.0
        self.verify_window_s = 0.25
        self.grasp_depth_tolerance_m = 0.015
        self.grasp_verified = False

        self.ascend_distance_m = 0.50
        self.ascend_step_m = 0.006
        self.ascend_target_z = None
        self.payload_grabbed = False
        self.task_complete = False

        self.hud_counter = 0

        print(
            f"Grip gate: depth <= {self.grip_depth_m:.2f} m for "
            f"{self.grip_hold_time:.2f} s"
        )
        print(
            f"Grip verification: {self.micro_lift_distance_m:.2f} m micro-lift | "
            f"depth tolerance {self.grasp_depth_tolerance_m:.3f} m"
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.settings = termios.tcgetattr(sys.stdin)

        print("\n===============================================")
        print("   🚁 SMOOTH CENTER + SAFE DESCENT TEST 🚁")
        print("===============================================")
        print("MANUAL:")
        print("   W/S : Forward / Backward")
        print("   A/D : Left / Right")
        print("   R/F : Ascend / Descend")
        print("   Q/E : Yaw")
        print("")
        print("AUTONOMOUS:")
        print("   T   : Start / Stop tracking")
        print("")
        print("SAFETY:")
        print("   X   : Land")
        print("   Z   : Disarm")
        print("===============================================")
        print(
            f"Payload top:        {self.payload_top_height_m:.2f} m"
        )
        print(
            f"Safe stop altitude: {self.safe_stop_altitude_m:.2f} m"
        )
        print("No gripping in this test.")
        print("===============================================\n")

    # -------------------------------------------------------------
    # Feedback
    # -------------------------------------------------------------

    def loc_callback(self, msg):
        self.drone_yaw = msg.heading
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.cx = frame.shape[1] / 2.0
            self.cy = frame.shape[0] / 2.0

            cv2.line(
                frame,
                (int(self.cx) - 20, int(self.cy)),
                (int(self.cx) + 20, int(self.cy)),
                (255, 0, 0),
                2,
            )
            cv2.line(
                frame,
                (int(self.cx), int(self.cy) - 20),
                (int(self.cx), int(self.cy) + 20),
                (255, 0, 0),
                2,
            )

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            chosen_corners = None
            chosen_ids = None
            chosen_index = None
            detection_mode = "PRIMARY"

            # ---------------------------------------------------------
            # PRIMARY: original detector, unchanged.
            # ---------------------------------------------------------
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )

            if ids is not None and len(corners) > 0:
                chosen_corners = corners
                chosen_ids = ids
                chosen_index = 0

            # ---------------------------------------------------------
            # CLOSE FALLBACK 1: detect on half-resolution frame.
            # Large close markers are often easier to recover this way.
            # ---------------------------------------------------------
            if chosen_index is None:
                small = cv2.resize(
                    gray,
                    None,
                    fx=0.5,
                    fy=0.5,
                    interpolation=cv2.INTER_AREA,
                )

                scorners, sids, _ = self.close_aruco_detector.detectMarkers(
                    small
                )

                if sids is not None and len(scorners) > 0:
                    restored = [
                        (c / 0.5).astype(np.float32)
                        for c in scorners
                    ]
                    chosen_corners = restored
                    chosen_ids = sids
                    chosen_index = 0
                    detection_mode = "HALF"

            # ---------------------------------------------------------
            # CLOSE FALLBACK 2: search a large crop around the last known
            # marker center. This is especially useful when the marker
            # becomes huge and the rest of the image is irrelevant.
            # ---------------------------------------------------------
            if chosen_index is None:
                last_u = int(np.clip(
                    round(self.last_valid_u),
                    0,
                    frame.shape[1] - 1,
                ))
                last_v = int(np.clip(
                    round(self.last_valid_v),
                    0,
                    frame.shape[0] - 1,
                ))

                crop_half_w = max(200, int(frame.shape[1] * 0.32))
                crop_half_h = max(150, int(frame.shape[0] * 0.32))

                x0 = max(0, last_u - crop_half_w)
                x1 = min(frame.shape[1], last_u + crop_half_w)
                y0 = max(0, last_v - crop_half_h)
                y1 = min(frame.shape[0], last_v + crop_half_h)

                crop = gray[y0:y1, x0:x1]

                ccorners, cids, _ = self.close_aruco_detector.detectMarkers(
                    crop
                )

                if cids is not None and len(ccorners) > 0:
                    restored = []
                    for c in ccorners:
                        c2 = c.copy().astype(np.float32)
                        c2[:, :, 0] += float(x0)
                        c2[:, :, 1] += float(y0)
                        restored.append(c2)

                    chosen_corners = restored
                    chosen_ids = cids
                    chosen_index = 0
                    detection_mode = "CROP"

            # ---------------------------------------------------------
            # Valid detection
            # ---------------------------------------------------------
            if chosen_index is not None:
                c = chosen_corners[chosen_index][0]

                self.target_u = float(np.mean(c[:, 0]))
                self.target_v = float(np.mean(c[:, 1]))

                self.x_error_px = self.target_u - self.cx
                self.y_error_px = self.cy - self.target_v

                raw_x = self.x_error_px / max(self.cx, 1.0)
                raw_y = self.y_error_px / max(self.cy, 1.0)

                # Keep the same smooth low-pass behavior.
                self.err_x = (
                    self.track_alpha * raw_x
                    + (1.0 - self.track_alpha) * self.err_x
                )
                self.err_y = (
                    self.track_alpha * raw_y
                    + (1.0 - self.track_alpha) * self.err_y
                )

                self.target_found = True
                self.vision_lost_frames = 0
                self.close_detection_used = detection_mode != "PRIMARY"

                self.last_valid_u = self.target_u
                self.last_valid_v = self.target_v

                # Draw recovered marker.
                try:
                    cv2.aruco.drawDetectedMarkers(
                        frame,
                        [chosen_corners[chosen_index]],
                        np.array([[int(chosen_ids.flatten()[chosen_index])]]),
                    )
                except Exception:
                    pass

                cv2.circle(
                    frame,
                    (int(self.target_u), int(self.target_v)),
                    6,
                    (0, 255, 0),
                    -1,
                )

                cv2.line(
                    frame,
                    (int(self.cx), int(self.cy)),
                    (int(self.target_u), int(self.target_v)),
                    (0, 255, 255),
                    2,
                )

                cv2.putText(
                    frame,
                    f"ARUCO: {detection_mode}",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

            else:
                # Brief dropout: preserve the last known target and let
                # the smooth controller settle naturally.
                self.vision_lost_frames += 1

                if self.vision_lost_frames <= self.max_vision_lost_frames:
                    self.target_found = True
                    self.close_detection_used = True

                    # Ease the filtered error toward zero instead of chasing
                    # a stale large error. This is appropriate because the
                    # last good state was already being centered.
                    self.err_x *= 0.92
                    self.err_y *= 0.92

                    self.target_u = self.last_valid_u
                    self.target_v = self.last_valid_v

                    cv2.putText(
                        frame,
                        f"ARUCO: HOLD-LAST ({self.vision_lost_frames})",
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 200, 255),
                        2,
                    )
                else:
                    self.target_found = False
                    self.close_detection_used = False

                    cv2.putText(
                        frame,
                        "ARUCO: LOST",
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )

            cv2.putText(
                frame,
                f"ERR: {self.err_x:+.3f}, {self.err_y:+.3f}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            cv2.putText(
                frame,
                f"PX: {int(self.x_error_px):+d}, {int(self.y_error_px):+d}",
                (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            cv2.putText(
                frame,
                f"LOSS: {self.vision_lost_frames}/{self.max_vision_lost_frames}",
                (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, "bgr8")
            )

        except Exception:
            pass

    def depth_callback(self, msg):
        # Depth is only monitored in this stage.
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

            u = int(np.clip(
                round(self.target_u),
                0,
                depth.shape[1] - 1,
            ))
            v = int(np.clip(
                round(self.target_v),
                0,
                depth.shape[0] - 1,
            ))

            raw = float(depth[v, u])

            if math.isfinite(raw) and raw > 0.0:
                self.z_distance = raw

        except Exception:
            pass

    # -------------------------------------------------------------
    # PX4 / Gazebo helpers
    # -------------------------------------------------------------

    def arm_and_takeoff(self):
        # Kept from the original controller.
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
        )

        self.offboard_set = True

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
    ):
        msg = VehicleCommand()

        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        self.vehicle_command_publisher.publish(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)

        key = sys.stdin.read(1) if rlist else ""

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings,
        )

        return key.lower()

    def trigger_gripper(self, action):
        value = "0.046" if action == "grip" else "0.001"
        subprocess.Popen(
            f'gz topic -t /gripper/front -m gz.msgs.Double -p "data: {value}"',
            shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        subprocess.Popen(
            f'gz topic -t /gripper/back -m gz.msgs.Double -p "data: {value}"',
            shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    # -------------------------------------------------------------
    # Smooth XY controller
    # -------------------------------------------------------------

    def slew(self, current, target, max_delta):
        delta = float(
            np.clip(
                target - current,
                -max_delta,
                max_delta,
            )
        )
        return current + delta

    def smooth_tracking_velocity(self):
        dt = 0.05

        d_x = (self.err_x - self.last_err_x) / dt
        d_y = (self.err_y - self.last_err_y) / dt

        self.last_err_x = self.err_x
        self.last_err_y = self.err_y

        body_right = (
            self.track_kp * self.err_x
            + self.track_kd * d_x
        )

        body_forward = (
            self.track_kp * self.err_y
            + self.track_kd * d_y
        )

        speed = math.hypot(
            body_forward,
            body_right,
        )

        if speed > self.max_track_speed and speed > 1e-6:
            scale = self.max_track_speed / speed
            body_forward *= scale
            body_right *= scale

        angle = self.drone_yaw + self.yaw_offset

        target_vn = (
            body_forward * math.cos(angle)
            - body_right * math.sin(angle)
        )

        target_ve = (
            body_forward * math.sin(angle)
            + body_right * math.cos(angle)
        )

        max_delta = self.max_track_accel * dt

        self.track_vn = self.slew(
            self.track_vn,
            target_vn,
            max_delta,
        )

        self.track_ve = self.slew(
            self.track_ve,
            target_ve,
            max_delta,
        )

    def smooth_stop_tracking(self):
        max_delta = self.max_track_accel * 0.05

        self.track_vn = self.slew(
            self.track_vn,
            0.0,
            max_delta,
        )

        self.track_ve = self.slew(
            self.track_ve,
            0.0,
            max_delta,
        )

    # -------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------

    def timer_callback(self):

        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.velocity = True
        mode_msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        self.offboard_control_mode_publisher.publish(mode_msg)

        key = self.get_key()

        if key:
            print(f"\n[COMMAND] {key.upper()}")

        # ---------------------------------------------------------
        # T = autonomous
        # ---------------------------------------------------------
        if key == "t":
            self.track_mode = not self.track_mode

            if self.track_mode:
                self.target_z = self.current_z

                self.last_err_x = self.err_x
                self.last_err_y = self.err_y

                self.track_vn = 0.0
                self.track_ve = 0.0

                self.center_lock_time = 0.0
                self.descent_active = False
                self.safe_height_reached = False
                self.last_center_x = None
                self.last_center_y = None
                self.position_hold_active = False
                self.best_center_error = float("inf")
                self.best_center_x = None
                self.best_center_y = None
                self.best_center_u = None
                self.best_center_v = None
                self.mission_phase = "ALIGN"
                self.grip_stable_time = 0.0
                self.grip_settle_elapsed = 0.0
                self.micro_lift_target_z = None
                self.pre_lift_depth = float("nan")
                self.verify_timer = 0.0
                self.grasp_verified = False
                self.ascend_target_z = None
                self.payload_grabbed = False
                self.task_complete = False

                print(
                    f"\n>>> AUTONOMOUS TRACKING ON "
                    f"| START ALT: {-self.target_z:.2f}m <<<"
                )
                print(
                    f">>> WILL STOP AT ~"
                    f"{self.safe_stop_altitude_m:.2f}m <<<"
                )
                print(
                    f">>> FINAL HYBRID: depth <= "
                    f"{self.final_depth_entry_m:.2f}m -> "
                    f"depth-controlled Z; grip <= "
                    f"{self.grip_depth_m:.2f}m <<<"
                )

            else:
                self.track_vn = 0.0
                self.track_ve = 0.0

                self.center_lock_time = 0.0
                self.descent_active = False
                self.safe_height_reached = False
                self.position_hold_active = False

                print(
                    "\n>>> AUTONOMOUS TRACKING OFF | "
                    "MANUAL RESUMED <<<"
                )

        # ---------------------------------------------------------
        # Safety
        # ---------------------------------------------------------
        if key == "g":
            self.trigger_gripper("grip")
            self.payload_grabbed = True
            print("\n>>> MANUAL GRIP <<<")

        elif key == "v":
            self.trigger_gripper("release")
            self.payload_grabbed = False
            print("\n>>> MANUAL RELEASE <<<")

        elif key == "x":
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )

        elif key == "z":
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                0.0,
            )
            self.offboard_set = False

        # ---------------------------------------------------------
        # Build setpoint
        # ---------------------------------------------------------
        sp_msg = TrajectorySetpoint()

        if self.track_mode:
            action_status = ""

            # Once safe height is reached, hold the saved best local X/Y.
            # Do not resume visual chasing even if ArUco comes back.
            if self.safe_height_reached:
                # FINAL_READY: stay over the saved center until the marker is
                # centered again, then begin a very slow final approach.
                if self.mission_phase == "FINAL_READY":
                    # At staging height, stop the normal visual velocity
                    # command, then evaluate the live payload depth.
                    self.smooth_stop_tracking()
                    self.position_hold_active = True

                    # The drone holds its CURRENT local X/Y. We use the
                    # best-center position only as historical diagnostic
                    # data, not as a command target.
                    hold_x = float(self.current_x)
                    hold_y = float(self.current_y)

                    if (
                        math.isfinite(self.z_distance)
                        and self.z_distance <= self.final_depth_entry_m
                    ):
                        self.mission_phase = "FINAL_DEPTH"
                        self.grip_stable_time = 0.0

                        print(
                            "\n>>> DEPTH IN FINAL ZONE | "
                            "LIVE FINAL XY + DEPTH Z <<<"
                        )

                    sp_msg.position = [
                        hold_x,
                        hold_y,
                        float(self.target_z),
                    ]
                    sp_msg.velocity = [
                        float("nan"),
                        float("nan"),
                        float("nan"),
                    ]
                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")

                    depth_text = (
                        f"{self.z_distance:.3f}"
                        if math.isfinite(self.z_distance)
                        else "---"
                    )

                    action_status = (
                        f"FINAL READY | CURRENT XY HOLD | D={depth_text}m"
                    )

                # ---------------------------------------------------------
                # FINAL_DEPTH:
                # - ArUco visible  -> weak/smooth live XY correction
                # - ArUco lost     -> hold CURRENT position
                # - Depth          -> controls final Z motion
                # ---------------------------------------------------------
                elif self.mission_phase == "FINAL_DEPTH":
                    depth_valid = math.isfinite(self.z_distance)

                    if self.target_found:
                        # Keep the same smooth visual direction, but cap it
                        # much lower for the last few centimeters.
                        self.position_hold_active = False

                        # Reuse the existing smooth controller, then scale
                        # its final command to a conservative close-range
                        # limit.
                        self.smooth_tracking_velocity()

                        vxy = math.hypot(
                            self.track_vn,
                            self.track_ve,
                        )

                        if vxy > self.final_track_speed and vxy > 1e-6:
                            scale = (
                                self.final_track_speed / vxy
                            )
                            self.track_vn *= scale
                            self.track_ve *= scale

                        action_status = "FINAL | LIVE XY + DEPTH Z"

                    else:
                        # Crucial change:
                        # do NOT jump back to an earlier best-center X/Y.
                        # Hold the CURRENT position exactly where vision
                        # was lost.
                        self.position_hold_active = True
                        self.smooth_stop_tracking()

                        action_status = "FINAL | VISION LOST | HOLD CURRENT"

                    # -----------------------------
                    # Final Z: depth-controlled
                    # -----------------------------
                    if depth_valid:
                        if self.z_distance > self.final_depth_entry_m:
                            step = self.final_step_normal_m
                        elif self.z_distance > self.final_depth_slow_m:
                            step = self.final_step_normal_m
                        elif self.z_distance > self.final_depth_stop_m:
                            step = self.final_step_slow_m
                        else:
                            step = 0.0

                        floor_z_ned = -self.final_floor_altitude_m

                        if (
                            step > 0.0
                            and self.target_z < floor_z_ned
                        ):
                            self.target_z = min(
                                self.target_z + step,
                                floor_z_ned,
                            )

                        # Depth is the primary final grip distance.
                        if self.z_distance <= self.grip_depth_m:
                            self.grip_stable_time += 0.05
                        else:
                            self.grip_stable_time = max(
                                0.0,
                                self.grip_stable_time - 0.05,
                            )

                        if (
                            self.grip_stable_time >= self.grip_hold_time
                        ):
                            self.trigger_gripper("grip")
                            self.payload_grabbed = True
                            self.mission_phase = "GRIP_SETTLE"
                            self.grip_settle_elapsed = 0.0
                            self.position_hold_active = True
                            self.target_z = self.target_z

                            print(
                                "\n>>> DEPTH TARGET REACHED | "
                                "GRIP CONFIRMED <<<"
                            )
                            print(
                                f"    FINAL DEPTH={self.z_distance:.3f}m | "
                                f"Z={-self.target_z:.3f}m"
                            )

                        action_status += (
                            f" | D={self.z_distance:.3f}m "
                            f"| GripTimer={self.grip_stable_time:.2f}s"
                        )
                    else:
                        # No depth = no blind downward motion.
                        action_status += " | DEPTH LOST | HOLD"

                    # If visual tracking is active, publish velocity XY.
                    # Otherwise publish the CURRENT X/Y as a position hold.
                    if self.position_hold_active:
                        sp_msg.position = [
                            float(self.current_x),
                            float(self.current_y),
                            float(self.target_z),
                        ]
                        sp_msg.velocity = [
                            float("nan"),
                            float("nan"),
                            float("nan"),
                        ]
                    else:
                        sp_msg.position = [
                            float("nan"),
                            float("nan"),
                            float(self.target_z),
                        ]
                        sp_msg.velocity = [
                            float(self.track_vn),
                            float(self.track_ve),
                            float("nan"),
                        ]

                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")

                # -----------------------------------------------------
                # GRIP_SETTLE: freeze the drone while the jaws close.
                # -----------------------------------------------------
                elif self.mission_phase == "GRIP_SETTLE":
                    self.grip_settle_elapsed += 0.05
                    self.position_hold_active = True
                    self.smooth_stop_tracking()

                    hold_x = (
                        self.current_x
                    )
                    hold_y = (
                        self.current_y
                    )

                    sp_msg.position = [
                        float(hold_x),
                        float(hold_y),
                        float(self.target_z),
                    ]
                    sp_msg.velocity = [
                        float("nan"),
                        float("nan"),
                        float("nan"),
                    ]
                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")

                    action_status = (
                        f"GRIP SETTLING {self.grip_settle_elapsed:.2f}s"
                    )

                    if self.grip_settle_elapsed >= self.grip_settle_time:
                        self.pre_lift_depth = (
                            float(self.z_distance)
                            if math.isfinite(self.z_distance)
                            else float("nan")
                        )

                        self.micro_lift_target_z = (
                            self.target_z
                            - self.micro_lift_distance_m
                        )

                        self.mission_phase = "MICRO_LIFT"
                        self.verify_timer = 0.0
                        self.grasp_verified = False

                        print(
                            "\n>>> GRIP SETTLED | "
                            "MICRO-LIFT VERIFY STARTED <<<"
                        )
                        print(
                            f"    Pre-lift depth: "
                            f"{self.pre_lift_depth:.3f}m"
                        )

                # -----------------------------------------------------
                # MICRO_LIFT: only 3 cm, with X/Y completely frozen.
                # -----------------------------------------------------
                elif self.mission_phase == "MICRO_LIFT":
                    self.position_hold_active = True
                    self.smooth_stop_tracking()

                    hold_x = self.current_x
                    hold_y = self.current_y

                    if self.micro_lift_target_z is None:
                        self.micro_lift_target_z = (
                            self.target_z
                            - self.micro_lift_distance_m
                        )

                    if self.target_z > self.micro_lift_target_z:
                        self.target_z = max(
                            self.target_z - self.micro_lift_step_m,
                            self.micro_lift_target_z,
                        )
                        action_status = "MICRO-LIFT | VERIFYING GRASP"
                    else:
                        self.target_z = self.micro_lift_target_z
                        self.mission_phase = "VERIFY_GRASP"
                        self.verify_timer = 0.0

                        print(
                            "\n>>> MICRO-LIFT REACHED | "
                            "VERIFYING PAYLOAD ATTACHMENT <<<"
                        )

                    sp_msg.position = [
                        float(hold_x),
                        float(hold_y),
                        float(self.target_z),
                    ]
                    sp_msg.velocity = [
                        float("nan"),
                        float("nan"),
                        float("nan"),
                    ]
                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")

                # -----------------------------------------------------
                # VERIFY_GRASP:
                # attached payload -> depth should remain nearly constant
                # detached payload -> depth should increase with the lift.
                # -----------------------------------------------------
                elif self.mission_phase == "VERIFY_GRASP":
                    self.verify_timer += 0.05
                    self.position_hold_active = True
                    self.smooth_stop_tracking()

                    hold_x = self.current_x
                    hold_y = self.current_y

                    depth_valid = math.isfinite(self.z_distance)
                    depth_delta = (
                        self.z_distance - self.pre_lift_depth
                        if (
                            depth_valid
                            and math.isfinite(self.pre_lift_depth)
                        )
                        else float("nan")
                    )

                    if (
                        depth_valid
                        and math.isfinite(self.pre_lift_depth)
                        and abs(depth_delta)
                        <= self.grasp_depth_tolerance_m
                    ):
                        self.grasp_verified = True

                    if self.verify_timer >= self.verify_window_s:
                        if self.grasp_verified:
                            self.mission_phase = "ASCEND"
                            self.ascend_target_z = (
                                self.target_z
                                - self.ascend_distance_m
                            )

                            print(
                                "\n>>> GRASP VERIFIED ✅ | "
                                f"DEPTH DELTA={depth_delta:.3f}m | "
                                "FULL ASCENT STARTING <<<"
                            )
                        else:
                            # Do NOT fly away with a failed grasp.
                            self.trigger_gripper("release")
                            self.payload_grabbed = False
                            self.mission_phase = "GRASP_FAILED"

                            print(
                                "\n>>> GRASP FAILED ❌ | "
                                f"DEPTH DELTA="
                                f"{depth_delta:.3f}m | "
                                "HOLDING POSITION <<<"
                            )

                    sp_msg.position = [
                        float(hold_x),
                        float(hold_y),
                        float(self.target_z),
                    ]
                    sp_msg.velocity = [
                        float("nan"),
                        float("nan"),
                        float("nan"),
                    ]
                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")

                    action_status = (
                        f"VERIFY GRASP | "
                        f"Dd={depth_delta:.3f}m"
                        if math.isfinite(depth_delta)
                        else "VERIFY GRASP | Dd=---"
                    )

                # -----------------------------------------------------
                # GRASP_FAILED: safe hold. No automatic flight away.
                # -----------------------------------------------------
                elif self.mission_phase == "GRASP_FAILED":
                    self.position_hold_active = True
                    self.smooth_stop_tracking()

                    hold_x = self.current_x
                    hold_y = self.current_y

                    sp_msg.position = [
                        float(hold_x),
                        float(hold_y),
                        float(self.target_z),
                    ]
                    sp_msg.velocity = [
                        float("nan"),
                        float("nan"),
                        float("nan"),
                    ]
                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")

                    action_status = "GRASP FAILED | HOLDING"

                # ASCEND: fixed local X/Y, smooth position-Z climb.
                elif self.mission_phase == "ASCEND":
                    self.position_hold_active = True
                    self.smooth_stop_tracking()
                    hold_x = self.last_center_x if self.last_center_x is not None else self.current_x
                    hold_y = self.last_center_y if self.last_center_y is not None else self.current_y

                    if self.ascend_target_z is None:
                        self.ascend_target_z = self.target_z - self.ascend_distance_m

                    if self.target_z > self.ascend_target_z:
                        self.target_z = max(self.target_z - self.ascend_step_m, self.ascend_target_z)
                        action_status = "ASCENDING | VERIFIED PAYLOAD"
                    else:
                        self.target_z = self.ascend_target_z
                        self.mission_phase = "COMPLETE"
                        self.task_complete = True
                        print("\n>>> TASK COMPLETE | PAYLOAD GRIPPED + ASCENDED <<<")
                        action_status = "TASK COMPLETE | HOLDING"

                    sp_msg.position = [float(hold_x), float(hold_y), float(self.target_z)]
                    sp_msg.velocity = [float("nan"), float("nan"), float("nan")]
                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")

                else:  # COMPLETE
                    self.position_hold_active = True
                    self.smooth_stop_tracking()
                    hold_x = self.last_center_x if self.last_center_x is not None else self.current_x
                    hold_y = self.last_center_y if self.last_center_y is not None else self.current_y
                    hold_z = self.ascend_target_z if self.ascend_target_z is not None else self.target_z
                    sp_msg.position = [float(hold_x), float(hold_y), float(hold_z)]
                    sp_msg.velocity = [float("nan"), float("nan"), float("nan")]
                    sp_msg.yaw = float(self.drone_yaw)
                    sp_msg.yawspeed = float("nan")
                    action_status = "TASK COMPLETE | PAYLOAD SECURED"

                self.hud_counter += 1
                if self.hud_counter % 5 == 0:
                    depth_text = f"{self.z_distance:.3f}" if math.isfinite(self.z_distance) else "---"
                    sys.stdout.write(
                        f"\\r[MISSION] {self.mission_phase:<13} | {action_status:<28} "
                        f"| Drift {int(self.x_error_px) if self.target_found else 0:4d},"
                        f"{int(self.y_error_px) if self.target_found else 0:4d} "
                        f"| Z {-self.target_z:.2f}m | D {depth_text}m"
                    )
                    sys.stdout.flush()

            elif not self.target_found:
                # During descent, a genuine vision loss holds the last saved
                # centered local position and freezes Z.
                self.smooth_stop_tracking()

                if self.last_center_x is not None and self.last_center_y is not None:
                    self.position_hold_active = True
                    sp_msg.position = [
                        float(self.last_center_x),
                        float(self.last_center_y),
                        float(self.target_z),
                    ]
                    action_status = "LOST | HOLD LAST CENTER POS"
                else:
                    sp_msg.position = [
                        float(self.current_x),
                        float(self.current_y),
                        float(self.target_z),
                    ]
                    action_status = "LOST | SAFE HOLD"

                sp_msg.velocity = [
                    float("nan"),
                    float("nan"),
                    float("nan"),
                ]
                sp_msg.yaw = float(self.drone_yaw)
                sp_msg.yawspeed = float("nan")

            else:
                self.position_hold_active = False
                self.smooth_tracking_velocity()

                current_center_error = math.hypot(
                    self.x_error_px,
                    self.y_error_px,
                )

                if current_center_error < self.best_center_error:
                    self.best_center_error = current_center_error
                    self.best_center_x = float(self.current_x)
                    self.best_center_y = float(self.current_y)
                    self.best_center_u = float(self.target_u)
                    self.best_center_v = float(self.target_v)

                centered = (
                    abs(self.x_error_px) <= 50.0
                    and abs(self.y_error_px) <= 50.0
                )

                if centered:
                    self.center_lock_time += 0.05
                else:
                    self.center_lock_time = max(
                        0.0,
                        self.center_lock_time - 0.05,
                    )

                if (
                    not self.descent_active
                    and not self.safe_height_reached
                    and self.center_lock_time >= self.required_center_time
                ):
                    self.last_center_x = (
                        self.best_center_x
                        if self.best_center_x is not None
                        else float(self.current_x)
                    )
                    self.last_center_y = (
                        self.best_center_y
                        if self.best_center_y is not None
                        else float(self.current_y)
                    )

                    self.descent_active = True
                    print(
                        "\n>>> CENTER LOCKED | SMOOTH DESCENT STARTED <<<"
                    )
                    print(
                        f"    BEST CENTER POS: "
                        f"X={self.last_center_x:+.3f} "
                        f"Y={self.last_center_y:+.3f} | "
                        f"PIXEL ERR={self.best_center_error:.1f}"
                    )

                if self.descent_active:
                    stop_z_ned = -self.safe_stop_altitude_m

                    if self.target_z < stop_z_ned:
                        self.target_z = min(
                            self.target_z + self.descent_step_m,
                            stop_z_ned,
                        )
                        action_status = "TRACK + DESCENDING"
                    else:
                        self.target_z = stop_z_ned
                        self.descent_active = False
                        self.safe_height_reached = True
                        self.position_hold_active = True
                        self.mission_phase = "FINAL_READY"
                        print(
                            "\n>>> SAFE HEIGHT REACHED | "
                            "READY FOR FINAL DESCENT / GRIP <<<"
                        )
                        print(
                            f"    HOLD POS: "
                            f"X={self.last_center_x:+.3f} "
                            f"Y={self.last_center_y:+.3f}"
                        )
                        action_status = "SAFE HEIGHT | POSITION HOLD"
                else:
                    action_status = "SMOOTHLY ALIGNING"

                sp_msg.position = [
                    float("nan"),
                    float("nan"),
                    float(self.target_z),
                ]
                sp_msg.velocity = [
                    float(self.track_vn),
                    float(self.track_ve),
                    float("nan"),
                ]
                sp_msg.yaw = float(self.drone_yaw)
                sp_msg.yawspeed = float("nan")

            self.hud_counter += 1
            if self.hud_counter % 10 == 0:
                drift_x = int(self.x_error_px) if self.target_found else 0
                drift_y = int(self.y_error_px) if self.target_found else 0
                sys.stdout.write(
                    f"\r[MISSION] {action_status:<31} "
                    f"| Drift: {drift_x:4d},{drift_y:4d} "
                    f"| AltCmd: {-self.target_z:.2f}m "
                    f"| Vxy: {math.hypot(self.track_vn, self.track_ve):.2f} "
                    f"| Lock: {self.center_lock_time:.2f}s "
                    f"| Hold:{'POS' if self.position_hold_active else 'VIS'} "
                    f"| Best:{self.best_center_error:5.1f}px"
                )
                sys.stdout.flush()

        else:
            # ---------------------------------------------------------
            # EXACT REFERENCE MANUAL CONTROLS
            # ---------------------------------------------------------
            body_vx = 0.0
            body_vy = 0.0
            vz = 0.0
            yaw_rate = 0.0

            speed = 3.0

            if key == "w":
                body_vx = speed
            elif key == "s":
                body_vx = -speed
            elif key == "a":
                body_vy = -speed
            elif key == "d":
                body_vy = speed
            elif key == "r":
                vz = -speed
            elif key == "f":
                vz = speed
            elif key == "q":
                yaw_rate = -1.0
            elif key == "e":
                yaw_rate = 1.0

            earth_vx = (
                body_vx * math.cos(self.drone_yaw)
                - body_vy * math.sin(self.drone_yaw)
            )

            earth_vy = (
                body_vx * math.sin(self.drone_yaw)
                + body_vy * math.cos(self.drone_yaw)
            )

            sp_msg.position = [
                float("nan"),
                float("nan"),
                float("nan"),
            ]

            sp_msg.velocity = [
                float(earth_vx),
                float(earth_vy),
                float(vz),
            ]

            sp_msg.yaw = float("nan")
            sp_msg.yawspeed = float(yaw_rate)

        # ---------------------------------------------------------
        # Auto-arm / Offboard -- same mechanism as the proven code
        # ---------------------------------------------------------
        if (
            not self.offboard_set
            and key in [
                "w",
                "a",
                "s",
                "d",
                "r",
                "f",
                "q",
                "e",
                "t",
            ]
        ):
            self.arm_and_takeoff()

        sp_msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        self.trajectory_setpoint_publisher.publish(sp_msg)


def main(args=None):
    rclpy.init(args=args)

    node = SmoothCenterSafeDescent()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                node.settings,
            )
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
