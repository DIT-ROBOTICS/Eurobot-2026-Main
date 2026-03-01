#!/usr/bin/env python3
"""
Component Test Script — Interactive Hardware/Feature Checker

Provides an interactive menu to test individual robot components:
  1. Nav2 NavigateToPose  — send (x, y, yaw) goal
  2. Flip                 — publish Int16MultiArray to /robot/on_flip
  3. Put                  — publish Int16 to /robot/on_put
  4. Take                 — publish Int16 to /robot/on_take
  5. Zero cmd_vel         — publish Twist(0,0,0) to /cmd_vel

Usage:
    python3 src/startup/scripts/component_test.py
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int16, Int16MultiArray


# ── colour helpers ──────────────────────────────────────────────
CYAN   = "\033[36m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
BOLD   = "\033[1m"
RESET  = "\033[0m"


class ComponentTester(Node):
    """Interactive node for testing robot components one by one."""

    def __init__(self):
        super().__init__("component_tester")

        # ── publishers ──────────────────────────────────────────
        self.flip_pub      = self.create_publisher(Int16MultiArray, "/robot/on_flip",   10)
        self.put_pub       = self.create_publisher(Int16,           "/robot/on_put",    10)
        self.take_pub      = self.create_publisher(Int16,           "/robot/on_take",   10)
        self.vel_pub       = self.create_publisher(Twist,           "/cmd_vel",         10)
        self.dock_side_pub = self.create_publisher(Int16,           "/robot/dock_side", 10)

        # ── nav2 action client ──────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info("Component Tester ready.")
        self._print_menu()

        # run the interactive prompt on a daemon thread
        threading.Thread(target=self._input_loop, daemon=True).start()

    # ────────────────────────────────────────────────────────────
    #  Menu
    # ────────────────────────────────────────────────────────────
    @staticmethod
    def _print_menu():
        print(f"""
{BOLD}{CYAN}╔══════════════════════════════════════════════════╗
║         Component Test — Interactive Menu        ║
╠══════════════════════════════════════════════════╣
║  1  │ Nav2 NavigateToPose  (x, y, yaw)           ║
║  2  │ Flip      → /robot/on_flip  (Int16Multi)   ║
║  3  │ Put       → /robot/on_put   (Int16)        ║
║  4  │ Take      → /robot/on_take  (Int16)        ║
║  5  │ Zero cmd_vel                               ║
║  6  │ Dock Side → /robot/dock_side (Int16)       ║
║  7  │ cmd_vel   → /cmd_vel  (vx, vy, wz)        ║
║  h  │ Show this menu                             ║
║  q  │ Quit                                       ║
╚══════════════════════════════════════════════════╝{RESET}
""")

    # ────────────────────────────────────────────────────────────
    #  Input loop
    # ────────────────────────────────────────────────────────────
    def _input_loop(self):
        while rclpy.ok():
            try:
                cmd = input(f"{BOLD}> {RESET}").strip().lower()
                if cmd == "q":
                    print("Exiting…")
                    rclpy.shutdown()
                    break
                elif cmd == "h" or cmd == "?":
                    self._print_menu()
                elif cmd == "1":
                    self._cmd_nav2()
                elif cmd == "2":
                    self._cmd_flip()
                elif cmd == "3":
                    self._cmd_put()
                elif cmd == "4":
                    self._cmd_take()
                elif cmd == "5":
                    self._cmd_zero_vel()
                elif cmd == "6":
                    self._cmd_dock_side()
                elif cmd == "7":
                    self._cmd_vel()
                elif cmd == "":
                    pass
                else:
                    print(f"{RED}Unknown command: {cmd}. Type 'h' for help.{RESET}")
            except (EOFError, KeyboardInterrupt):
                break

    # ────────────────────────────────────────────────────────────
    #  1 — NavigateToPose
    # ────────────────────────────────────────────────────────────
    def _cmd_nav2(self):
        try:
            x   = float(input(f"  {YELLOW}x   : {RESET}"))
            y   = float(input(f"  {YELLOW}y   : {RESET}"))
            yaw = float(input(f"  {YELLOW}yaw (deg): {RESET}"))
        except ValueError:
            print(f"{RED}Invalid number.{RESET}")
            return

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # yaw → quaternion  (rotation about Z)
        yaw_rad = math.radians(yaw)
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        print(f"{GREEN}Sending NavigateToPose → x={x}, y={y}, yaw={yaw}°{RESET}")
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        pos = fb.current_pose.pose.position
        self.get_logger().info(
            f"Nav feedback — pos=({pos.x:.2f}, {pos.y:.2f})"
        )

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("NavigateToPose goal REJECTED")
            return
        self.get_logger().info("NavigateToPose goal accepted, waiting for result…")
        goal_handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"{GREEN}NavigateToPose SUCCEEDED{RESET}")
        else:
            self.get_logger().warn(f"NavigateToPose finished with status {status}")

    # ────────────────────────────────────────────────────────────
    #  2 — Flip
    # ────────────────────────────────────────────────────────────
    def _cmd_flip(self):
        """
        /robot/on_flip  →  Int16MultiArray (length 5)
          index 0-3 : which hazelnut slots to flip (1 = flip, 0 = no)
          index 4   : side index
        Example input: 1 0 1 1 3  → flip slots 0,2,3 on side 3
        """
        raw = input(
            f"  {YELLOW}Enter 5 ints (flip0 flip1 flip2 flip3 side): {RESET}"
        ).strip()
        try:
            vals = list(map(int, raw.split()))
            if len(vals) != 5:
                raise ValueError
        except ValueError:
            print(f"{RED}Need exactly 5 integers.{RESET}")
            return

        msg = Int16MultiArray()
        msg.data = vals
        self.flip_pub.publish(msg)
        print(f"{GREEN}Published /robot/on_flip → {vals}{RESET}")

    # ────────────────────────────────────────────────────────────
    #  3 — Put
    # ────────────────────────────────────────────────────────────
    def _cmd_put(self):
        """
        /robot/on_put  →  Int16
          data : side index to perform put action
        """
        raw = input(f"  {YELLOW}Enter side index (Int16): {RESET}").strip()
        try:
            val = int(raw)
        except ValueError:
            print(f"{RED}Invalid integer.{RESET}")
            return

        msg = Int16()
        msg.data = val
        self.put_pub.publish(msg)
        print(f"{GREEN}Published /robot/on_put → {val}{RESET}")

    # ────────────────────────────────────────────────────────────
    #  4 — Take
    # ────────────────────────────────────────────────────────────
    def _cmd_take(self):
        """
        /robot/on_take  →  Int16
          data : side index to perform take action
        """
        raw = input(f"  {YELLOW}Enter side index (Int16): {RESET}").strip()
        try:
            val = int(raw)
        except ValueError:
            print(f"{RED}Invalid integer.{RESET}")
            return

        msg = Int16()
        msg.data = val
        self.take_pub.publish(msg)
        print(f"{GREEN}Published /robot/on_take → {val}{RESET}")

    # ────────────────────────────────────────────────────────────
    #  5 — Zero cmd_vel
    # ────────────────────────────────────────────────────────────
    def _cmd_zero_vel(self):
        msg = Twist()  # all fields default to 0.0
        self.vel_pub.publish(msg)
        print(f"{GREEN}Published zero Twist to /cmd_vel{RESET}")

    # ────────────────────────────────────────────────────────────
    #  6 — Dock Side
    # ────────────────────────────────────────────────────────────
    def _cmd_dock_side(self):
        """
        /robot/dock_side  →  Int16
          data : dock side index
        """
        raw = input(f"  {YELLOW}Enter dock side (Int16): {RESET}").strip()
        try:
            val = int(raw)
        except ValueError:
            print(f"{RED}Invalid integer.{RESET}")
            return

        msg = Int16()
        msg.data = val
        self.dock_side_pub.publish(msg)
        print(f"{GREEN}Published /robot/dock_side → {val}{RESET}")

    # ────────────────────────────────────────────────────────────
    #  7 — cmd_vel
    # ────────────────────────────────────────────────────────────
    def _cmd_vel(self):
        """
        /cmd_vel  →  Twist
          linear.x, linear.y, angular.z
        """
        try:
            vx = float(input(f"  {YELLOW}linear.x  (vx) : {RESET}"))
            vy = float(input(f"  {YELLOW}linear.y  (vy) : {RESET}"))
            wz = float(input(f"  {YELLOW}angular.z (wz) : {RESET}"))
        except ValueError:
            print(f"{RED}Invalid number.{RESET}")
            return

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.vel_pub.publish(msg)
        print(f"{GREEN}Published /cmd_vel → vx={vx}, vy={vy}, wz={wz}{RESET}")


# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = ComponentTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
