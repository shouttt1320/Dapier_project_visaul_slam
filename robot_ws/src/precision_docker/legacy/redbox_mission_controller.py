#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist, PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class RedBoxMissionController(Node):
    def __init__(self):
        super().__init__('redbox_mission_controller')

        # Parameters
        self.declare_parameter('search_angular_velocity', 0.25)
        self.declare_parameter('fine_align_angular_velocity', 0.12)
        self.declare_parameter('align_pixel_tolerance', 0.08)  # normalized x tolerance

        self.search_w = self.get_parameter('search_angular_velocity').value
        self.align_w = self.get_parameter('fine_align_angular_velocity').value
        self.align_tol = self.get_parameter('align_pixel_tolerance').value

        # Nav2 BasicNavigator
        self.navigator = BasicNavigator()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State storage
        self.latest_detection = None
        self.latest_goal = None
        self.detection_streak = 0

        # Subscribers
        self.sub_detection = self.create_subscription(
            PointStamped,
            '/redbox/detection',
            self.detection_callback,
            10
        )
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/redbox/target_goal',
            self.goal_callback,
            10
        )

        self.get_logger().info('RedBox Mission Controller Node initialized.')

    def detection_callback(self, msg: PointStamped):
        self.latest_detection = msg
        if msg.point.z > 0.5:
            self.detection_streak += 1
        else:
            self.detection_streak = max(0, self.detection_streak - 1)

    def goal_callback(self, msg: PoseStamped):
        self.latest_goal = msg

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def run_mission(self):
        self.get_logger().info('==========================================')
        self.get_logger().info(' Starting Red Box Autonomous Mission... ')
        self.get_logger().info('==========================================')

        # 1. Wait for Nav2 to be fully active (Using SLAM localization, not AMCL)
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        self.get_logger().info('Nav2 is active and ready!')

        # 2. Phase 1: Search for Red Box (360-degree rotation)
        self.get_logger().info('Phase 1: Searching for Red Box...')
        search_start = time.time()
        max_search_time = 35.0  # seconds for full revolution

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check if we have consistent detection
            if self.detection_streak >= 5 and self.latest_goal is not None:
                self.publish_cmd_vel(0.0, 0.0)
                self.get_logger().info('>>> Red Box Detected & Locked! Target Goal Acquired.')
                break

            # Check search timeout
            if time.time() - search_start > max_search_time:
                self.publish_cmd_vel(0.0, 0.0)
                self.get_logger().warn('Search timeout: Red Box not found in 360-degree scan.')
                return

            # Rotate in place to search
            self.publish_cmd_vel(0.0, self.search_w)
            time.sleep(0.05)

        # 3. Phase 2: Autonomous Navigation using Nav2
        target_goal = self.latest_goal
        self.get_logger().info(f'Phase 2: Navigating to target stopping pose: ({target_goal.pose.position.x:.2f}, {target_goal.pose.position.y:.2f})...')

        self.navigator.goToPose(target_goal)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            feedback = self.navigator.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                self.get_logger().info(f'Navigating... Distance remaining: {dist:.2f}m', throttle_duration_sec=2.0)
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('>>> Nav2 Navigation Succeeded! Reached target stopping zone (0.5m).')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation was canceled.')
            return
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed to reach the goal pose.')
            return

        # 4. Phase 3: Fine Visual Alignment
        self.get_logger().info('Phase 3: Fine Visual Alignment towards Red Box...')
        align_start = time.time()
        while rclpy.ok() and (time.time() - align_start < 8.0):
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_detection is not None and self.latest_detection.point.z > 0.5:
                norm_x = self.latest_detection.point.x
                if abs(norm_x) < self.align_tol:
                    self.get_logger().info('>>> Visual Alignment Complete!')
                    break
                # Rotate towards center
                w_cmd = -self.align_w if norm_x > 0 else self.align_w
                self.publish_cmd_vel(0.0, w_cmd)
            else:
                self.publish_cmd_vel(0.0, 0.0)
            time.sleep(0.05)

        # 5. Phase 4: Final Stop & Complete
        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().info('========================================================')
        self.get_logger().info(' [MISSION COMPLETE] Robot successfully stopped 0.5m in front of Red Box! ')
        self.get_logger().info('========================================================')


def main(args=None):
    rclpy.init(args=args)
    controller = RedBoxMissionController()
    try:
        controller.run_mission()
    except KeyboardInterrupt:
        controller.publish_cmd_vel(0.0, 0.0)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
