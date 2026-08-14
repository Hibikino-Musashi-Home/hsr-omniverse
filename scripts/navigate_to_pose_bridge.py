#!/usr/bin/env python3
"""Nav2なしで hsrb_interface の NavigateToPose を台車軌道へ橋渡しする。"""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


class NavigateToPoseBridge(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_bridge')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True),
        ])
        self._group = ReentrantCallbackGroup()
        self._tf = Buffer()
        self._tf_listener = TransformListener(
            self._tf, self, spin_thread=False)
        self._trajectory = ActionClient(
            self,
            FollowJointTrajectory,
            '/omni_base_controller/follow_joint_trajectory',
            callback_group=self._group,
        )
        self._servers = [
            ActionServer(
                self,
                NavigateToPose,
                action_name,
                execute_callback=self._execute,
                goal_callback=self._goal,
                cancel_callback=self._cancel,
                callback_group=self._group,
            )
            for action_name in ('/navigate_to_pose', '/move_base/move')
        ]
        self.get_logger().info(
            'Nav2-less NavigateToPose bridge is ready on '
            '/navigate_to_pose and /move_base/move (map == odom)')

    def _goal(self, _request):
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _current_pose(self):
        transform = self._tf.lookup_transform(
            'odom', 'base_footprint', rclpy.time.Time())
        translation = transform.transform.translation
        return translation.x, translation.y, yaw_of(
            transform.transform.rotation), transform

    @staticmethod
    def _wait_future(future, timeout=None):
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        return done.wait(timeout)

    def _target(self, request, current):
        pose = request.pose.pose
        frame = request.pose.header.frame_id.strip('/') or 'map'
        goal_yaw = yaw_of(pose.orientation)
        if frame in ('base_footprint', 'base_link'):
            cosr = math.cos(current[2])
            sinr = math.sin(current[2])
            x = current[0] + pose.position.x * cosr - pose.position.y * sinr
            y = current[1] + pose.position.x * sinr + pose.position.y * cosr
            yaw = current[2] + goal_yaw
        elif frame in ('map', 'odom', 'world'):
            # localization/Nav2を起動しないsim構成ではmapとodomを同一視する。
            x = float(pose.position.x)
            y = float(pose.position.y)
            yaw = goal_yaw
        else:
            raise ValueError('unsupported goal frame: {}'.format(frame))
        # 等価な角度のうち、現在姿勢から最短の表現を軌道へ渡す。
        yaw = current[2] + wrap(yaw - current[2])
        return (x, y, yaw), frame

    def _publish_feedback(self, goal_handle, start_wall, target):
        try:
            x, y, yaw, transform = self._current_pose()
        except Exception:
            return
        feedback = NavigateToPose.Feedback()
        feedback.current_pose.header = transform.header
        feedback.current_pose.header.frame_id = 'odom'
        feedback.current_pose.pose.position.x = x
        feedback.current_pose.pose.position.y = y
        feedback.current_pose.pose.orientation = transform.transform.rotation
        distance = math.hypot(target[0] - x, target[1] - y)
        yaw_error = abs(wrap(target[2] - yaw))
        feedback.distance_remaining = float(distance)
        feedback.number_of_recoveries = 0
        elapsed = max(0.0, time.monotonic() - start_wall)
        feedback.navigation_time.sec = int(elapsed)
        feedback.navigation_time.nanosec = int(
            (elapsed - int(elapsed)) * 1e9)
        remaining = distance / 0.25 + yaw_error / 0.6
        feedback.estimated_time_remaining.sec = int(remaining)
        feedback.estimated_time_remaining.nanosec = int(
            (remaining - int(remaining)) * 1e9)
        goal_handle.publish_feedback(feedback)

    def _execute(self, goal_handle):
        result = NavigateToPose.Result()
        try:
            current = self._current_pose()
            target, frame = self._target(goal_handle.request, current)
        except Exception as exc:
            self.get_logger().error(str(exc))
            goal_handle.abort()
            return result

        if not self._trajectory.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('base trajectory action is unavailable')
            goal_handle.abort()
            return result

        distance = math.hypot(target[0] - current[0], target[1] - current[1])
        yaw_distance = abs(target[2] - current[2])
        duration = max(1.0, distance / 0.25, yaw_distance / 0.6)
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = [
            'odom_x', 'odom_y', 'odom_t']
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(
            (duration - int(duration)) * 1e9)
        trajectory_goal.trajectory.points = [point]

        self.get_logger().info(
            'goal frame={} target=({:.4f}, {:.4f}, {:.4f}) duration={:.2f}s'
            .format(frame, target[0], target[1], target[2], duration))
        send_future = self._trajectory.send_goal_async(trajectory_goal)
        if not self._wait_future(send_future, 10.0):
            self.get_logger().error('timed out sending base trajectory')
            goal_handle.abort()
            return result
        trajectory_handle = send_future.result()
        if trajectory_handle is None or not trajectory_handle.accepted:
            self.get_logger().error('base trajectory was rejected')
            goal_handle.abort()
            return result

        result_future = trajectory_handle.get_result_async()
        start_wall = time.monotonic()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                trajectory_handle.cancel_goal_async()
                goal_handle.canceled()
                return result
            self._publish_feedback(goal_handle, start_wall, target)
            time.sleep(0.05)

        wrapped_result = result_future.result()
        if (wrapped_result is not None
                and wrapped_result.status == GoalStatus.STATUS_SUCCEEDED):
            goal_handle.succeed()
            self.get_logger().info('goal succeeded')
        else:
            status = wrapped_result.status if wrapped_result else 'no result'
            self.get_logger().error(
                'base trajectory failed: status={}'.format(status))
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = NavigateToPoseBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
