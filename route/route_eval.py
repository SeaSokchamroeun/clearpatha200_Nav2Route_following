#!/usr/bin/env python3
"""
route_eval.py — Drives a route N times and logs pose (via TF), commanded
velocity, and the reference path for later performance evaluation.

Run with the TF namespace remap, e.g.:

  python3 route_eval.py --start 0 --end 2 --runs 10 \
    --ros-args -r /tf:=/a200_1103/tf -r /tf_static:=/a200_1103/tf_static \
    -p use_sim_time:=true
"""
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from nav2_msgs.action import ComputeAndTrackRoute, FollowPath
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

NAMESPACE     = 'a200_1103'
ROUTE_ACTION  = f'/{NAMESPACE}/compute_and_track_route'
FOLLOW_ACTION = f'/{NAMESPACE}/follow_path'
VEL_TOPIC     = f'/{NAMESPACE}/cmd_vel_nav'


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RouteEvalRun(Node):
    def __init__(self, start_id, end_id, run_index, logdir):
        super().__init__(f'route_eval_run_{run_index}')
        self.start_id, self.end_id, self.run_index, self.logdir = start_id, end_id, run_index, logdir

        self._route_client = ActionClient(self, ComputeAndTrackRoute, ROUTE_ACTION)
        self._follow_client = ActionClient(self, FollowPath, FOLLOW_ACTION)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trajectory = []
        self.velocity = []
        self.reference_path = []
        self.route_nodes = []
        self.result = 'UNKNOWN'
        self._path_sent = False
        self._t0 = time.time()
        self._done = False

        self.create_subscription(TwistStamped, VEL_TOPIC, self._on_vel, 20)
        self.create_timer(0.1, self._on_tf_timer)  # 10 Hz pose logging

    def _on_tf_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        t = time.time() - self._t0
        self.trajectory.append({
            't': t,
            'x': tf.transform.translation.x,
            'y': tf.transform.translation.y,
            'yaw': yaw_from_quat(tf.transform.rotation),
        })

    def _on_vel(self, msg):
        t = time.time() - self._t0
        self.velocity.append({'t': t, 'vx': msg.twist.linear.x, 'wz': msg.twist.angular.z})

    def start(self):
        self.get_logger().info(f'[run {self.run_index}] waiting for route server...')
        self._route_client.wait_for_server()
        goal = ComputeAndTrackRoute.Goal()
        goal.start_id, goal.goal_id = self.start_id, self.end_id
        goal.use_start = False
        goal.use_poses = False
        self.get_logger().info(f'[run {self.run_index}] requesting route {self.start_id} -> {self.end_id}')
        future = self._route_client.send_goal_async(goal, feedback_callback=self._on_route_feedback)
        future.add_done_callback(self._on_route_response)

    def _on_route_feedback(self, msg):
        fb = msg.feedback
        if not self.route_nodes and len(fb.route.nodes) > 0:
            self.route_nodes = [{'id': n.nodeid, 'x': n.position.x, 'y': n.position.y} for n in fb.route.nodes]
        if not self._path_sent and len(fb.path.poses) > 0:
            self.reference_path = [[ps.pose.position.x, ps.pose.position.y] for ps in fb.path.poses]
            self._path_sent = True
            self.get_logger().info(f'[run {self.run_index}] path received ({len(self.reference_path)} pts) -> FollowPath')
            self._send_follow_path(fb.path)

    def _on_route_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'[run {self.run_index}] route goal REJECTED')
            self.result, self._done = 'ROUTE_REJECTED', True
            return
        handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_result(self, future):
        try:
            status = future.result().status
            labels = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            self.get_logger().info(f"[run {self.run_index}] route tracking finished: {labels.get(status, status)}")
        except Exception as e:
            self.get_logger().error(f'[run {self.run_index}] route result error: {e}')

    def _send_follow_path(self, path):
        self._follow_client.wait_for_server()
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ''
        goal.goal_checker_id = ''
        future = self._follow_client.send_goal_async(goal)
        future.add_done_callback(self._on_follow_response)

    def _on_follow_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'[run {self.run_index}] FollowPath REJECTED')
            self.result, self._done = 'FOLLOW_REJECTED', True
            return
        handle.get_result_async().add_done_callback(self._on_follow_result)

    def _on_follow_result(self, future):
        try:
            err = future.result().result.error_code
            self.result = 'SUCCEEDED' if err == 0 else f'FOLLOW_ERROR_{err}'
        except Exception as e:
            self.result = f'ERROR:{e}'
        self.get_logger().info(f'[run {self.run_index}] FollowPath finished: {self.result}')
        self._done = True

    def is_done(self):
        return self._done

    def save(self):
        os.makedirs(self.logdir, exist_ok=True)
        fname = os.path.join(
            self.logdir,
            f'run_{self.run_index:03d}_{self.start_id}to{self.end_id}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        )
        with open(fname, 'w') as f:
            json.dump({
                'run_index': self.run_index, 'start_id': self.start_id, 'end_id': self.end_id,
                'result': self.result, 'route_nodes': self.route_nodes,
                'reference_path': self.reference_path, 'trajectory': self.trajectory,
                'velocity': self.velocity,
            }, f)
        self.get_logger().info(f'[run {self.run_index}] saved log: {fname}')


def main():
    rclpy.init()
    parsed = remove_ros_args(args=sys.argv)[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--end', type=int, default=2)
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--logdir', type=str, default=os.path.expanduser('~/clearpath/eval_logs'))
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args(parsed)

    for i in range(1, args.runs + 1):
        node = RouteEvalRun(args.start, args.end, i, args.logdir)
        node.start()
        t_start = time.time()
        while rclpy.ok() and not node.is_done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - t_start > args.timeout:
                node.result = 'TIMEOUT'
                break
        node.save()
        node.destroy_node()
        time.sleep(1.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()