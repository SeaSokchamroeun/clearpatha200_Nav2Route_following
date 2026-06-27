#!/usr/bin/env python3
"""
route_loop_eval.py (v2) — Drives the full loop (0->1->2->3->0) for N laps
using a single persistent node across all laps. Cancels BOTH the route
goal and the follow goal on any timeout, since route_server keeps
tracking (and emitting feedback) until explicitly told to stop.
"""
import argparse, json, math, os, sys, time
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
LOOP_EDGES    = [(0, 1), (1, 2), (2, 3), (3, 0)]


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LoopEvaluator(Node):
    def __init__(self, logdir, edge_timeout):
        super().__init__('route_loop_evaluator')
        self.logdir, self.edge_timeout = logdir, edge_timeout
        self._route_client = ActionClient(self, ComputeAndTrackRoute, ROUTE_ACTION)
        self._follow_client = ActionClient(self, FollowPath, FOLLOW_ACTION)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(TwistStamped, VEL_TOPIC, self._on_vel, 20)
        self.create_timer(0.1, self._on_tf_timer)
        self._t0 = time.time()
        self._reset_lap_state()

    def _reset_lap_state(self):
        self.trajectory, self.velocity, self.reference_path = [], [], []
        self.node_log, self.edge_results = [], []
        self._route_handle, self._follow_handle = None, None

    def _on_tf_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        t = time.time() - self._t0
        self.trajectory.append({'t': t, 'x': tf.transform.translation.x,
                                 'y': tf.transform.translation.y,
                                 'yaw': yaw_from_quat(tf.transform.rotation)})

    def _on_vel(self, msg):
        t = time.time() - self._t0
        self.velocity.append({'t': t, 'vx': msg.twist.linear.x, 'wz': msg.twist.angular.z})

    def run_lap(self, lap_index):
        self._reset_lap_state()
        self.get_logger().info(f'[lap {lap_index}] starting full loop 0->1->2->3->0')
        node_positions = {}
        for (s, e) in LOOP_EDGES:
            ok = self._run_edge(lap_index, s, e, node_positions)
            self.edge_results.append({'edge': f'{s}->{e}', 'result': self._edge_result})
            if not ok:
                self.get_logger().warn(f'[lap {lap_index}] edge {s}->{e} did not succeed '
                                        f'({self._edge_result}) — cancelling and continuing')
                self._cancel_outstanding()
                t_settle = time.time()
                while rclpy.ok() and time.time() - t_settle < 1.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
        self.lap_result = 'SUCCEEDED' if all(r['result'] == 'SUCCEEDED' for r in self.edge_results) else 'PARTIAL_FAILURE'
        self.get_logger().info(f'[lap {lap_index}] lap finished: {self.lap_result}')

    def _run_edge(self, lap_index, start_id, end_id, node_positions):
        self._edge_done, self._edge_result, self._path_sent = False, 'UNKNOWN', False
        self._route_handle, self._follow_handle = None, None

        if not self._route_client.wait_for_server(timeout_sec=5.0):
            self._edge_result = 'ROUTE_SERVER_UNAVAILABLE'
            return False

        goal = ComputeAndTrackRoute.Goal()
        goal.start_id, goal.goal_id = start_id, end_id
        goal.use_start, goal.use_poses = False, False
        self.get_logger().info(f'[lap {lap_index}] edge {start_id} -> {end_id}')

        future = self._route_client.send_goal_async(
            goal, feedback_callback=lambda m: self._on_route_feedback(m, lap_index, end_id, node_positions))
        t0 = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > self.edge_timeout:
                self._edge_result = 'ROUTE_TIMEOUT'
                return False

        handle = future.result()
        if not handle.accepted:
            self._edge_result = 'ROUTE_REJECTED'
            return False
        self._route_handle = handle

        t0 = time.time()
        while rclpy.ok() and not self._edge_done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > self.edge_timeout:
                self._edge_result = 'TIMEOUT'
                break
        return self._edge_result == 'SUCCEEDED'

    def _cancel_outstanding(self):
        if self._follow_handle is not None:
            try:
                self._follow_handle.cancel_goal_async()
            except Exception:
                pass
        if self._route_handle is not None:
            try:
                self._route_handle.cancel_goal_async()
            except Exception:
                pass

    def _on_route_feedback(self, msg, lap_index, end_id, node_positions):
        fb = msg.feedback
        for n in fb.route.nodes:
            node_positions.setdefault(n.nodeid, (n.position.x, n.position.y))
        if not self._path_sent and len(fb.path.poses) > 0:
            self._path_sent = True
            seg = [[ps.pose.position.x, ps.pose.position.y] for ps in fb.path.poses]
            self.reference_path.extend(seg)
            self.get_logger().info(f'[lap {lap_index}]   path: {len(seg)} pts -> FollowPath')
            self._send_follow_path(fb.path, end_id, node_positions)

    def _send_follow_path(self, path, end_id, node_positions):
        if not self._follow_client.wait_for_server(timeout_sec=5.0):
            self._edge_result, self._edge_done = 'FOLLOW_SERVER_UNAVAILABLE', True
            return
        goal = FollowPath.Goal()
        goal.path, goal.controller_id, goal.goal_checker_id = path, '', ''
        future = self._follow_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_follow_response(f, end_id, node_positions))

    def _on_follow_response(self, future, end_id, node_positions):
        try:
            handle = future.result()
        except Exception as e:
            self._edge_result, self._edge_done = f'FOLLOW_SEND_ERROR:{e}', True
            return
        self._follow_handle = handle
        if not handle.accepted:
            self._edge_result, self._edge_done = 'FOLLOW_REJECTED', True
            return
        handle.get_result_async().add_done_callback(lambda f: self._on_follow_result(f, end_id, node_positions))

    def _on_follow_result(self, future, end_id, node_positions):
        try:
            err = future.result().result.error_code
            self._edge_result = 'SUCCEEDED' if err == 0 else f'FOLLOW_ERROR_{err}'
        except Exception as e:
            self._edge_result = f'ERROR:{e}'
        t_arrival = time.time() - self._t0
        if end_id in node_positions:
            x, y = node_positions[end_id]
            self.node_log.append({'id': end_id, 'x': x, 'y': y, 't_arrival': t_arrival})
        self._edge_done = True

    def save(self, lap_index):
        os.makedirs(self.logdir, exist_ok=True)
        fname = os.path.join(self.logdir, f'lap_{lap_index:03d}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        with open(fname, 'w') as f:
            json.dump({'lap_index': lap_index, 'lap_result': self.lap_result,
                       'edge_results': self.edge_results, 'node_log': self.node_log,
                       'reference_path': self.reference_path, 'trajectory': self.trajectory,
                       'velocity': self.velocity}, f)
        self.get_logger().info(f'[lap {lap_index}] saved log: {fname}')


def main():
    rclpy.init()
    parsed = remove_ros_args(args=sys.argv)[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--logdir', type=str, default=os.path.expanduser('~/clearpath/eval_logs'))
    parser.add_argument('--edge-timeout', type=float, default=45.0)
    parser.add_argument('--settle', type=float, default=1.5)
    args = parser.parse_args(parsed)

    node = LoopEvaluator(args.logdir, args.edge_timeout)
    for i in range(1, args.laps + 1):
        node.run_lap(i)
        node.save(i)
        time.sleep(args.settle)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()