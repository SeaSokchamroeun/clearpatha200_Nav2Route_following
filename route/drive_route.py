#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeAndTrackRoute, FollowPath

NAMESPACE     = 'a200_1103'
ROUTE_ACTION  = f'/{NAMESPACE}/compute_and_track_route'
FOLLOW_ACTION = f'/{NAMESPACE}/follow_path'


class RouteDriver(Node):
    def __init__(self, args):
        super().__init__('route_driver')
        self._args          = args
        self._route_client  = ActionClient(self, ComputeAndTrackRoute, ROUTE_ACTION)
        self._follow_client = ActionClient(self, FollowPath, FOLLOW_ACTION)
        self._path_sent     = False

    def send_route_goal(self):
        self.get_logger().info(f'Waiting for {ROUTE_ACTION}...')
        self._route_client.wait_for_server()
        goal = ComputeAndTrackRoute.Goal()
        goal.start_id  = self._args.start
        goal.goal_id   = self._args.end
        goal.use_start = False
        goal.use_poses = False
        self.get_logger().info(f'Requesting route: node {self._args.start} -> node {self._args.end}')
        future = self._route_client.send_goal_async(goal, feedback_callback=self._on_route_feedback)
        future.add_done_callback(self._on_route_response)

    def _on_route_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Route goal REJECTED')
            rclpy.shutdown()
            return
        self.get_logger().info('Route goal ACCEPTED — tracking...')
        handle.get_result_async().add_done_callback(self._on_route_result)

    def _on_route_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'  -> edge={fb.current_edge_id} next_node={fb.next_node_id} path_poses={len(fb.path.poses)}'
        )
        if not self._path_sent and len(fb.path.poses) > 0:
            self._path_sent = True
            self.get_logger().info('Path received — sending to FollowPath controller...')
            self._send_follow_path(fb.path)

    def _send_follow_path(self, path):
        self._follow_client.wait_for_server()
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ''
        goal.goal_checker_id = ''
        future = self._follow_client.send_goal_async(goal, feedback_callback=self._on_follow_feedback)
        future.add_done_callback(self._on_follow_response)

    def _on_follow_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowPath goal REJECTED')
            return
        self.get_logger().info('FollowPath goal ACCEPTED — robot should now move')
        handle.get_result_async().add_done_callback(self._on_follow_result)

    def _on_follow_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'  [drive] distance_to_goal={fb.distance_to_goal:.2f} speed={fb.speed:.2f}')

    def _on_follow_result(self, future):
        self.get_logger().info('FollowPath finished')

    def _on_route_result(self, future):
        try:
            status = future.result().status
            labels = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            self.get_logger().info(f'Route tracking finished: {labels.get(status, f"status={status}")}')
        except Exception as e:
            self.get_logger().error(f'Result error: {e}')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--end',   type=int, default=2)
    args = parser.parse_args()
    rclpy.init()
    node = RouteDriver(args)
    node.send_route_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()