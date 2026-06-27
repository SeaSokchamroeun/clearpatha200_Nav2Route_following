#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeAndTrackRoute

NAMESPACE = 'a200_1103'
ACTION    = f'/{NAMESPACE}/compute_and_track_route'

class RouteGoalSender(Node):
    def __init__(self, args):
        super().__init__('route_goal_sender')
        self._args   = args
        self._client = ActionClient(self, ComputeAndTrackRoute, ACTION)

    def send(self):
        self.get_logger().info(f'Waiting for action server: {ACTION}')
        self._client.wait_for_server()
        goal = ComputeAndTrackRoute.Goal()
        goal.start_id  = self._args.start
        goal.goal_id   = self._args.end
        goal.use_start = False
        self.get_logger().info(f'Sending route: node {self._args.start} -> node {self._args.end}')
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal ACCEPTED — Husky is navigating...')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        try:
            fb = msg.feedback
            info = []
            for attr in ['current_edge_id', 'distance_remaining', 'number_of_edges_remaining']:
                if hasattr(fb, attr):
                    info.append(f'{attr}={getattr(fb, attr)}')
            self.get_logger().info('  -> ' + ('  '.join(info) if info else 'feedback received'))
        except Exception:
            pass

    def _on_result(self, future):
        try:
            status = future.result().status
            labels = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            self.get_logger().info(f'Route finished: {labels.get(status, f"status={status}")}')
        except Exception as e:
            self.get_logger().error(f'Result error: {e}')
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--end',   type=int, default=3)
    args = parser.parse_args()
    rclpy.init()
    node = RouteGoalSender(args)
    node.send()
    rclpy.spin(node)

if __name__ == '__main__':
    main()