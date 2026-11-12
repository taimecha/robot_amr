#!/usr/bin/env python3
"""Start a new pallet recognition or pause pallet-only tracking; no motion."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def request(node, topic, enabled, wait_timeout):
    client = node.create_client(SetBool, topic)
    if not client.wait_for_service(timeout_sec=wait_timeout):
        raise RuntimeError(f'missing service {topic}')
    future = client.call_async(SetBool.Request(data=enabled))
    rclpy.spin_until_future_complete(node, future, timeout_sec=wait_timeout)
    if not future.done():
        client.remove_pending_request(future)
        raise RuntimeError(f'{topic} did not answer within {wait_timeout:g} s')
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f'{topic} refused: {response.message if response else "empty response"}')
    return response.message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('recognize', 'pause', 'resume'))
    parser.add_argument('--wait-timeout', type=float, default=3.)
    args = parser.parse_args()
    if not math.isfinite(args.wait_timeout) or args.wait_timeout <= 0:
        parser.error('wait timeout must be finite and positive')
    rclpy.init()
    node = Node('r3_perception_switch')
    try:
        if args.action == 'recognize':
            request(node, '/pallet/r3/pause_target_tracking', False, args.wait_timeout)
            request(node, '/pallet/set_enabled', False, args.wait_timeout)
            time.sleep(.1)
            print(request(node, '/pallet/set_enabled', True, args.wait_timeout), flush=True)
        else:
            print(request(node, '/pallet/r3/pause_target_tracking',
                          args.action == 'pause', args.wait_timeout), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
