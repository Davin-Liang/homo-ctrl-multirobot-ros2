#!/usr/bin/env python3
"""Wait for and activate lifecycle nodes (map_server + AMCL)."""

import sys
import time

import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition


def activate_node(node, node_name, timeout=30.0):
    """Configure and activate a lifecycle node."""
    node_name = node_name.lstrip("/")
    configure_srv = f"/{node_name}/change_state"

    # Wait for the change_state service to appear
    start = time.time()
    while time.time() - start < timeout:
        service_names = [s[0] for s in node.get_service_names_and_types()]
        if configure_srv in service_names:
            break
        time.sleep(0.5)
    else:
        print(f"[lifecycle_activator] Timeout waiting for {configure_srv}")
        return False

    # Configure (transition id=1)
    change_state = node.create_client(ChangeState, configure_srv)
    if not change_state.wait_for_service(timeout_sec=5.0):
        print(f"[lifecycle_activator] Cannot reach {configure_srv}")
        return False

    req = ChangeState.Request()
    req.transition = Transition(id=1, label="configure")
    future = change_state.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.result() or not future.result().success:
        print(f"[lifecycle_activator] Configure {node_name} failed"
              f" (maybe already configured)")
    time.sleep(1.0)

    # Activate (transition id=3)
    req.transition = Transition(id=3, label="activate")
    future = change_state.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() and future.result().success:
        print(f"[lifecycle_activator] {node_name} activated successfully")
        return True
    else:
        print(f"[lifecycle_activator] Activate {node_name} failed"
              f" (maybe already active)")
        return True


def main():
    rclpy.init()

    node_names = sys.argv[1:] if len(sys.argv) > 1 else [
        "map_server",
        "robot1/amcl",
    ]

    node = rclpy.create_node("lifecycle_activator")

    print(f"[lifecycle_activator] Activating nodes: {node_names}")
    # Wait a bit for nodes to start
    time.sleep(5.0)

    for name in node_names:
        print(f"[lifecycle_activator] Processing {name}...")
        activate_node(node, name)

    node.destroy_node()
    rclpy.shutdown()
    print("[lifecycle_activator] Done")


if __name__ == "__main__":
    main()
