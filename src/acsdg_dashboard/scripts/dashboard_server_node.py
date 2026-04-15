#!/usr/bin/env python3
"""
dashboard_server_node.py — Serves the built React app on port 3000.

Finds the web/build directory relative to the installed share path,
then starts a simple HTTP server in a background thread.
"""

import os
import pathlib
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

PORT = 3000


class DashboardServerNode(Node):
    def __init__(self) -> None:
        super().__init__('dashboard_server_node')

        # Resolve path via ament index
        try:
            share_dir = pathlib.Path(get_package_share_directory('acsdg_dashboard')) / 'web'
        except Exception:
            share_dir = pathlib.Path('/nonexistent')

        if not share_dir.exists():
            self.get_logger().error(
                f'React build not found at {share_dir}. '
                'Run: cd ~/acsdg_ws/src/acsdg_dashboard/web && npm run build && colcon build --packages-select acsdg_dashboard')
            return

        os.chdir(str(share_dir))

        handler = SimpleHTTPRequestHandler
        handler.log_message = lambda *a: None   # suppress per-request logs
        server = HTTPServer(('0.0.0.0', PORT), handler)

        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        self.get_logger().info(
            f'Dashboard HTTP server running at http://localhost:{PORT}')
        self.get_logger().info(
            f'Serving from: {share_dir}')

        # Keep-alive timer
        self.create_timer(60.0, lambda: None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
