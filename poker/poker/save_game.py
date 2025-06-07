#!/usr/bin/env python3

import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from poker_msgs.msg import GameState

class GameStateLogger(Node):
    def __init__(self):
        super().__init__('game_state_logger')
        self.prev_msg = None

        # Determine and prepare assets directory for CSV output
        share_dir = get_package_share_directory('poker')
        assets_dir = os.path.join(share_dir, 'assets')
        os.makedirs(assets_dir, exist_ok=True)
        self.csv_path = os.path.join(assets_dir, 'state_changes.csv')

        # Create CSV with header if it doesn't exist
        if not os.path.isfile(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'field', 'old_value', 'new_value'])

        # Subscribe to GameState topic
        self.subscription = self.create_subscription(
            GameState,
            'game_state',
            self.game_state_callback,
            10
        )

    def game_state_callback(self, msg: GameState):
        changes = []

        # Compare each field in the message to the previous message
        if self.prev_msg is not None:
            for field_name, _ in msg._fields_and_field_types.items():
                old_value = getattr(self.prev_msg, field_name)
                new_value = getattr(msg, field_name)
                if old_value != new_value:
                    changes.append((field_name, old_value, new_value))

        # Record changes to CSV
        if changes:
            timestamp = datetime.now().isoformat()
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                for field, old, new in changes:
                    writer.writerow([timestamp, field, str(old), str(new)])

        # Update previous message reference
        self.prev_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = GameStateLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()