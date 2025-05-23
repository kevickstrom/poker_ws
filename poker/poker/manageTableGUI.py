#!/usr/bin/env python3

# This will likely be unused.
#
#
#


import rclpy
from rclpy.node import Node
from poker_msgs.msg import GameState
import pygame, sys

BACKGROUND = (30, 92, 58)
WINDOW_WIDTH = 1800
WINDOW_HEIGHT = 500

class manageTableGUI(Node):
    def __init__(self):
        super().__init__('table_manager')
        self.gameStateSub = self.create_subscription(GameState, 'game_state', self.gameState_cb, 10)
        self.gameState = None

        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Table Manager")

    def gameState_cb(self, gameState):
        self.gameState = gameState

    def run(self):
        pyClock = pygame.time.Clock()

        while rclpy.ok():
            # rclpy spin
            rclpy.spin_once(self, timeout_sec=0.1)
            # get inputs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rclpy.shutdown()
                    pygame.quit()
                    sys.exit()

            self.screen.fill(BACKGROUND)
            pygame.display.update()

            pyClock.tick(60)

def main():
    rclpy.init()
    pygame.init()
    gui = manageTableGUI()
    gui.run()

if __name__ == '__main__':
    main()