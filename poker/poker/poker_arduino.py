#!/usr/bin/env python3

# poker_arduino.py
#
# Kyle Vickstrom
#
# Poker arduino node
# this sends and receives serial data from an arduino that the player uses to fold, call bet

import rclpy
from rclpy.node import Node


from poker_msgs.srv import PlayerTurn
from poker_msgs.msg import GameLog, GameState

import serial

class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__('poker_arduino')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.GameState = None
        self.last_state = 0
        self.last_action = -1

        self.newTurn_client = self.create_client(PlayerTurn, 'player_turn')
        self.subGame = self.create_subscription(GameState, 'game_state', self.readGameState, 10)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for player_turn service...')

        self.timer = self.create_timer(0.5, self.timer_callback)


    def timer_callback(self):
        if self.GameState is None:
            # update max bet threshold if in betting stage
            if self.Gamestate.hand_state > 0 and self.last_action != self.GameState.action_on:
                self.send_max_value_to_arduino(self.Gamestate.active_players[self.Gamestate.action_on].stack)
                # update last action so we dont send serial again and just listen until action selected
                self.last_action = self.GameState.action_on
            return
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            self.log(f"Arduino: {line}")
            if "state" in line and "size" in line:
                try:
                    parts = dict(part.split(':') for part in line.split(','))
                    state = int(parts["state"])
                    size = float(parts["size"])
                    # turn state num into string or unknown
                    action = {1: "fold", 2: "call", 3: "bet"}.get(state, "unknown")

                    if action != "unknown":
                        self.call_bet_service(action, size)
                    else:
                        self.log(f"Unknown state: {state}")
                except Exception as e:
                    self.log(f"Parse error: {e}")

    def send_player_turn(self, action, amount):
        request = PlayerTurn.Request()
        request.action = action
        request.amount = amount

        future = self.cli.call_async(request)

    def send_max_value_to_arduino(self, val: float):
    try:
        message = f"{val:.2f}\n"
        self.ser.write(message.encode('utf-8'))
        self.log(f"Sent to Arduino: {message.strip()}")
    except Exception as e:
        self.log(f"Serial write error: {e}")

    def readGameState(self, msg: GameState):
        """
        Subscriber callback to the GameState topic
        Update last vars
        """
        self.last_state = self.GameState.hand_state
        if self.last_state > 0:
            self.last_action = self.GameState.action_on
        self.GameState = msg

    def log(self, msg):
        """
        log to topic and to curses terminal window
        """
        if self.logging:
            newmsg = GameLog()
            newmsg.stamp = self.get_clock().now().to_msg()
            newmsg.node_name = self.get_name()
            newmsg.content = msg
            self.pubLog.publish(newmsg)
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
