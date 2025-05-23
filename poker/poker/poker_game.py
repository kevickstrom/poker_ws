#!/usr/bin/env python3

# poker_console.py
#
# Kyle Vickstrom
#
# Poker game manager node.
# Publishes GameState every time there is a change.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
import curses

from poker_msgs.msg import GameLog, GameState, Player
from poker_msgs.srv import NewGame, PlayerTurn

class PokerGame(Node):
    def __init__(self):
        super().__init__('poker_game')
        self.logging = True
        self.GameState = None
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.pubGame = self.create_publisher(GameState, 'game_state', 10)
        self.newGame_srv = self.create_service(NewGame, 'new_game', self.newGame)
        self.playerTurn_srv = self.create_service(PlayerTurn, 'player_turn', self.playerTurn)

    def newGame(self, request, response):
        """
        This clears the game state and resets.
        Service Server.

        Starts a completely new table with blank players
        """
        newGame = GameState()
        newGame.seats = 10
        # init player list of nobodies
        player_list = []
        for i in range(newGame.seats):
            nullPlayer = Player()
            nullPlayer.name = "Empty"
            nullPlayer.seat_pos = i
            nullPlayer.stack = 0.0
            nullPlayer.buy_in = 0.0
            nullPlayer.afk = True
            player_list.append(nullPlayer)
        newGame.active_players = player_list

        newGame.dealer_index = 0
        newGame.hand_state = 'waiting'

        self.GameState = newGame

        self.pubGame.publish(newGame)
        self.log("New game created.")

        return response


    def playerTurn(self, request, response):
        """
        Service call for player to mark a new location
        Respondes true / false for valid move or not
        publishes updated GameState msg
        """
        # TODO: Rewrite for poker actions bet, call, fold
        '''
        # check for correct id
        if request.player_id != self.GameState.turn:
            self.log("Wrong player!")
            response.valid = False
            return response

        if request.location >= 9:
            self.log("Invalid location!")
            response.valid = False
            return response

        # check for valid move
        if self.GameState.board[request.location] != -1:
            # space is not blank
            self.log("Space already occupied!")
            response.valid = False
            return response

        self.GameState.board[request.location] = self.GameState.turn
        self.GameState.num_turns += 1
        self.log(f"Placed on {request.location}.")
        # TODO: Check for win condition here?

        # advance turn
        if self.GameState.turn == 0:
            self.GameState.turn = 1
            # TODO: Trigger animation / speech here?
            self.log("Your turn!")
        else:
            # TODO: Also animation here
            self.GameState.turn = 0
            self.log("My turn!")

        # publish updated game state
        self.pubGame.publish(self.GameState)

        response.valid = True
        '''

        return response

        

        # set move

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

def createGame(args=None):
    rclpy.init(args=args)
    game = PokerGame()
    rclpy.spin(game)
    rclpy.shutdown()

if __name__ == "__main__":
    createGame()