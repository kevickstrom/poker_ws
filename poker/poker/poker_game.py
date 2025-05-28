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

from rclpy.parameter import parameter_value_to_python, Parameter
from rclpy.parameter_event_handler import ParameterEventHandler

from rcl_interfaces.msg import SetParametersResult

from poker_msgs.msg import GameLog, GameState, Player
from poker_msgs.srv import NewGame, PlayerTurn, AddPlayer
from poker_msgs.srv import AdvanceHand

class PokerGame(Node):
    def __init__(self):
        super().__init__('poker_game')
        self.logging = True
        self.GameState = None
        #self.declare_parameter("seats", 2)
        #self.seats = 
        self.setup_params()
        # declare params -- GameState built from these
        self.add_on_set_parameters_callback(self.on_param_change)
        self.pubLog = self.create_publisher(GameLog, 'game_log', 10)
        self.pubGame = self.create_publisher(GameState, 'game_state', 10)
        self.newGame_srv = self.create_service(NewGame, 'new_game', self.newGame)
        self.addPlayer_srv = self.create_service(AddPlayer, 'add_player', self.addPlayer)
        self.playerTurn_srv = self.create_service(PlayerTurn, 'player_turn', self.playerTurn)
        self.advanceHand_srv = self.create_service(AdvanceHand, 'advance_hand', self.advanceHand)

    def setup_params(self):
        """
        Set up game parameters that the GameState will build from
        """
        param_list = [
            ("seats", 10),
            ("flop", ["none", "none", "none"]),
            ("turn", "none"),
            ("river", "none")
        ]

        for name, default in param_list:
            self.declare_parameter(name, default)

    def on_param_change(self, params):
        """
        Update params and republish new GameState
        """
        for p in params:
            # build table cards
            table_cards = self.GameState.table_cards.copy()
            if p.name == "flop" and p.type_ == Parameter.Type.STRING_ARRAY:
                for i, card in enumerate(p.value):
                    table_cards[i] = card
            elif p.name == "turn" and p.type_ == Parameter.Type.STRING:
                table_cards[3] = p.value
            elif p.name == "river" and p.type_ == Parameter.Type.STRING:
                table_cards[4] = p.value


        # publish updated game
        self.GameState.table_cards = table_cards
        self.pubGame.publish(self.GameState)
        return SetParametersResult(successful=True, reason='')


        

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
        newGame.afk_players = []
        newGame.table_cards = ["none", "none", "none", "none", "none"]

        newGame.blind_index = [0, 1, 2]
        newGame.hand_state = 0
        newGame.big_blind = 0.20

        self.GameState = newGame

        self.pubGame.publish(newGame)
        self.log("New game created.")

        return response

    def addPlayer(self, request, response):
        """
        Service call to add players to the table.
        If player set to afk, will be added to afk list
        Otherwise will attempt to be placed at requested seat pos
        If seat is already full, request will be denied.
        Proper usage would be to init player to afk list then move players to their correct seats.
        """
        # TODO: init blind 0,1,2,3 based on dealer index
        newPlayer = request.player
        # Add to afk list
        if newPlayer.afk:
            self.GameState.afk_players.append(newPlayer)
            newPlayer.seat_pos = 99
            response.added = True
            self.log(f"Added {newPlayer.name} to afk list with buy in {newPlayer.buy_in}.")
        else:
            # check if seat occupied
            if self.GameState.active_players[request.seat_pos].name == "Empty":
                self.GameState.active_players[request.seat_pos] = newPlayer
                newPlayer.seat_pos = request.seat_pos
                newPlayer.afk = False
                self.log(f"Added {newPlayer.name} to seat {request.seat_pos} with buy in {newPlayer.buy_in}.")
                response.added = True
            else:
                # if occupied add to afk list for user to manually move
                newPlayer.seat_pos = 99
                newPlayer.afk = True
                self.GameState.afk_players.append(newPlayer)
                self.log(f"Seat occupied. Added {newPlayer.name} to afk list with buy in {newPlayer.buy_in}.")
                response.added = False

        # publish the new state
        self.pubGame.publish(self.GameState)
        return response

    def advanceHand(self, request, response):
        """
        Service call to change hand state.
        Give new hand state and the table cards for that state
        """
        # start a new hand waiting -> preflop
        self.log(f"Pre advance state: {self.GameState.hand_state}")
        if self.GameState.hand_state == 0:
            self.log("Starting hand...")
            self.GameState.hand_state = self.GameState.hand_state + 1
            # add all the players to the hand
            for player in self.GameState.active_players:
                player.in_hand = True
        # everyone folded so assign win
        elif request.folded:
            # TODO: Assign win
            # self.GameState.winner =
            self.newHand()
        # advance to the next stage
        elif self.GameState.pot_good or request.force_advance:
            self.log("advancing...")
            # TODO: If camera connected:
            # TODO: Get cards from camera -> this will trigger the param change and auto update the GameState table_cards
            #self.GameState.table_cards = request.table_cards
            # check if finished -> reset for new hand
            if self.GameState.hand_state == 5:
                self.newHand()
                return response
            # check if river -> finished
            elif self.GameState.hand_state == 4:
                pass
                # TODO: assign win
                # self.GameState.winner =
            self.GameState.hand_state = self.GameState.hand_state + 1
            self.GameState.pot_good = False
            self.log(f"Advanced to new state: {self.GameState.hand_state}")
        self.log(f"Post advance state: {self.GameState.hand_state}")
        self.log(f"Table Cards: {self.GameState.table_cards}")
        self.pubGame.publish(self.GameState)
        return response

    def newHand(self):
        """
        Resets self.GameState to be in state 0 (waiting)
        """
        self.log("Resetting for next hand...")
        # reset pot
        self.GameState.pot = self.GameState.big_blind + self.GameState.big_blind/2.0
        # reset table cards
        #self.GameState.table_cards = []
        self.GameState.num_actions = 0
        self.GameState.num_in_hand = 0
        for p in self.GameState.active_players:
            if p.name == "Empty":
                p.in_hand = False
            else:
                self.GameState.num_in_hand += 1
                p.in_hand = True

        self.GameState.hand_state = 0

        #TODO: advance blinds to next non empty seat

        # reset table cards (This will publish the new gamestate)
        new_cards = [
            Parameter("flop", Parameter.Type.STRING_ARRAY, ["none", "none", "none"]),
            Parameter("turn", Parameter.Type.STRING, "none"),
            Parameter("river", Parameter.Type.STRING, "none")
        ]
        self.set_parameters(new_cards)

        

    def playerTurn(self, request, response):
        """
        Service call for player to either bet, call, check.
        """
        action = request.action

        curr_player = self.GameState.active_players[self.GameState.action_on]
        # TODO: check for conditions when stack hits 0
        if action == 'fold':
            curr_player.in_hand = False
            self.GameState.num_in_hand -= 1
            # this is the last player to fold (hand over)
            if self.GameState.in_hand == 0:
                self.GameState.pot_good = True
                # TODO: assign win
                # put into waiting state
                self.GameState = 5
        elif action == 'call':
            # if i've already bet, find the difference
            if curr_player.bet_this_hand > 0:
                need_to_pay = self.GameState.curr_bet - curr_player.bet_this_hand
            else:
                need_to_pay = self.GameState.curr_bet
            # pay the needed amt
            curr_player.bet_this_hand = self.GameState.curr_bet
            curr_player.stack -= need_to_pay
            self.GameState.pot += need_to_pay

        elif action == 'bet':
            # set the new amount
            self.GameState.curr_bet = request.amount
            # now pay, just like a call
            if curr_player.bet_this_hand > 0:
                need_to_pay = self.GameState.curr_bet - curr_player.bet_this_hand
            else:
                need_to_pay = self.GameState.curr_bet
            # pay the needed amt
            curr_player.bet_this_hand = self.GameState.curr_bet
            curr_player.stack -= need_to_pay
            self.GameState.pot += need_to_pay

        self.GameState.num_actions += 1

        # advance the action
        self.GameState.action_on += 1
        if self.GameState.action_on > self.GameState.seats - 1:
            self.GameState.action_on = 0

        # go to next player that hasn't folded
        found_next = self.GameState.active_players[self.GameState.action_on].in_hand
        while not found_next:
            self.GameState.action_on += 1
            found_next = self.GameState.active_players[self.GameState.action_on].in_hand

        # Check to see if bets are all good
        # not the best way method but it works
        all_paid = True
        for p in self.GameState.active_players:
            if (p.in_hand) and (p.bet_this_hand != self.GameState.curr_bet):
                all_paid = False
        self.GameState.pot_good = all_paid

        # TODO: Assign winner here?

        self.pubGame.publish(self.GameState)

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