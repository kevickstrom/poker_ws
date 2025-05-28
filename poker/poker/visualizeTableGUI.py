#!/usr/bin/env python3

#
#
#
#


import rclpy
from rclpy.node import Node
from poker_msgs.msg import GameState

from ament_index_python.packages import get_package_share_directory


import pygame, sys, os

BACKGROUND = (30, 92, 58)
WINDOW_WIDTH = 1800
WINDOW_HEIGHT = 600

CARD_W, CARD_H = 150, 120
NUM_BACKS = 9
NUM_FACE_CARDS = 3 * 4 # K Q J
NUM_LOW_CARDS = 10 * 4 # A - 10
TOTAL_CARDS = NUM_BACKS+NUM_FACE_CARDS+NUM_LOW_CARDS


class visualizeTableGUI(Node):
    def __init__(self):
        super().__init__('table_visualizer')
        self.gameStateSub = self.create_subscription(GameState, 'game_state', self.gameState_cb, 10)
        self.GameState = None

        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.player_font = pygame.font.SysFont("Verdana", 18, bold=True)
        self.name_color = (245, 245, 245) # grayish white
        self.stack_color = (255, 215, 0) # gold tone
        self.init_images()
        pygame.display.set_caption("Table Visualizer")

    def init_images(self):
        """
        Set up background, card sheet
        """
        pkg_path = get_package_share_directory('poker')
        card_path = os.path.join(pkg_path, 'assets/', "cards-Sheet.png")
        table_path = os.path.join(pkg_path, 'assets/', "table_bg.png")
        sheet = pygame.image.load(card_path).convert_alpha()
        self.table_bg = pygame.image.load(table_path)
        self.cards = []
        for i in range(TOTAL_CARDS):
            rect = pygame.Rect(i * CARD_W, 0, CARD_W, CARD_H)
            self.cards.append(sheet.subsurface(rect).copy())

        self.card_map = {"red_one":self.cards[0],
                        "blue_one":self.cards[1],
                        "gray_one":self.cards[2],
                        "red_deck":self.cards[3],
                        "blue_deck":self.cards[4],
                        "gray_deck":self.cards[5],
                        "red_two":self.cards[6],
                        "blue_two":self.cards[7],
                        "gray_two":self.cards[8]}

        # face cards
        faces = [('k',4),('q',4),('j',4)]
        suits = ['s','c','d','h']
        start = NUM_BACKS
        for face, count in faces:
            for si, suit in enumerate(suits):
                card = f'{face}{suit}'
                self.card_map[card] = self.cards[start + si]
            start += count

        # A-10
        vals = ['a'] + [str(n) for n in range(2,11)]
        for vi, val in enumerate(vals):
            for si, suit in enumerate(suits):
                card = f"{val}{suit}"
                idx = start + vi*4 + si
                self.card_map[card] = self.cards[idx]

        
        # x y coords to draw players at
        # center the cards here
        self.player_locations = [
                (900, 80),     # seat 0 top center
                (1200, 80),    # seat 1 top right
                (1480, 200),    # seat 2 right upper
                (1480, 375),    # seat 3 right lower
                (1200, 510),    # seat 4 bottom right
                (900, 510),     # seat 5 bottom center
                (500, 510),     # seat 6 bottom left
                (320, 375),     # seat 7 left lower
                (320, 200),     # seat 8 left upper
                (500, 80),     # seat 9 top left
        ]

        # x y coords to draw player names
        self.name_locations = [
                (980, 35),     # seat 0 top center
                (1280, 35),    # seat 1 top right   # bottom right te
                (1560, 155),    # seat 2 right upper
                (1560, 330),    # seat 3 right lower
                (1280, 510),    # seat 4 bottom right
                (980, 510),     # seat 5 bottom center
                (580, 510),     # seat 6 bottom left
                (250, 330),     # seat 7 left lower
                (250, 155),     # seat 8 left upper
                (580, 35),     # seat 9 top left

        ]


    def gameState_cb(self, gameState):
        self.GameState = gameState

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

            #self.screen.fill(BACKGROUND)
            self.screen.blit(self.table_bg, (0,0))
            self.drawPlayers()
            self.drawCards()
            pygame.display.update()

            pyClock.tick(60)

    def drawPlayers(self):
        """
        Draw the player cards, stack size on the table
        """
        if self.GameState is not None:
            # draw players
            for i, p in enumerate(self.GameState.active_players):
                if p.name != "Empty":
                    # draw player name, stack size and buy in
                    name_surf = self.player_font.render(p.name, True, self.name_color)
                    stack_surf = self.player_font.render(f"Stack: {p.stack}", True, self.stack_color)

                    if i < 4 or i == 9:
                        # grab the bottom left
                        name_rect = name_surf.get_rect(bottomleft=self.name_locations[i])
                        stack_rect = stack_surf.get_rect(topleft=(self.name_locations[i][0], self.name_locations[i][1]+10))
                    elif i < 7:
                        # grab the top left
                        name_rect = name_surf.get_rect(topleft=self.name_locations[i])
                        stack_rect = stack_surf.get_rect(topleft=(self.name_locations[i][0], self.name_locations[i][1]+25))
                    else:
                        # grab the bottom right
                        name_rect = name_surf.get_rect(bottomright=self.name_locations[i])
                        stack_rect = stack_surf.get_rect(bottomright=(self.name_locations[i][0], self.name_locations[i][1]+25))
                    self.screen.blit(name_surf, name_rect)
                    self.screen.blit(stack_surf, stack_rect)
                    
                    # draw player stack

                    # draw cards
                    if p.in_hand:
                        if self.GameState.hand_state == 1 or self.GameState.hand_state == 5:
                            # preflop and finished
                            hand = self.card_map["blue_two"]
                        elif self.GameState.hand_state == 0:
                            # waiting state
                            hand = self.card_map["gray_two"]
                        else:
                            # hand in play
                            hand = self.card_map["red_two"]
                    else:
                        # draw gray cards
                        hand = self.card_map["gray_two"]
                    self.screen.blit(hand, hand.get_rect(center=self.player_locations[i]))

    def drawCards(self):
        """
        Draw the table cards
        """
        if self.GameState is not None:
            # draw the deck
            # waiting - gray deck
            if self.GameState.hand_state == 0:
                deck = self.card_map["gray_deck"]
            elif self.GameState.hand_state == 1:
                # preflop - blue deck
                deck = self.card_map["blue_deck"]
            else:
                # in hand so grab color deck
                deck = self.card_map["red_deck"]

            deck_x = 900-3*CARD_W/2
            deck_y = 300
            self.screen.blit(deck, deck.get_rect(center=(deck_x, deck_y)))

            # draw table cards
            if self.GameState.hand_state > 1:
                for i, card_str in enumerate(self.GameState.table_cards):
                    if card_str == "none":
                        continue
                    card_img = self.card_map[card_str]
                    self.screen.blit(card_img, deck.get_rect(center=(deck_x+i*CARD_W/2+CARD_W, deck_y)))


def main():
    rclpy.init()
    pygame.init()
    gui = visualizeTableGUI()
    gui.run()

if __name__ == '__main__':
    main()