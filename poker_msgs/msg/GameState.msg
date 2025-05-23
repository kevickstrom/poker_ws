int32 seats
Player[10] active_players
Player[] afk_players

string[] table_cards
float32 pot
# indicate when betting is done
bool pot_good

int32 dealer_index
int32 action_on
int32 hands_played

# 'waiting', 'preflop', 'flop', 'turn', 'river'
string hand_state