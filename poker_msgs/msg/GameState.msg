int32 seats
Player[10] active_players
Player[] afk_players

string[] table_cards
float32 pot
# indicate when betting is done
bool pot_good
string winner
float32 big_blind

int32 dealer_index
int32 action_on
int32 hands_played

# 'waiting', 'preflop', 'flop', 'turn', 'river', 'finished'
# 0, 1, 2, 3, 4, 5
int hand_state