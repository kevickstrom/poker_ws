# represents a player at the table.
string name
float32 bet_this_hand
float32 stack
float32 buy_in
bool in_hand
bool all_in

# where are they sitting at the table?
int32 seat_pos

# indicate not blind (0), bb (1), sb (2), button (3)
int32 blind

bool afk