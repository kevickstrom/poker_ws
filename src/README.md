### Kyle Vickstrom Live Poker Hand Tracker ROB499

This is meant to be a project for ROB499 which I can use outside the class.  
Using ROS2, this will visualize the current state of a poker table, allowing for easy game management and tracking.  

This should be designed in such a way that physical sensors (cameras) can autmate inputting user hands and the flop, turn and river.

One window will be for game management where the table and gameplay can be configured, edited, saved.  
Another window will visualize the state of the table--visible cards, those in the hand, whose turn it is, pot size, etc.  

Current progress:
   Game will be played via console UI (cmd like inputs)

   Next step:
   Arduino player action puck (3 button inputs)
   Camera, picture, ocr nodes for table cards