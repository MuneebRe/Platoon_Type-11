1 - Mandatory Requirements: Player 1/2 & Robot A/B
The video demonstrates that the robot works well with
Robot A Attack vs Robot B Evade
Robot B Attack vs Robot A Evade
However, using shared memory and twice the vision processing slows down performance and causes some lag, maybe it's due to the PC's system limitation, or maybe that the shared_memory are running on the same thread.

Demo shown in 
Player_1_A_Attack_vs_Player_2_B_Evade.mp4
Player_1_B_Attack_vs_Player_2_A_Evade.mp4

To showcase that they also work no matter the color of the obstacle, and that the robots never fail to identify their obstacles, demo shown in 
Obstacle_Testing_Blue.mp4
Obstacle_Testing_Green.mp4
Obstacle_Testing_Orange.mp4
Obstacle_Testing_Red.mp4

To showcase the behind the scens for the Attack and Evade strategy, demo shown in:
Manual_Attack_vs_Auto_Evade_Strategy.mp4
Manual_Evade_vs_Auto_Attack_Strategy.mp4

2 - Optional Features: Real Robot
Watch Real_Robot [Top_View+First_Person+Image_View].mp4
Discontinued since mandatory part of the project was more complex than expected. Bluetooth control works well though, including capturing vision with Image View.

3 - Optional Features: Evolutionary Neural Network
Watch Evolutionary_Neural_Network_Performance.mp4
Discontinued since training for different cases would take so much time without guarantee that the robot perform well under different scenarios. Took alot of time to make a forward propagation neural network from scratch.