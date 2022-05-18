# Robotics

# Summary

My solution uses the laser scan to detect whether there is a wall ahead of the robot. If there is, the robot will turn to the left.
If there is a wall in front and left, the robot turns right. If there is a wall in front and right, the robot turns left. 
In addition, the robot also detects whether there are any red or green squares. For the red squares, the robot turns left until it can no longer see red.
If the square is green, it will stop. If these conditions are not met, the robot will move forwards.

# Steps to Run System

My system doesn't take any specific steps in order to run it just:
1. Use chmod a+x to make the script executable
2. use rosrun to run the code
