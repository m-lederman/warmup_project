# warmup_project

## Drive Square
For this problem, I was supposed to figure out how to make my robot drive in a square. To do this, I thought that I should create a loop that repeated 4 times that made the robot advance and then turn. Once the loop was finished, I would reset the robot to be at rest. To set the movement and direction of the robot, I used the same concept as the twist functions used in the lab to modify and then publish velocity.

In terms of functions used, all I had to do outside of the main function was have an initializing function that had the for loop (that made the square) since my robot did not need to interact with its environment and so didn't need other functions to complete the square. 

My biggest challenge was that I did not know how to make my robot properly turn 90 degrees. However, I found out that the angular velocity is in radians per second, so to simulate a 90 degree angle, I put the speed at half a pi, and waited one second. That however did not turn enough, so I realized I had to increase speed to account for friction, and also I assume that as battery dies it functions at less efficiency. For this reason, I went with a speed of 1.7, which is slightly higher than half of pi, and I achieved an angle much closer to 90. 

If I had more time, I would have wanted to perfect the angle. I think I either would have tried to pull out a ruler to test the actual robot's angle, or I would have wanted to experiment in gazebo, because I assume it would have been able to give me more information about the turning angle. 

I think this project was really helpful for me because it helped me get down the basics for coding the robot to move, which is something that will be a key element to every futur project.

![Alt Text](https://github.com/m-lederman/warmup_project/blob/main/ezgif.com-gif-maker.gif)
