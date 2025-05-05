### READme Projet


Author: 
Guillaume OUDET:  21306765
and
Liviu STAN : 21211659


In our package, there two different launch file:


###############################################3 

The first one is called "challenge.launch" and will execute all the challenges required for this project except for the last one (with doors) which  which has not been implemented. Essentially, the robot will start its run at the beginning of the track and will continue without stopping until the last red line where it will stop. There is also an emergency stop for the robot in case an object is placed in front of it.

You can launch it directly with:

roslaunch projet challenge.launch


####################################################

The second one is for teleoperation. Since we did not include teleoperation in the first launch, it has its own launch file. Here, the robot also starts from the beginning of the track but can only be manipulated manually. There is no emergency stop feature in this code, and it needs to be stopped with the keyboard shortcut "s".

You can launch it directly with:

roslaunch projet mybot_teleop.launch

