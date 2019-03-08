# UdacityROSGoChaseIt
ROS program for chasing a white ball through a Gazebo simulator.

In order to execute program a catkin workspace is required.

Steps for creating project:
1.  Create a catkin workspace
* `mkdir catkin_ws`
2.  Create a src folder in the catkin workspace
* `cd catkin_ws && mkdir src`
3.  Clone project into src folder.
* `git clone [project_name]`
4.  Change directory to workspace home - run catkin_make
* `cd ../`
* `catkin_make`

Steps to execute the project:
1.  Source the workspace
* `source devel/setup.bash`
2.  Launch the environment
* `roslaunch my_robot world.launch`
3.  Execute the ball chaser
* `roslaunch ball_chaser ball_chaser.launch`
4.  Place the white ball in front of the robot and observe it chase it.


