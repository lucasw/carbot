# carbot

ros robot with car-like steering

# Dependencies

Currently in jade and kinetic it is necessary to

    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git

because it is not available as a deb.

If using kinetic, get the kinetic branch:

     cd gazebo_ros_pkgs
     git checkout remotes/origin/kinetic-devel

# gazebo simulation

    roslaunch carbot_gazebo_control carbot_gazebo_control.launch

If it is the first time gazebo has launched, this may take a long time before you see

    [Spawn status: SpawnModel: Successfully spawned model]

and the model appears in rviz and gazebo.

Gazebo frequently doesn't exit cleanly after pressing ctrl-c, you may have to kill the roscore and restart it to run again.

If it does exit cleanly it is a good idea to:

    rosparam delete /

Or at least set sim_time to false.

# commanding motion

Any generator of cmd_vel messages can work here, though teleop twist keyboard and teleop joy are scaled inappropriately by default so need adjustment.

Stop the car:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y0.0}, angular: {z: 0.0}}" -1

Drive straight forward:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"

Turn to left:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.02}}"

Turn to right:

    rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: -0.02}}"

Trying to go to fast glitches the gazebo simulation- it's possible toroidal/capsule shaped wheels would do better.
(The advice given in ode/bullte simulation tutorials for high speed car simulations don't model the spinning wheels physically interacting with the ground, it is handled with rays- but it's not clear if gazebo can handle that approach.)

# linear.y vs. angular.z

It's impossible to command a car/bicycle/ackerman vehicle to drive only sideways (linear.y) or rotate in place (angular.z) without also driving forward or backward (linear.x),
so it is a matter of convention to have to always command some linear.x and then angular.z or linear.y which may or may not be achieveable depending on the distance of the steering wheels to the fixed wheels, and the joint limits of the steering wheels.

It appears that commanding linear.x in combination with angular.z is the preferred method (and linear.y results even if desired to be zero).
carbot also has a mode where linear.x and linear.y are commanded and there is a resulting angular.z.
