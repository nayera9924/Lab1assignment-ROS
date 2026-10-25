# Lab1assignment-ROS
This project serves as a comprehensive simulation demonstrating the core concepts of ROS 2 focusing on inter-node communication through Services and Actions. It simulates a robotic control architecture using the turtlesim environment, where custom interfaces (.srv and .action), dynamic parameter management, multi-threaded execution, and system automation via launch files are integrated to orchestrate precise autonomous turtle navigation and path execution.

🛠️ Step-by-Step Guide & Testing Commands
1.Before running any node or launch file, always build your workspace and source the setup script (you can put the source command in the bashrc):

cd ~/mechatronics_ws
colcon build --packages-select node_pkg interfaces_pkg
source install/setup.bash

2.Instead of running nodes manually in separate terminals, you can launch the entire stack (Simulator, GoToPose Server, and Square Action Server) using a single launch command:
ros2 launch node_pkg turtlesim.launch.py 

3.You can dynamically change parameters (such as maximum speeds or turtlesim background colors) at runtime or via YAML configuration files (GoToPose.yaml and turtlesim.yaml):
ros2 param set /go_to_pose_server max_linear_speed 1.5        -> ex for max linear speed value

*if you want to change it from the .yaml file open the pakage then the config folder then you can find the yaml file and change the parameters value then you have to build and run the launch file again

4.To get or check parameter:
ros2 param get /go_to_pose_server <wanted_parameter>

5.To reset the turtle to the center home position , clear the canvas, and disable the pen line during teleportation:
ros2 service call /reset_turtle std_srvs/srv/Trigger "{}"

6.To command the turtle to automatically execute a complete square path by visiting all four corners sequentially:
ros2 action send_goal /draw_square interfaces_pkg/action/Square "{side_length: 2.0}" --feedback

Some Project Detials

Python Nodes (node_pkg):
go_to_pose_server.py: The main control node. It uses a proportional controller to guide the turtle precisely to target coordinates, handles speed limits, and manages the reset/teleport service.
square_action_server.py: An action server that automatically draws a complete square by clearing the screen and sending four sequential corner goals to the go_to_pose server.

Custom Interfaces (interfaces_pkg)
This ia a C pakage
GoToPose.action: Custom action definition specifying the target position (x, y, yaw) as a goal, along with feedback and completion results.
Square.action: Custom action definition specifying the side length for drawing a square, along with corner progress feedback and results.

🚀Launch File
turtlesim.launch.py: A unified automation script that starts the Turtlesim simulator, the pose server, and the square action server all at once with a single command instead of running each node in separated terminals.

Configuration Files (config/)
GoToPose.yaml: A configuration file storing  parameters for the motion server, such as maximum linear/angular speeds and position/angle tolerances.
turtlesim.yaml: A configuration file storing  rgb parameters of background of turtle simulator

Package Management Files
setup.py & package.xml: Standard ROS 2 configuration files that manage package dependencies, Python modules, entry points, and ensure launch/config files are installed correctly.




