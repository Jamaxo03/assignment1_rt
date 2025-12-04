To run the program, first start the turtlesim node:
ros2 run turtlesim turtlesim_node
Then start the node that spawns the second turtle:
ros2 run assignment1py_rt turtle_spawn
Next, launch the two nodes related to this assignment:
ros2 run assignment1py_rt ui_node
ros2 run assignment1py_rt distance_node

Once everything is running, the user interface will appear in the terminal where ui_node was launched. You will be asked which turtle you want to control (1 or 2), and once selected, you can enter the linear and angular velocity directly from the terminal.

If during its movement the turtle enters a “forbidden” area, it will automatically be assigned the inverse of its previous velocity for half a second. This mechanism is used because simply stopping the turtle would cause it to fail all subsequent position checks, preventing it from moving again. By reversing the motion briefly, the system avoids potential deadlocks.