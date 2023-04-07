Akermann to CAN message using ROS2 in python

can_message_tan (package) - 'AkerToCAN.py' contains project code
                            'package.xml' should have ackermann msg as a dependency
                            'setup.py' contains 'main_node' which is the installed node to run

moa (package) - contains the message structure for CAN

How to use / Steps:
1) open three terminals
2) In one of the terminals build the packages
3) run main node using: 'ros2 run can_message_tan main_node' 
4) In another terminal show /cmd_vel topic output using: 'ros2 topic echo /cmd_vel'
5) In the last terminal show /pub_raw_can topic using: 'ros2 topic echo /pub_raw_can'

The first terminal will show the converted values after transmitting the data to compare with the original values which will be published in the /cmd_vel topic and the raw transmitted data will be show in the /pub_raw_can topic