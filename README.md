# neo_docking2

neo_docking2 is a ROS 2 package, which is the spirtual successor of neo_docking.

Like neo_docking, neo_docking2 docks the Neobotix robot autonomously with the charging station. Unlike neo_docking, neo_docking2 does not use a depth camera nor QR tags for detecting the coordinates of the charging contacts. Rather, the user has to once teach the docking position of the robot by manually driving (prefarably with a joystick) and docking the robot to the charging contacts. The docking coordinates are stored in a yaml file, with the help of a service. Later, the docking coordinates are utilized in a 3-step docking process coupled with Navigation 2.

More information about the docking can be found in our [official documenation.](https://neobotix-docs.de/ros/packages/neo_docking2.html)

## Video

https://user-images.githubusercontent.com/20242192/208465136-19763651-d1c3-4b4c-baef-2384c6806431.mp4

## RViz plugin

coming soon ...

