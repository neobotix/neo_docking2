# neo_docking2

neo_docking2 is a ROS 2 package, which is the spirtual successor of neo_docking.

Like neo_docking, neo_docking2 docks the Neobotix robot autonomously with the charging station. Unlike neo_docking, neo_docking2 does not use QR tags for detecting the coordinates of the charging contacts. Currently the package offers two modes:

* Autonomous mode: For this mode, the package use the closed-source neo_perception2 package for detecting the contour and docking with it. 
* Manual mode: The user has to once teach the docking position of the robot by manually driving (prefarably with a joystick) and docking the robot to the charging contacts. The docking coordinates are stored in a yaml file, with the help of a service. At the moment, please use the main branch for the manual mode. 

For both the modes, the docking coordinates are utilized in a 3-step docking process coupled with Navigation 2.


## To Build:

Open a command line, source your ros distribution and continue the following steps:

```
cd your_ros_workspace/src
git clone git@github.com:neobotix/neo_docking2.git
cd ..
colcon build --symlink-install --packages-select neo_docking2
. install/setup.bash
```

Please make sure to contact us for the neo_perception2 package. 

## Parameters:

`auto_detect`: Should be obviously set to true. 

`scan_topic`: Scanner topic that would be used for contour matching. 

`pcd_source`: File path to the contour source.

`pre_dock_dist`: Two pre-docking locations are provided, with the initial pre-docking position automatically set as the default value of 1.3 meters. The second pre-docking pose represents the position that the robot will reach just before reaching the actual docking pose. You can specify the distance to this second pre-docking pose using the pre_dock_dist parameter.

`undock_dist`: Distance upto which the robot needs to travel for undocking. (Note: Undocking is based upon a simple P-Controller)

`offset_x`, `offset_y`: Represents the position to which the robot needs to dock with respect to the map frame. For any contour, the left most point would be the origin. Depending upon the position where you want your robot to dock to, the corresponding co-ordinates needs to be given as the offset.

`offset_yaw`: Orients the docking link transforms according to the required orientation with which the robot can align and dock with the stations. 

## To Launch:

Once done you can either 
  * If you have a robot from Neobotix, then you should use the launch:
 
    ```ros2 launch neo_mp(?)_(?)00-2 docking_navigation.launch.py```
    
    the launch can be found under the corresponding robot package. The poses for docking are stored under `dock_param.yaml`, which can be found under this (neo_docking2) package.
    
  * launch the node using the given launch file:
  
    ```ros2 launch neo_docking2 docking_launch.py```

    by default uses the pose from  `dock_param.yaml`
  
  * or just use `ros2 run` to run the executable. Of course, the params can be also passed along with it using the extension `--ros-args` and pointing to the file containing the params
  
    ```ros2 run neo_docking2 neo_docking2 --ros-args --params-file src/neo_docking2/launch/dock_param.yaml```

## Available services:
  * `/go_and_dock`: Initiates docking process
  * `/undock_and_arm`: Undocks and is ready for the next command

## The process:

First, we need to store the pose of the docking station, it needs to be done manually. Use the joystick to navigate the robot to the docking position. Once the robot is in the docking position, use the `/store_pose` service to store the docking position. 

```ros2 service call /store_pose std_srvs/srv/Empty {}```

Next, the docking is a 2 step process, we need to use `/go_and_dock` service to initiate the docking. 

```ros2 service call /go_and_dock std_srvs/srv/Empty {} ```

Once the process has been initiated, the robot navigates to a pre-dock position, which is 0.5 meters in front of the docking station. Then in the second step, the robot starts to dock. 

To undock, use the service `/undock_and_arm`:

```ros2 service call /undock_and_arm std_srvs/srv/Empty {} ```

Once the process has been initiated, the robot navigates back to the pre-dock position.

## Safety Instruction

If you have brought the robot and the charging station from us, please remember the following points at all the times:

 - Once the robot is docked, make sure that you do not pass any velocity commands nor send a navigation goal from RViz. This would cause serious physical damage to the charging station.
 - While storing the poses for the charging station, you do not have to go very deep into the charging station, rather the advisable range would be not more than 2.0 cm.
 - Please make sure to remove the recovery behaviors in behavior tree.

## Video

coming soon ...

## RViz plugin

coming soon ...

