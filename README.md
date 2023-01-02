# neo_docking2

neo_docking2 is a ROS 2 package, which is the spirtual successor of neo_docking.

Like neo_docking, neo_docking2 docks the Neobotix robot autonomously with the charging station. Unlike neo_docking, neo_docking2 does not use a depth camera nor QR tags for detecting the coordinates of the charging contacts. Rather, the user has to once teach the docking position of the robot by manually driving (prefarably with a joystick) and docking the robot to the charging contacts. The docking coordinates are stored in a yaml file, with the help of a service. Later, the docking coordinates are utilized in a 3-step docking process coupled with Navigation 2.


## To Build:

Open a command line, source your ros distribution and continue the following steps:

```
cd your_ros_workspace/src
git clone git@github.com:neobotix/neo_docking2.git
cd ..
colcon build --symlink-install --packages-select neo_docking2
. install/setup.bash
```

## To Launch:

Once done you can either 
  * If you have a robot from Neobotix, then you should use the launch:
 
    ```ros2 launch neo_mp(?)_(?)00-2 docking_navigation.launch.py```
    
    the launch can be found under the corresponding robot package. The poses for docking are stored under `dock_pose.yaml`, which can be found under this (neo_docking2) package.
    
  * launch the node using the given launch file:
  
    ```ros2 launch neo_docking2 docking_launch.py```

    by default uses the pose from  `dock_pose.yaml`
  
  * or just use `ros2 run` to run the executable. Of course, the params can be also passed along with it using the extension `--ros-args` and pointing to the file containing the params
  
    ```ros2 run neo_docking2 neo_docking2 --ros-args --params-file src/neo_docking2/launch/dock_pose.yaml```

## Available services:
  * `/go_and_dock`: Initiates docking process
  * `/undock_and_arm`: Undocks and is ready for the next command
  * `/store_pose`: Poses are stored in the `dock_pose.yaml` file, which can be found under the launch directory. 

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

## Open Issue:

Once you store the pose, it is always necessary for you to check if the values are in `double` data type. Currently there is an issue with the Yaml-CPP not being able to emit 1 or 0 as a `double` data type. See this https://github.com/neobotix/neo_docking2/issues/14 for more details. 

## Video

https://user-images.githubusercontent.com/20242192/208465136-19763651-d1c3-4b4c-baef-2384c6806431.mp4

## RViz plugin

coming soon ...

