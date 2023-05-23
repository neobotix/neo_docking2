# neo_docking2

neo_docking2 is a ROS 2 package, which is the spirtual successor of neo_docking.

Like neo_docking, neo_docking2 docks the Neobotix robot autonomously with the charging station. Unlike neo_docking, neo_docking2 does not use QR tags for detecting the coordinates of the charging contacts. Currently the package offers two modes:

* Autonomous mode: For this mode, the package use the closed-source neo_perception2 package for detecting the contour and docking with it. 
* Manual mode: The user has to once teach the docking position of the robot by manually driving (prefarably with a joystick) and docking the robot to the charging contacts. The docking coordinates are stored in a yaml file, with the help of a service. At the moment, please use the main branch for the manual mode. 

For both the modes, the docking coordinates are utilized in a 3-step docking process coupled with Navigation 2.


## To Build

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

`offset_x`, `offset_y`: Represents the position to which the robot needs to dock with respect to the map frame. For any contour, the left most point would be the origin. Depending upon the position where you want your robot to dock to, the corresponding co-ordinates needs to be given as the offset. (In meters)

`offset_yaw`: Orients the docking link transforms according to the required orientation with which the robot can align and dock with the stations. (In radians)

## To Launch

Once done you can either 
  * If you have a robot from Neobotix, then you should use the launch:
 
    ```ros2 launch neo_mp(?)_(?)00-2 docking_navigation.launch.py```
    
    the launch can be found under the corresponding robot package. The poses for docking are stored under `dock_param.yaml`, which can be found under this (neo_docking2) package.
    
  * launch the node using the given launch file:
  
    ```ros2 launch neo_docking2 docking_launch.py```

    by default uses the pose from  `dock_param.yaml`
  
  * or just use `ros2 run` to run the executable. Of course, the params can be also passed along with it using the extension `--ros-args` and pointing to the file containing the params
  
    ```ros2 run neo_docking2 neo_docking2 --ros-args --params-file src/neo_docking2/launch/dock_param.yaml```

## Available services
  * `/go_and_dock`: Initiates docking process
  * `/undock_and_arm`: Undocks and is ready for the next command
  * `/perception/init_contour_pose`: Sets the initial guess

## The process

First, we need to give the initial guess of the location of the docking station. For this we need to use the service `/perception/init_contour_pose`. An example for the same is:

`ros2 service call /perception/init_contour_pose neo_srvs2/srv/InitializeContourMatching "{init_pose: {position: {x: 0.0, y: -0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.08715, w: 0.9961}}}"`

Note that, in order for the neo_perception2 package to detect the contours, it is essential that the laser scanner see most of the contour. For example, if you are detecting a table and if one leg of the table is not detected, then, the algorithm won't give you efficient results.

Next, the docking is a 3 step process, we need to use `/go_and_dock` service to initiate the docking. 

```ros2 service call /go_and_dock std_srvs/srv/Empty {} ```

Once the process has been initiated, the robot navigates to two of the pre-dock positions. The distance to the first pre-dock position defaults to 1.3, whereas the second pre-dock position can be configured from the config files depending upon the requirements from the application. The third step is the docking pose. To reach all the 3 poses, this docking package uses the Nav2 for setting the goals.

To undock, use the service `/undock_and_arm`:

```ros2 service call /undock_and_arm std_srvs/srv/Empty {} ```

Once the process has been initiated, the robot navigates to the pose depending upon the user-specified undocking distance.

## Safety Instruction

If you have brought the robot and the charging station from us, please remember the following points at all the times:

 - Once the robot is docked, make sure that you do not pass any velocity commands nor send a navigation goal from RViz. This would cause serious physical damage to the charging station.
 - While storing the poses for the charging station, you do not have to go very deep into the charging station, rather the advisable range would be not more than 2.0 cm.
 - Please make sure to remove the recovery behaviors in behavior tree.

## Tuning Guide

#### Setting offset_yaw

The value for this parameter can be set to `0.0`, if you are able to record the complete contour that you want to dock with from the docking position, or a position relatively close to it. In most cases, this is not possible. In that case, the offset_yaw should be tuned, so that the robot can easily align and dock. Here is an example, say you want to dock with a table at it's exact centre, and you have recorded thecontour from the robot position as shown in the figure:

![saving_at_an_angle](https://github.com/neobotix/neo_docking2/assets/20242192/a61bcd31-393b-47fc-a757-4820c86dd954)

Once you decide to dock with the offset_yaw set to `0.0`, it is important to note that the angle of the docking_link at the midpoint deviates by 10 degrees. Ideally, we want the robot to approach and dock with the table in a parallel orientation with respect to the table. However, due to this deviation, there is a risk of the robot triggering an emergency stop or potentially causing harm to the docking object.

![matched_pcd_wo_offset](https://github.com/neobotix/neo_docking2/assets/20242192/d66088cb-6cdf-4ff3-86b3-16d8a6701232)

To mitigate this issue, we need to address the deviation in the docking_link angle. The best possible solution is to adjust the offset_yaw value to compensate for the deviation and ensure that the robot aligns properly with the table. By carefully calibrating the offset, we can achieve the desired parallel orientation and minimize the risk of any collisions or harm to the docking object. For this scenario an angle of 5 degrees (0.075 radians) was set as the `offset_yaw`.


![matched_wi_offset](https://github.com/neobotix/neo_docking2/assets/20242192/618cae74-11b2-4758-9bb4-ac6f5c5fcebc)

#### Setting offset_x and offset_y

Taking the same example as above of robot docking to a table. Setting the `offset_x` and `offset_y` value to be `0.0`, then it means that the origin of the docking object is in the bottom, left-most position of the docking object as shown in the figure below.

![wo_tuned_offset](https://github.com/neobotix/neo_docking2/assets/20242192/44d08b59-5bd2-4ea5-92ca-eb6d0b9ace65)

Assume that we want to dock the robot to the middke of the table, then the offset values are adjusted, in this case ` offset_x: -0.70 and offset_y: -0.37`. 

![With_tuned_offset](https://github.com/neobotix/neo_docking2/assets/20242192/2624ce77-6596-4f3b-8368-204eeb1b1210)



## Video

coming soon ...

## RViz plugin

coming soon ...

