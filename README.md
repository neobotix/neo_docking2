# neo_docking2
ROS2 package for neo_docking

This docking package is designed to be used with the object for which the positions are known. There are no, perception module (at the moment) to detect the position of the object. 

The position of the objects are stored in the yaml file. Currently, we support only one object, which could also be later extended to support multiple use-cases. 

## To Build:

Open a command line, source your ros distribution and continue the following steps:

```
cd your_ros_workspace/src
git clone git@github.com:neobotix/neo_docking2.git
cd ..
colcon build --symlink-install --packages-select neo_docking2
. install/setup.bash
```

Once done you can either 
  * launch the node using the given launch file:
  
    ```ros2 launch neo_docking2 docking_launch.py```

    by default uses the pose from  `dock_pose.yaml`
  
  * or just use `ros2 run` to run the executable. Of course, the params can be also passed along with it using the extension `--ros-args` and pointing to the file containing the params
  
    ```ros2 run neo_docking2 neo_docking2 --ros-args --params-file src/neo_docking2/launch/dock_pose.yaml```

Available services:
  * `/go_and_dock`: Initiates docking process
  * `/undock_and_arm`: Undocks and is ready for the next command
  * `/store_pose`: Poses are stored in the `dock_pose.yaml` file, which can be found under the launch directory. 

## The process:

First we need to store the pose of the docking station, it needs to be done manually. Use the teleop node to navigate the robot to the docking position. Once the robot is in the docking position, use the `/store_pose` service to store the docking position. 

```ros2 service call /store_pose std_srvs/srv/Empty {}```

Next, the docking is a 2 step process, we need to use `/go_and_dock` service to initiate the docking. 

```ros2 service call /go_and_dock std_srvs/srv/Empty {} ```

Once the process has been initiated, the robot navigates to a pre-dock position, which is 0.5 meters in front of the docking station. Then in the second step, the robot starts to dock. 

To undock, use the service `/undock_and_arm`:

```ros2 service call /undock_and_arm std_srvs/srv/Empty {} ```

Once the process has been initiated, the robot navigates back to the pre-dock position.
  
## RViz plugin:

Coming soon... 
