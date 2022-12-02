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
  * export the neo_docking2 executable to a launch file and pass the config yaml containing the dock pose.
  * or just use `ros2 run` to run the executable. Of course, the params can be also passed along with it using the extension `--ros-args` and pointing to the file containing the params
  
## RViz plugin:

Coming soon... 
