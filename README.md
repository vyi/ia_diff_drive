Interval Analysis based collision avoidance ROS simulation
-----------------------------------------------------------


To use this package:
 - Extract package inside your workspace/src/
 - Build with catkin\_make to update the project entry
 - Launch the world with `roslaunch ia\_diff\_drive load\_world.launch`

The second version of this package uses kobuki-gazebo package. Install this package with the following command:
 - apt install ros-kinetic-kobuki-*
 - Build with catkin\_make to update the project entry
 - Launch the second version with `roslaunch ia\_diff\_drive load_kobo_version.launch`

The new package named ia\_ca was added that uses 3 Kobo robots and a walled environment. A new script to visualize the sectors and free space is also added. The new project can be launched with 
 - `roslaunch ia\_ca ca\_multi.launch`
Note: Kobo internally computes the odometry using sensor plugins and also accumulates noise (although very low). To transform the odometry to world frame tf2 package has been used.


A pdf file in included inside the docs directory that explains some aspects of the project:




