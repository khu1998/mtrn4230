## mtrn4230
Repository to store source code for Group 13 MTRN4230

# How to setup
1. clone directory into simulation_ws/src
2. run 
```
catkin_make
```
then 
``` 
roslaunch ur5_sorting_system ur5_world.launch 
```
(this is currently only launch file but will create more as we get further and have more working parts) 

# Spawning Objects
1. Open a new terminal then run 
```
./devel/lib/ur5_sorting_system/objects_spawner -h
```
This will print usage options for spawning. To spawn random object 1 at a time by pressing enter, run:
```
./devel/lib/ur5_sorting_system/objects_spawner -r -d -1
```
# MATLAB GUI 
1. Open matlab/GUI.mlpaa on your host machine.
2. Select "code view" once App designer window opens.
3. change the ipaddress to your ROS_IP of the virtual machine that's running the Gazebo simulation.
4. Run the code.

# Central Controller
1. Open a new terminal in virtual machine, then run 
```
python src/central_controller.py
```
In summary, 

```
catkin_make
roslaunch ur5_sorting_system ur5_world.launch 
// new terminal
./devel/lib/ur5_sorting_system/objects_spawner -r -d -1
```
--- Open MATLAB GUI on host machine ---
```
//new terminal in virtual machine
python src/central_controller.py
```

# Folder structure
/urdf   <-- put all urdf files here

/src    <-- put all python files here

/config <-- put all yaml or config files here

/launch <-- put all launch files
