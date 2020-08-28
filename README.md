# mtrn4230
Repository to store source code for Group 13 MTRN4230

## How to setup
1. clone directory into simulation_ws/src
2. run 
```
# run from simulation_ws directory
catkin_make
```
then run 
```
# run from simulation_ws directory
roslaunch ur5_sorting_system ur5_world.launch 
```
(this is currently only launch file but will create more as we get further and have more working parts) 

## Spawning Objects
1. Open a new terminal then run 
```
# run from simulation_ws directory
./devel/lib/ur5_sorting_system/objects_spawner -h
```
This will print usage options for spawning. To spawn random objects 1 at a time by pressing enter, run:
```
# run from simulation_was directory
./devel/lib/ur5_sorting_system/objects_spawner -r -d -1
```
## MATLAB GUI 
1. Open matlab/GUI.mlpaa on your host machine.
2. Select "code view" once App designer window opens.
3. change the ipaddress to your ROS_IP of the virtual machine that's running the Gazebo simulation.
4. Run the code.

## Central Controller
1. Open a new terminal in virtual machine, then run 
```
# run from repository root directory
python src/central_controller.py
```

## Summary
In summary, 
```
catkin_make
roslaunch ur5_sorting_system ur5_world.launch 
# new terminal
./devel/lib/ur5_sorting_system/objects_spawner -r -d -1
```
--- Open MATLAB GUI on host machine ---
```
# new terminal in virtual machine
python src/central_controller.py
```

## Folder structure
/config -> store config and yaml files
/launch -> store launch files
/matlab -> store matlab files
/meshes -> store mesh files to define custom models in ROS Gazebo
/src -> store C++ and Python files
/urdf -> store urdf files
/world -> store world files
