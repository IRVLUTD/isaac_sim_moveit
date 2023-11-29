MoveIt interface of the Fetch Mobile Manipulator for Isaac Sim

## Usage
1. launch standalone application using Isaac Sim Python
    ```Shell
    cd standalone examples
    ./$ISAAC_ROOT/python fetch_moveit.py
    ``` 

2. launch fetch controllers
    ```Shell
    cd launch
    roslaunch fetch_isaac_controllers.launch
    ```     

3. launch moveit
    ```Shell
    cd launch
    roslaunch fetch_isaac_moveit.launch
    ``` 

## Example
<p align="center">
  <img src="videos/example.gif">
</p>