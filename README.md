# ME449-Capstone
Final Capstone Project of ME449 Introduction to Robotic Manipulation

General assignments of ME449 can be found [here](https://github.com/S-odland/ME449-RoboticManipulation)

### Milestone 1 - youBot Kinematics Simulator and csv output
* My code for this milestion contains three functions:
  * writeCSV(): writes a .csv file
  * NextState(): computes the next state of the robot configuration
  * simControl(): simulates a second of robot manipulation (wheels, joints and chassis)
* To execute this code, navigate to where the file is downloaded and type this into the command line:

```python
python NextState.py
```

### Milestone 2 - Reference Trajectory Generation
* My code for this milestone contains three functions:
  * scTose(): this function computes the transformation matrices in the end effector frame from those given in the cube frame
  * InitTG(): this function just sets up the various transformation matrices for the gripper and cube
  * TrajectoryGenerator(): This function computes the trajectories by:
    * Iterating through the eight segments defined in traj_iter
    * Each segment has a specified duration in t
    * From these the inputs to ScrewTrajectory are generated
    * In each iteration the results of ScrewTrajectory are appending to a list of trajectories and the corresponding gripstates are appending to grip_states as well
    * After this the csv file is creates
* To execute this code, navigate to where the file is downloaded and type this into the command line:
  
```python
python TrajectoryGenerator.py
```  
### Milestone 3 - Feedforward Control
