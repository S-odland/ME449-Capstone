# ME449-Capstone
Final Capstone Project of ME449 Introduction to Robotic Manipulation

General assignments of ME449 can be found [here](https://github.com/S-odland/ME449-RoboticManipulation)

## This project is broken down into 3 milestones:
### Milestone 1 - youBot Kinematics Simulator and csv output
### Milestone 2 - Reference Trajectory Generation
* My code for this milestone contains two functions:
  * InitTG(): this function just sets up the various transformation matrices for the gripper and cube
  * TrajectoryGenerator(): This function computes the trajectories by:
    * Iterating through the eight segments defined in traj_iter
    * Each segment has a specified duration in t
    * From these the inputs to ScrewTrajectory are generated
    * In each iteration the results of ScrewTrajectory are appending to a list of trajectories and the corresponding gripstates are appending to grip_states as well
    * After this the csv file is creates
* Running the code can be done in two ways:
  * The first is by running this command
'''python
import TrajectoryGenerator as TG
TG.TrajectoryGenerator()
'''
### Milestone 3 - Feedforward Control
