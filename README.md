# segway_sim

This is a ROS-based C++ simulation environment for a two-wheeled inverted pendulum (Segway) robot. It currently has two branches, a Model Predictive Control (MPC) branch that uses a Mixed Observable Markov Decision Process (MOMDP) planner for high-level planning, and uses the MPC for trajectory generation and control. The other branch is a Control Barrier Function (CBF) branch that guarantees that the system stays in a "safe" subset of the state space, regardless of the inputs given to the system from the user.

<p align="center">
<img src="https://i.imgur.com/Y8BJ5XU.png" width="250" />
</p>

### Prerequisites

Eigen is required. If Eigen is not installed, or you want to run the version of Eigen that runs on the Segway, use the Eigen_embedded folder and replace all instances of Eigen_embedded with Eigen.

The OSQP solver is used for both branches, and can be found [here](https://github.com/oxfordcontrol/osqp)

The MPC branch requires the following [library](https://github.com/urosolia/multirate-mpc-cbf) for MPC, as well as the following ROS [package](https://github.com/DrewSingletary/ambercortex_ros) for its message structure.

CBF branch installation coming soon.

## Description

### The robot
The simulation is based on the Ninebot Elite used in the AMBER lab [link](http://www.bipedalrobotics.com/). The code run in this simulation environment has been used directly on the hardware, as evidenced in this [![video](https://img.youtube.com/vi/Tr6bpjmzHcE/maxresdefault.jpg)](https://youtu.be/Tr6bpjmzHcE)

### How to run

After intalling the MPC and OSQP libraries listend in the above Prerequisites section follows the following steps:

- Run the main.py file in the folder /segway_sim/src/pyFun. It should save a .pkl object in the /segway_sim/src/pyFun/data/Qug_1 folder
- Download the ambercorted_ros pakage from [here](http://www.bipedalrobotics.com/)
- Checkout the dev branch in the ambercorted_ros package
- Build your catkin workspace
- Open the launch file main.launch. At the bottom of the launch file enter the folder where you would like to save your bag file
- Run roslaunch segway_sim main.launch
- In a new terminal run rosservice call /segway_sim/integrator/ui "cmd: 1"
- After the simulation run the file readNewBag.py in the /segway_sim/plotUtils folder (it is needed to change the path where the bag file has been saved)


## Results
Coming soon

## References

This code is based on the following:

(CBF)
* Gurriet, T., Singletary, A., Reher, J., Ciarletta, L., Feron, E., & Ames, A. (2018, April). Towards a framework for realizable safety critical control through active set invariance. In 2018 ACM/IEEE 9th International Conference on Cyber-Physical Systems (ICCPS) (pp. 98-106). IEEE. [PDF](https://ieeexplore.ieee.org/abstract/document/8443725)
* U. Rosolia and A. D. Ames, "Multi-Rate Control Design Leveraging Control Barrier Functions and Model Predictive Control Policies," in IEEE Control Systems Letters, vol. 5, no. 3, pp. 1007-1012, July 2021, [PDF](https://ieeexplore.ieee.org/document/9137248)
