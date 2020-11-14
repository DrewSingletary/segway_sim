To run single agent in sim:
	1) roslaunch segway_sim main.launch
	2) rosservice call /segway_sim/integrator/ui "cmd: 1"

To run multi-agent in sim:
	1) roslaunch segway_sim mpcDrone.launch
	2) roslaunch segway_sim mainMultiAgent.launch
	3) rosservice call /segway_sim/integrator/ui "cmd: 1"
