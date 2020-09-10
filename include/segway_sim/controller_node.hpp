#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "ros/ros.h"
#include "segway_sim/common.hpp"
#include "segway_sim/input.h"
#include "segway_sim/state.h"
#include "segway_sim/cmd.h"
#include "visualization_msgs/Marker.h"

enum class STATUS : uint8_t
{
	FAILURE = 0,
	RUNNING = 1
};

#endif