#ifndef MPC_NODE_H
#define MPC_NODE_H

#include "ros/ros.h"
#include "segway_sim/cmd.h"
#include "segway_sim/input.h"
#include "segway_sim/state.h"
#include "segway_sim/valFunCnst.h"
#include "segway_sim/optSol.h"
#include "segway_sim/goalSetAndState.h"
#include "segway_sim/linearMatrices.h"
#include "segway_sim/common.hpp"
#include "ambercortex_ros/state.h"
#include "ambercortex_ros/cmd.h"
#include "sensor_msgs/Joy.h"
#include <fstream>

#include <iostream>
#include <string>
#include <ros/package.h>
#include <osqp.h>
#include <mpc.hpp>
#include <segway_example.hpp>

#define MSG_MODE 'M'

#define MODE_INIT 10
#define MODE_RUN 20
#define MODE_BACKUP 30
#define MSG_CTRL 'K'

static const uint32_t nx = 7;     // Number of system state
static const uint32_t nu = 2;     // Number of inputs
static const uint32_t N  = 40; 	  // Horizon length
static const uint32_t printLevel =  0; // 0 = no printing, 1 = only x_t, 2 = minimal, 3 = minimal + initialization, 4 = all

union num32_t {
  int32_t i;
  uint32_t ui;
  float f;
  uint8_t c[4];
};

void add_float_to_vec(std::vector<uint8_t> & vec, float f) {
  // add four bits to the end of a std::vector<uint8_t> representing a float
  num32_t n32;
  n32.f = f;
  vec.push_back(n32.c[0]);
  vec.push_back(n32.c[1]);
  vec.push_back(n32.c[2]);
  vec.push_back(n32.c[3]);
}

uint8_t compute_vec_chksum(const std::vector<uint8_t> & vec) {
  // compute checksum of a std::vector<uint8_t>
  uint8_t chksum = 0;
  for (auto it = vec.cbegin(); it != vec.cend(); ++it)
    chksum += *it;
  return chksum;
}

#endif