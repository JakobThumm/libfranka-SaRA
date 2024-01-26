// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "examples_common.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>


ShieldMotionGenerator::ShieldMotionGenerator(double speed_factor, double control_time)
    : MotionGenerator(speed_factor, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}){
  control_time_ = control_time;
}

void ShieldMotionGenerator::reset(const std::vector<double> q_goal) {
  q_goal_ = Vector7d(q_goal.data());
  time_ = 0.0;
}

franka::JointPositions ShieldMotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q_d.data());
    delta_q_ = q_goal_ - q_start_;
    calculateSynchronizedValues();
  }

  Vector7d delta_q_d;
  bool motion_finished = time_ >= control_time_;

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}
