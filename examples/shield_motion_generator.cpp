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
  sample_time_ = 0.001;
  std::string trajectory_config_file = std::string("../../external/sara-shield/safety_shield/config/trajectory_parameters_panda.yaml");
  std::string robot_config_file = std::string("../../external/sara-shield/safety_shield/config/robot_parameters_panda.yaml");
  std::string mocap_config_file = std::string("../../external/sara-shield/safety_shield/config/cmu_mocap_no_hand.yaml");
  
  std::vector<double> init_qpos = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  
  shield_ = safety_shield::SafetyShield(sample_time_, trajectory_config_file, robot_config_file, mocap_config_file, init_x_,
                                  init_y_, init_z_, init_roll_, init_pitch_, init_yaw_, init_qpos, shield_type_);
  // Dummy human measurement
  std::vector<reach_lib::Point> dummy_human_meas(21);
  for (int i = 0; i < 21; i++) {
    dummy_human_meas[i] = reach_lib::Point(10.0, 10.0, 0.0);
  }
  dummy_human_meas_ = dummy_human_meas;
}

void ShieldMotionGenerator::reset(const std::vector<double>& q_init) {
  time_ = 0.0;
  shield_.reset(init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_, q_init, time_, shield_type_);
}

void ShieldMotionGenerator::new_goal(const std::vector<double>& q_goal) {
  q_goal_vec_ = q_goal;
  time_ = 0.0;
}

franka::JointPositions ShieldMotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    std::vector<double> q_init(robot_state.q_d.begin(), robot_state.q_d.end());
    reset(q_init);
    shield_.newLongTermTrajectory(q_goal_vec_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  }
  if (time_ >= last_shield_time_ + sample_time_) {
    shield_.humanMeasurement(dummy_human_meas_, time_);
    safety_shield::Motion next_motion = shield_.step(time_);
    std::vector<double> next_q = next_motion.getAngle();
    std::copy(next_q.begin(), next_q.end(), next_q_array_.begin());
    last_shield_time_ = time_;
  }
  
  franka::JointPositions output(next_q_array_);

  if (time_ >= control_time_) {
    std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
    return franka::MotionFinished(output);
  }
  return output;
}
