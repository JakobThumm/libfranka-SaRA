// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <string>
#include <vector>
#include <chrono>
#include <ctime>

#include "SaRA/reach_lib.hpp"
#include "safety_shield/motion.h"
#include "safety_shield/safety_shield.h"

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  double sample_time = 0.004;
  std::string trajectory_config_file = std::string("../../external/sara-shield/safety_shield/config/trajectory_parameters_panda.yaml");
  std::string robot_config_file = std::string("../../external/sara-shield/safety_shield/config/robot_parameters_panda.yaml");
  std::string mocap_config_file = std::string("../../external/sara-shield/safety_shield/config/cmu_mocap_no_hand.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::PFL;

  safety_shield::SafetyShield shield =
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, shield_type);
  // Dummy human measurement
  std::vector<reach_lib::Point> dummy_human_meas(21);
  for (int i = 0; i < 21; i++) {
    dummy_human_meas[i] = reach_lib::Point(10.0, 10.0, 0.0);
  }

  spdlog::info("Debug started.");
  auto start = std::chrono::system_clock::now();
  double t = 0.0;
  try {
      

    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    double speed_factor = 1.0;
    ShieldMotionGenerator shield_motion_generator(speed_factor, sample_time);
    std::array<double, 7> initial_position;
    double time = 0.0;

    for (int i = 0; i < 10000; i++) {
      t += 0.001;
      shield.humanMeasurement(dummy_human_meas, t);
      t += 0.003;
      if (i % 1000 == 0) {  // % 2
        std::vector<double> qpos;
        if (i % 2000 == 0) {
          qpos = {-M_PI_4, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
        } else {
          qpos = {M_PI_4, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
        }
        std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        shield.newLongTermTrajectory(qpos, qvel);
        std::cout << "new LTT" << std::endl;
      }
      safety_shield::Motion next_motion = shield.step(t);
      std::cout << "Time: " << t << ", next_motion.q = [ " <<
        next_motion.getAngle()[0] << ", " <<
        next_motion.getAngle()[1] << ", " <<
        next_motion.getAngle()[2] << ", " <<
        next_motion.getAngle()[3] << ", " <<
        next_motion.getAngle()[4] << ", " <<
        next_motion.getAngle()[5] << ", " <<
        next_motion.getAngle()[6] << "] " << std::endl;
      shield_motion_generator.reset(next_motion.getAngle());
      robot.control(shield_motion_generator);
    }
    // shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, shield_type);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
