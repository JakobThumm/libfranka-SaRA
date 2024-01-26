// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <functional>
#include <string>
#include <vector>
#include <chrono>
#include <ctime>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "examples_common.h"


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
  

  spdlog::info("Debug started.");
  auto start = std::chrono::system_clock::now();
  double t = 0.0;
  try {
      

    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

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
    double control_time = 3.0;
    ShieldMotionGenerator shield_motion_generator(speed_factor, control_time);
    std::array<double, 7> initial_position;
    double time = 0.0;

    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback =
            [&model, k_gains, d_gains](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
      // Read current coriolis terms from model.
      std::array<double, 7> coriolis = model.coriolis(state);

      // Compute torque command from joint impedance control law.
      // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
      // time step delay.
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] =
            k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
      }

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be
      // adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

      // Send torque command.
      return tau_d_rate_limited;
    };

    for (int i = 0; i < 20; i++) {
      std::vector<double> qpos;
      if (i % 2 == 0) {
        qpos = {-M_PI_4, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
      } else {
        qpos = {M_PI_4, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
      }
      shield_motion_generator.new_goal(qpos);
      robot.control(impedance_control_callback, shield_motion_generator);
    }
    // shield.reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, shield_type);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
