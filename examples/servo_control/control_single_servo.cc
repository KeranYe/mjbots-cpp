// Diagnostic Protocol Test for Single Moteus Servo with Pi3hat Transport
// Author: Keran Ye
// Date: Jan 2026

#include <unistd.h>

#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <iomanip>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

using namespace mjbots;
using Transport = pi3hat::Pi3HatMoteusTransport;

struct ServoConfig {
  double kp = 400.0;
  double ki = 1.0;
  double kd = 2.0;
  double position_min = -3.0; // rot
  double position_max = 3.0; // rot
};

struct ServoPositionFeedback {
  moteus::Mode mode;  
  moteus::PositionMode::Command position_feedback;
};



int main(int argc, char** argv) {
  
  // Configuration
  const int servo_id = 3; 
  const int can_bus = 2;
  const int loop_move_ms = 5; // 0.005 seconds, 200 Hz
  const int loop_print_ms = 1000; // 1 second
  const int loop_duration_s = 30; // run for 10 seconds

  const double position_increment = 0.001; // increment position by 0.01 rot each loop
  const double velocity_limit = 0.5; // max velocity limit, rot/s 

  double pos_initial = 0.0; // initial position, rot

  ServoConfig new_config;

  // Pi3hat Transport
  Transport::Options pi3hat_options;
  pi3hat_options.servo_map[servo_id] = can_bus;
  pi3hat_options.attitude_rate_hz = 100;
  pi3hat_options.enable_aux = false;

  pi3hat_options.mounting_deg.pitch = 0;
  pi3hat_options.mounting_deg.yaw = 0;
  pi3hat_options.mounting_deg.roll = 0;

  auto pi3hat_transport = std::make_shared<Transport>(pi3hat_options);

  // pi3hat::Attitude attitude;

  // Moteus Controller
  moteus::Controller::Options moteus_options; 
  moteus_options.transport = pi3hat_transport;
  moteus_options.id = servo_id;

  auto moteus_controller = std::make_shared<mjbots::moteus::Controller>(moteus_options);

  std::string servo_status; 

  // lambda: clear servo error
  auto clear_servo_error = [](
    std::shared_ptr<mjbots::moteus::Controller> moteus_controller
  ) -> int {
    moteus_controller->DiagnosticWrite("tel stop\n");
    moteus_controller->DiagnosticFlush();
    return 0; 
  };

  // lambda: stop servo
  auto stop_servo = [](
    std::shared_ptr<mjbots::moteus::Controller> moteus_controller
  ) -> int {
    moteus_controller->DiagnosticWrite("tel stop\n");
    moteus_controller->DiagnosticFlush();
    auto status = moteus_controller->DiagnosticCommand("d stop", moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    if (status != "OK") {
      std::cerr << "Error stopping servo, returned output: " << status << std::endl;
      return 1;
    }
    return 0; 
  };

  // lambda: rezero servo
  auto rezero_servo = [](
    std::shared_ptr<mjbots::moteus::Controller> moteus_controller
  ) -> int {
    auto status = moteus_controller->DiagnosticCommand("d cfg-set-output 0", moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    if (status != "OK") {
      std::cerr << "Error rezeroing servo, returned output: " << status  << std::endl;
      return 1;
    }
    return 0; 
  };

  // lambda: servo info
  auto servo_info = []( 
    std::shared_ptr<mjbots::moteus::Controller> moteus_controller 
  ) -> int 
  {
    // servo info
    const auto servo_id = std::stoi( 
      moteus_controller->DiagnosticCommand("conf get id.id", moteus::Controller::kExpectSingleLine)
    );
    moteus_controller->DiagnosticFlush();

    const auto servo_gear_ratio = std::stod( 
      moteus_controller->DiagnosticCommand("conf get motor_position.rotor_to_output_ratio", moteus::Controller::kExpectSingleLine)
    );
    moteus_controller->DiagnosticFlush();

    const auto servo_kp = std::stod( 
      moteus_controller->DiagnosticCommand("conf get servo.pid_position.kp", moteus::Controller::kExpectSingleLine)
    );
    moteus_controller->DiagnosticFlush();

    const auto servo_ki = std::stod( 
      moteus_controller->DiagnosticCommand("conf get servo.pid_position.ki", moteus::Controller::kExpectSingleLine)
    );
    moteus_controller->DiagnosticFlush();

    const auto servo_kd = std::stod( 
      moteus_controller->DiagnosticCommand("conf get servo.pid_position.kd", moteus::Controller::kExpectSingleLine)
    );
    moteus_controller->DiagnosticFlush();

    const auto servo_limit_posmax = std::stod( 
      moteus_controller->DiagnosticCommand("conf get servopos.position_max", moteus::Controller::kExpectSingleLine)
    );
    moteus_controller->DiagnosticFlush();

    const auto servo_limit_posmin = std::stod( 
      moteus_controller->DiagnosticCommand("conf get servopos.position_min", moteus::Controller::kExpectSingleLine)
    );
    moteus_controller->DiagnosticFlush();

    std::cout 
      << "Servo ID: " << servo_id << "\n" 
      << "Gear Ratio: " << servo_gear_ratio << "\n"
      << "PID: kp=" << servo_kp << ", ki=" << servo_ki << ", kd=" << servo_kd << "\n"
      << "Position Limits: min=" << servo_limit_posmin << ", max=" << servo_limit_posmax
      << std::endl; 
    return 0;
  }; 

  // lambda: servo conf set
  auto servo_conf_set = []( 
    const ServoConfig& new_config, 
    std::shared_ptr<mjbots::moteus::Controller> moteus_controller 
  ) -> int {

    std::ostringstream ostr;
    std::string response;

    ostr << "conf set servo.pid_position.kp " << new_config.kp;
    response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    ostr.str(""); // clear stream
    if (response != "OK") {
      std::cerr << "Error setting kp, response: " << response << std::endl;
      return 1;
    }

    ostr << "conf set servo.pid_position.ki " << new_config.ki;
    response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    ostr.str(""); // clear stream
    if (response != "OK") {
      std::cerr << "Error setting ki, response: " << response << std::endl;
      return 1;
    }
    
    ostr << "conf set servo.pid_position.kd " << new_config.kd;
    response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    ostr.str(""); // clear stream
    if (response != "OK") {
      std::cerr << "Error setting kd, response: " << response << std::endl;
      return 1;
    }

    ostr << "conf set servopos.position_min " << new_config.position_min;
    response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    ostr.str(""); // clear stream
    if (response != "OK") {
      std::cerr << "Error setting position_min, response: " << response << std::endl;
      return 1;
    }
    
    ostr << "conf set servopos.position_max " << new_config.position_max;
    response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    ostr.str(""); // clear stream
    if (response != "OK") {
      std::cerr << "Error setting position_max, response: " << response << std::endl;
      return 1;
    }

    return 0;  
  }; 

  // lambda: reset servo cmd
  auto reset_servo_cmd = []( moteus::PositionMode::Command& pos_cmd ) {
    pos_cmd.position = std::numeric_limits<float>::quiet_NaN();
    pos_cmd.velocity = 0.0;
    pos_cmd.feedforward_torque = 0.0;
  };

  // lambda: reset servo feedback
  auto reset_servo_feedback = []( ServoPositionFeedback& servo_feedback ) {
    servo_feedback.mode = moteus::Mode::kStopped;
    servo_feedback.position_feedback.position = std::numeric_limits<float>::quiet_NaN();
    servo_feedback.position_feedback.velocity = 0.0;
    servo_feedback.position_feedback.feedforward_torque = 0.0;
  };

  // lambda: servo sinusoid trajectory next waypoint, 
  // input of time in seconds, initial position, increment, frequency
  auto servo_sinusoid_trajectory_next = [](
    double time_s,
    double pos_initial = 0.0,
    double position_increment = 0.001, // rot
    double frequency = 0.1 // Hz
  ) -> double {
    // Simple sinusoid trajectory: A sin(2 pi f t)
    const double amplitude = position_increment * 1000; // rot
    double desired_position = pos_initial + amplitude * sin(2 * M_PI * frequency * time_s);
    return desired_position;
  };

  
  // initialize servo
  if (stop_servo(moteus_controller) != 0) return 1; 
  // if (clear_servo_error() != 0) return 1;

  if (servo_conf_set( new_config, moteus_controller ) != 0) return 1;

  if (servo_info(moteus_controller) != 0) return 1;

  if (rezero_servo(moteus_controller) != 0) return 1;
  std::cout << "Rezeroed servo successfully.\n";
  

  // start servo
  // moteus::PositionMode::Command pos_cmd;
  // pos_cmd.position = std::numeric_limits<float>::quiet_NaN();

  std::vector<moteus::CanFdFrame> frames;
  std::vector<moteus::CanFdFrame> replies;

  ServoPositionFeedback servo_feedback;
  reset_servo_feedback( servo_feedback );

  moteus::PositionMode::Command pos_cmd;
  reset_servo_cmd( pos_cmd );

  std::mutex data_mutex;

  // moteus::PositionMode::Command pos_feedback; 
  // reset_servo_cmd( pos_feedback );

  
  const auto start_time = std::chrono::steady_clock::now();
  std::chrono::nanoseconds elapsed_time; 

  std::thread move_thread([&]() {
    std::cout << "Starting moving loop for " << loop_duration_s << " seconds...\n";
    while (true) {

      frames.clear();
      replies.clear();

      elapsed_time = std::chrono::steady_clock::now() - start_time;
      if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >= loop_duration_s) {
        break;
      }

      // Update command under lock since print thread reads `pos_cmd`.
      if (std::isnan(servo_feedback.position_feedback.position)) {
        std::lock_guard<std::mutex> lock(data_mutex);
        pos_cmd.position = pos_initial; // move to initial position if no feedback
        pos_cmd.velocity = 0.0;
        pos_cmd.feedforward_torque = 0.0;
      } else {
        std::lock_guard<std::mutex> lock(data_mutex);
        // pos_cmd.position = pos_initial 
                            // + static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count()) * position_increment; // move incrementally
        pos_cmd.position = servo_sinusoid_trajectory_next(
          std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() / 1e3,
          pos_initial,
          position_increment,
          0.1 // frequency, Hz
        );
        pos_cmd.velocity = velocity_limit; // set velocity limit
        pos_cmd.feedforward_torque = 0.0;
      }

      frames.push_back(moteus_controller->MakePosition(pos_cmd));

      moteus::BlockingCallback cbk;
      pi3hat_transport->Cycle(frames.data(), frames.size(),
                      &replies, nullptr,
                      nullptr, nullptr,
                      cbk.callback());
      cbk.Wait();

      for (const auto& frame : replies) {
        if (frame.source == servo_id) {  // Check if it's from servo ID
          const auto result = moteus::Query::Parse(frame.data, frame.size);

          std::lock_guard<std::mutex> lock(data_mutex);
          servo_feedback.mode = result.mode;
          servo_feedback.position_feedback.position = result.position;
          servo_feedback.position_feedback.velocity = result.velocity;
          servo_feedback.position_feedback.feedforward_torque = result.torque;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(loop_move_ms));
    }

    std::cout << "Completed moving loop.\n";
  });

  

    
  // if (stop_servo() != 0) return 1;

  // print out feedback and recent command in a separate thread
  std::thread print_thread([&]() {
    while (true) {

      // Copy shared data under lock to minimize hold time.
      std::chrono::nanoseconds local_elapsed_time; 
      moteus::PositionMode::Command local_cmd;
      ServoPositionFeedback local_feedback;
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        local_elapsed_time = elapsed_time;
        local_cmd = pos_cmd;
        local_feedback.mode = servo_feedback.mode;
        local_feedback.position_feedback = servo_feedback.position_feedback;
      }

      if (local_elapsed_time.count() >= loop_duration_s * 1e9) {
        break;
      }

      double elapsed_ms = local_elapsed_time.count() / 1e6;

      std::cout << std::showpos << std::fixed << std::setprecision(3)
        << "Elapsed: " << std::setw(12) << elapsed_ms << " ms | "
        << "Cmd Pos:" << std::setw(8) << local_cmd.position
        << " Cmd Vel:" << std::setw(8) << local_cmd.velocity
        << " Cmd Trq:" << std::setw(8) << local_cmd.feedforward_torque
        << " | Fb Mode:" << std::noshowpos << std::dec << std::setw(2) << static_cast<int>(local_feedback.mode)
        << std::showpos
        << " Fb Pos:" << std::setw(8) << local_feedback.position_feedback.position
        << " Fb Vel:" << std::setw(8) << local_feedback.position_feedback.velocity
        << " Fb Trq:" << std::setw(8) << local_feedback.position_feedback.feedforward_torque
        << std::endl << std::noshowpos;

      std::this_thread::sleep_for(std::chrono::milliseconds(loop_print_ms));
    }
  });

  print_thread.join();
  move_thread.join();

  if (stop_servo(moteus_controller) != 0) return 1;

  return 0;
}
