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
#include "mjbotscpp.h"

using namespace mjbots;
using Transport = pi3hat::Pi3HatMoteusTransport;


struct ServoPositionFeedback {
  moteus::Mode mode;  
  moteus::PositionMode::Command position_feedback;
};

void reset_servo_cmd(moteus::PositionMode::Command& pos_cmd) {
  pos_cmd.position = std::numeric_limits<float>::quiet_NaN();
  pos_cmd.velocity = 0.0;
  pos_cmd.feedforward_torque = 0.0;
}

void reset_servo_feedback(ServoPositionFeedback& servo_feedback) {
  servo_feedback.mode = moteus::Mode::kStopped;
  servo_feedback.position_feedback.position = std::numeric_limits<float>::quiet_NaN();
  servo_feedback.position_feedback.velocity = 0.0;
  servo_feedback.position_feedback.feedforward_torque = 0.0;
}

double servo_sinusoid_trajectory_next(double time_s, double pos_initial = 0.0, double position_increment = 0.001, double frequency = 0.1) {
  const double amplitude = position_increment * 1000; // rot
  double desired_position = pos_initial + amplitude * sin(2 * M_PI * frequency * time_s);
  return desired_position;
}

int main(int argc, char** argv) {
  
  // Configuration
  
  const std::map<int,int> servo_map = // servo id - can bus map 
  {
    {1, 2}, 
    {3, 2}, 
  };

  const int loop_move_ms = 2; // 0.002 seconds, 500 Hz
  const int loop_print_ms = 500; // 0.5 second
  const int loop_duration_s = 30; // run for 30 seconds

  const double position_increment = 0.001; // increment position by 0.01 rot each loop
  const double velocity_limit = 0.5; // max velocity limit, rot/s 

  double pos_initial = 0.0; // initial position, rot

  mjbotscpp::ServoConfig new_config;
  {
    new_config.kp = 400.0;
    new_config.ki = 1.0;
    new_config.kd = 2.0;
    new_config.position_min = -3.0; // rot
    new_config.position_max = 3.0; // rot

    new_config.override_direction = true; // whether to override the default direction from the servo config
    new_config.direction = 1; // default to 1, set to -1 for reversed
    new_config.override_reduction_ratio = true; // whether to override the default reduction ratio from the servo config
    new_config.reduction_ratio = 1.0/6.0; // default to 6:1 reduction, which is used in qdd100 servos
  }

  // Pi3hat Transport
  Transport::Options pi3hat_options;
  pi3hat_options.servo_map = servo_map;
  pi3hat_options.attitude_rate_hz = 100;
  pi3hat_options.enable_aux = false;

  pi3hat_options.mounting_deg.pitch = 0;
  pi3hat_options.mounting_deg.yaw = 0;
  pi3hat_options.mounting_deg.roll = 0;

  auto pi3hat_transport = std::make_shared<Transport>(pi3hat_options);

  // pi3hat::Attitude attitude;

  // Moteus controllers with configurations for each servo
  std::vector<moteus::Controller::Options> moteus_options_list;
  std::vector<std::shared_ptr<mjbots::moteus::Controller>> moteus_controller_list;
  std::vector<mjbotscpp::ServoConfigWithID> servo_config_with_id_list;
  for (const auto& [servo_id, can_bus] : servo_map) {
    moteus::Controller::Options moteus_options;
    moteus_options.transport = pi3hat_transport;
    moteus_options.id = servo_id;
    moteus_options.bus = can_bus;
    auto moteus_controller = std::make_shared<mjbots::moteus::Controller>(moteus_options);
    moteus_controller_list.push_back(moteus_controller);
    moteus_options_list.push_back(moteus_options);

    mjbotscpp::ServoConfigWithID servo_config_with_id;
    servo_config_with_id.id = servo_id;
    servo_config_with_id.can = can_bus;
    servo_config_with_id.config = new_config;
    servo_config_with_id_list.push_back(servo_config_with_id);
  }
  
  // initialize servo
  std::cout << "Stopping servos..." << std::endl;
  if (mjbotscpp::StopServos(moteus_controller_list) != 0) return 1;
  std::cout << "Stopped servos successfully." << std::endl; 
  // if (clear_servo_error() != 0) return 1;

  std::cout << "Setting servo configurations..." << std::endl;
  // if (mjbotscpp::ServosConfSet( new_config, moteus_controller_list ) != 0) return 1;
  if (mjbotscpp::ServosConfSet( servo_config_with_id_list, moteus_controller_list ) != 0) return 1;
  std::cout << "Set servo configurations successfully." << std::endl;

  std::cout << "Fetching servo information..." << std::endl;
  std::cout << mjbotscpp::ServosInfo(moteus_controller_list) << std::endl;
  std::cout << "Fetched servo information successfully." << std::endl;

  std::cout << "Rezeroing servos..." << std::endl;
  if (mjbotscpp::RezeroServos(moteus_controller_list) != 0) return 1;
  std::cout << "Rezeroed servos successfully." << std::endl;
  

  // start servo

  std::vector<moteus::CanFdFrame> frames;
  std::vector<moteus::CanFdFrame> replies;

  std::map< int, ServoPositionFeedback > servo_feedback_list; 
  for (const auto& [servo_id, can_bus] : servo_map) {
    ServoPositionFeedback servo_feedback;
    reset_servo_feedback( servo_feedback );
    servo_feedback_list[servo_id] = servo_feedback;
  }

  std::map< int, moteus::PositionMode::Command > pos_cmd_list;
  for (const auto& [servo_id, can_bus] : servo_map) {
    moteus::PositionMode::Command pos_cmd;
    reset_servo_cmd( pos_cmd );
    pos_cmd_list[servo_id] = pos_cmd;
  }

  std::mutex data_mutex;
  
  const auto start_time = std::chrono::steady_clock::now();
  std::chrono::nanoseconds elapsed_time = std::chrono::nanoseconds::zero(); 

  std::thread move_thread([&]() {
    std::cout << "Starting moving loop for " << loop_duration_s << " seconds...\n";
    while (true) {

      frames.clear();
      replies.clear();
      
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        elapsed_time = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >= loop_duration_s) {
          break;
        }
      }

      // if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >= loop_duration_s) {
      //   break;
      // }

      // Update command under lock since print thread reads `pos_cmd`.
      for (const auto& [servo_id, can_bus] : servo_map) {
        std::lock_guard<std::mutex> lock(data_mutex);

        moteus::PositionMode::Command& pos_cmd = pos_cmd_list[servo_id];
        ServoPositionFeedback& servo_feedback = servo_feedback_list[servo_id];
        if (std::isnan(servo_feedback.position_feedback.position)) {
          // std::lock_guard<std::mutex> lock(data_mutex);
          pos_cmd.position = pos_initial; // move to initial position if no feedback
          pos_cmd.velocity = 0.0;
          pos_cmd.feedforward_torque = 0.0;
        } else {
          // std::lock_guard<std::mutex> lock(data_mutex);
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
      }

      // frames.push_back(moteus_controller->MakePosition(pos_cmd));
      for (auto& moteus_controller : moteus_controller_list) {
        const int servo_id = moteus_controller->options().id;
        const moteus::PositionMode::Command& pos_cmd = pos_cmd_list[servo_id];
        frames.push_back(moteus_controller->MakePosition(pos_cmd));
      }

      moteus::BlockingCallback cbk;
      pi3hat_transport->Cycle(frames.data(), frames.size(),
                      &replies, nullptr,
                      nullptr, nullptr,
                      cbk.callback());
      cbk.Wait();

      // parse feedback
      for ( const auto& frame : replies ) {
        const int servo_id = frame.source;
        if ( servo_map.find(servo_id) == servo_map.end() ) {
          continue; // skip if not in servo_map
        }
        ServoPositionFeedback& servo_feedback = servo_feedback_list[servo_id];
        const auto result = moteus::Query::Parse(frame.data, frame.size);

        std::lock_guard<std::mutex> lock(data_mutex);
        servo_feedback.mode = result.mode;
        servo_feedback.position_feedback.position = result.position;
        servo_feedback.position_feedback.velocity = result.velocity;
        servo_feedback.position_feedback.feedforward_torque = result.torque;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(loop_move_ms));
    }

    std::cout << "Completed moving loop.\n";
  });


  std::thread print_thread([&]() {
    while (true) {

      // Copy shared data under lock to minimize hold time.
      std::chrono::nanoseconds local_elapsed_time; 
      std::map<int, moteus::PositionMode::Command> local_cmd_list;
      std::map<int, ServoPositionFeedback> local_feedback_list;
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        local_elapsed_time = elapsed_time;
        local_cmd_list = pos_cmd_list;
        local_feedback_list = servo_feedback_list;
      }

      if (local_elapsed_time.count() >= loop_duration_s * 1e9) {
        break;
      }

      double elapsed_ms = local_elapsed_time.count() / 1e6;

      for (const auto& [servo_id, can_bus] : servo_map) {
        const moteus::PositionMode::Command& local_cmd = local_cmd_list[servo_id];
        const ServoPositionFeedback& local_feedback = local_feedback_list[servo_id];

        std::cout << std::showpos << std::fixed << std::setprecision(3)
          << "Servo ID: " << servo_id << " | "
          << "Elapsed: " << std::setw(12) << elapsed_ms << " ms | "
          << "[Command] Pos:" << std::setw(8) << local_cmd.position
          << " Vel:" << std::setw(8) << local_cmd.velocity
          << " Trq:" << std::setw(8) << local_cmd.feedforward_torque
          << " | [Feedback] Mode:" << std::noshowpos << std::dec << std::setw(2) << static_cast<int>(local_feedback.mode)
          << std::showpos
          << " Pos:" << std::setw(8) << local_feedback.position_feedback.position
          << " Vel:" << std::setw(8) << local_feedback.position_feedback.velocity
          << " Trq:" << std::setw(8) << local_feedback.position_feedback.feedforward_torque
          << std::endl << std::noshowpos;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(loop_print_ms));
    }
  });

  print_thread.join();
  move_thread.join();

  if (mjbotscpp::StopServos(moteus_controller_list) != 0) return 1;

  return 0;
}
