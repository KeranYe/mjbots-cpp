// Diagnostic and Control Protocol Test for Multiple Moteus Servos with Pi3hat Transport
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
#include <future>

#include <yaml-cpp/yaml.h>

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
  const std::string config_path = (argc > 1) ? argv[1] : "conf.yaml";

  // Parse servo map from YAML: can_servo_map: {can_bus: [servo_ids]}
  // Convert to {servo_id: can_bus}
  std::map<int, int> servo_map;
  {
    YAML::Node config = YAML::LoadFile(config_path);
    if (config.Type() == YAML::NodeType::Null || config.Type() == YAML::NodeType::Undefined) {
      std::cerr << "Error: failed to load config file " << config_path << std::endl;
      return 1;
    }

    const YAML::Node& can_servo_map = config["can_servo_map"];
    if (can_servo_map.Type() == YAML::NodeType::Null) {
      std::cerr << "Error: can_servo_map not found in config file " << config_path << std::endl;
      return 1;
    }
    if (can_servo_map.Type() != YAML::NodeType::Map) {
      std::cerr << "Error: can_servo_map in config file " << config_path << " is not a map" << std::endl;
      return 1;
    }


    for (const auto& entry : can_servo_map) {
      int can_bus = entry.first.as<int>();
      for (const auto& srv : entry.second) {
        servo_map[srv.as<int>()] = can_bus;
      }
    }
  }

  if (servo_map.empty()) {
    std::cerr << "Error: no servos found in " << config_path << std::endl;
    return 1;
  } else {
    std::cout << "Found the following servos in " << config_path << ":" << std::endl;
    for (const auto& [servo_id, can_bus] : servo_map) {
      std::cout << "  Servo ID: " << servo_id << " on CAN bus: " << can_bus << std::endl;
    }
  }

  // timing parameters
  int loop_move_ms = 2;
  int loop_idle_ms = 2;
  int loop_print_ms = 500;
  int loop_move_duration_s = 30;
  int loop_idle_duration_s = 30;
  {
    YAML::Node config = YAML::LoadFile(config_path);
    const YAML::Node& timing = config["timing_parameters"];
    if (timing && timing.IsMap()) {
      if (timing["loop_move_ms"])      loop_move_ms      = timing["loop_move_ms"].as<int>();
      if (timing["loop_idle_ms"])      loop_idle_ms      = timing["loop_idle_ms"].as<int>();
      if (timing["loop_print_ms"])     loop_print_ms     = timing["loop_print_ms"].as<int>();
      if (timing["loop_move_duration_s"]) loop_move_duration_s = timing["loop_move_duration_s"].as<int>();
      if (timing["loop_idle_duration_s"]) loop_idle_duration_s = timing["loop_idle_duration_s"].as<int>();

      std::cout << "Timing parameters from config:" << std::endl;
      std::cout << "  loop_move_ms: " << loop_move_ms << std::endl;
      std::cout << "  loop_idle_ms: " << loop_idle_ms << std::endl;
      std::cout << "  loop_print_ms: " << loop_print_ms << std::endl;
      std::cout << "  loop_move_duration_s: " << loop_move_duration_s << std::endl;
      std::cout << "  loop_idle_duration_s: " << loop_idle_duration_s << std::endl;

    }
  }
  const int loop_total_duration_s = loop_move_duration_s + loop_idle_duration_s;
  std::cout << "Total loop duration (move + idle) from config: " << loop_total_duration_s << " s" << std::endl;

  // trajectory parameters
  double position_increment = 0.001;
  double velocity_limit = 0.5;
  double pos_initial = 0.0;
  {
    YAML::Node config = YAML::LoadFile(config_path);
    const YAML::Node& traj = config["trajectory_parameters"];
    if (traj && traj.IsMap()) {
      if (traj["position_increment"]) position_increment = traj["position_increment"].as<double>();
      if (traj["velocity_limit"])     velocity_limit     = traj["velocity_limit"].as<double>();
      if (traj["pos_initial"])        pos_initial        = traj["pos_initial"].as<double>();
      
      std::cout << "Trajectory parameters from config:" << std::endl;
      std::cout << "  position_increment: " << position_increment << std::endl;
      std::cout << "  velocity_limit: " << velocity_limit << std::endl;
      std::cout << "  pos_initial: " << pos_initial << std::endl;
    }
  }

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


  // only start when pressed key r
  std::cout << "Press 'r' to start the servo control loop..." << std::endl;
  char c;
  do {
    std::cin >> c;
  } while (c != 'r'); 
  
  const auto start_time = std::chrono::steady_clock::now(); // overall start time for the program
  std::chrono::nanoseconds elapsed_time = std::chrono::nanoseconds::zero(); // overall elapsed time for the program

  
  /**********************/
  /** Lambda Functions **/
  /**********************/

  std::promise<void> move_thread_promise_completed;
  std::future<void> move_thread_future_completed = move_thread_promise_completed.get_future();

  std::promise<void> idle_thread_promise_completed;
  std::future<void> idle_thread_future_completed = idle_thread_promise_completed.get_future();

  auto move_thread_func = [&]() {
    std::cout << "Starting moving loop for " << loop_move_duration_s << " seconds...\n";
    
    const auto thread_start_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds thread_elapsed_time = std::chrono::nanoseconds::zero();

    while (true) {

      frames.clear();
      replies.clear();
      
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        thread_elapsed_time = std::chrono::steady_clock::now() - thread_start_time;
        elapsed_time = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(thread_elapsed_time).count() >= loop_move_duration_s) {
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
    move_thread_promise_completed.set_value(); // signal that move thread is completed

    return;
  }; 

  auto idle_thread_func = [&]() {
    // wait until move thread is completed before starting idle loop
    move_thread_future_completed.wait();

    std::cout << "Starting idle loop for " << loop_idle_duration_s << " seconds...\n";

    const auto thread_start_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds thread_elapsed_time = std::chrono::nanoseconds::zero();

    while (true) {

      frames.clear();
      replies.clear();
      
      {
        std::lock_guard<std::mutex> lock(data_mutex);
        thread_elapsed_time = std::chrono::steady_clock::now() - thread_start_time;
        elapsed_time = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::seconds>(thread_elapsed_time).count() >= loop_idle_duration_s) {
          break;
        }
      }

      // if (std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count() >= loop_duration_s) {
      //   break;
      // }

      // Update command under lock since print thread reads `pos_cmd`.
      for (const auto& [servo_id, can_bus] : servo_map) {
        std::lock_guard<std::mutex> lock(data_mutex);

        // quiet_NaN command: maintain current position
        moteus::PositionMode::Command& pos_cmd = pos_cmd_list[servo_id];
        ServoPositionFeedback& servo_feedback = servo_feedback_list[servo_id];
        pos_cmd.position = std::numeric_limits<double>::quiet_NaN(); // move to initial position if no feedback
        pos_cmd.velocity = std::numeric_limits<double>::quiet_NaN();
        pos_cmd.feedforward_torque = std::numeric_limits<double>::quiet_NaN();
      }

      // frames.push_back(moteus_controller->MakePosition(pos_cmd));
      for (auto& moteus_controller : moteus_controller_list) {
        const int servo_id = moteus_controller->options().id;
        const moteus::PositionMode::Command& pos_cmd = pos_cmd_list[servo_id];

        // frames.push_back(moteus_controller->MakePosition(pos_cmd)); // send quiet_NaN command to maintain current position

        // frames.push_back(moteus_controller->MakeQuery()); // send query command to get feedback while idling, current on, with some friction

        frames.push_back(moteus_controller->MakeStop()); // send stop command to get feedback while idling, current off, no friction
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

      std::this_thread::sleep_for(std::chrono::milliseconds(loop_idle_ms));
    }

    std::cout << "Completed idle loop.\n";
    idle_thread_promise_completed.set_value(); // signal that idle thread is completed

    return;
  }; 

  auto print_thread_func = [&]() {
    size_t num_servos = servo_map.size();
    bool first_print = true;
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

      // future-based loop exit condition
      if (move_thread_future_completed.wait_for(std::chrono::seconds(0)) == std::future_status::ready &&
          idle_thread_future_completed.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        break;
      }

      double elapsed_ms = local_elapsed_time.count() / 1e6;

      // Move cursor up to overwrite previous output (except first print)
      if (!first_print) {
        // Move cursor up by num_servos lines
        std::cout << "\033[" << num_servos << "A";
      } else {
        first_print = false;
      }

      for (const auto& [servo_id, can_bus] : servo_map) {
        const moteus::PositionMode::Command& local_cmd = local_cmd_list[servo_id];
        const ServoPositionFeedback& local_feedback = local_feedback_list[servo_id];

        // Clear the line before printing (optional, for cleaner output)
        std::cout << "\r\033[K";
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

    std::cout << "Completed printing loop.\n";
    return;
  };

  /**********************/
  /** Threads **/
  /**********************/
  std::thread move_thread, idle_thread, print_thread;

  print_thread = std::thread( print_thread_func );
  move_thread = std::thread( move_thread_func );
  idle_thread = std::thread( idle_thread_func );

  move_thread.join();
  idle_thread.join();
  print_thread.join();


  if (mjbotscpp::StopServos(moteus_controller_list) != 0) return 1;

  return 0;
}
