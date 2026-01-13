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

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

int main(int argc, char** argv) {
  using namespace mjbots;
  using Transport = pi3hat::Pi3HatMoteusTransport;

  const int servo_id = 3; 
  const int can_bus = 2;

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

  // stop servo
  moteus_controller->DiagnosticWrite("tel stop\n");
  moteus_controller->DiagnosticFlush();
  servo_status = moteus_controller->DiagnosticCommand("d stop", moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  if (servo_status != "OK") {
    std::cerr << "Error stopping servo, returned output: " << servo_status << std::endl;
    return 1;
  }

  { // servo info
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
  }

  // rezero servo
  // moteus_controller->DiagnosticWrite("d cfg-set-output 0\n");
  // moteus_controller->DiagnosticFlush();
  servo_status = moteus_controller->DiagnosticCommand("d cfg-set-output 0", moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  if (servo_status != "OK") {
    std::cerr << "Error rezeroing servo, returned output: " << servo_status  << std::endl;
    return 1;
  }
  std::cout << "Rezeroed servo successfully.\n";
  
  // const auto new_kp = 4.0;

  // std::ostringstream ostr;
  // ostr << "conf set servo.pid_position.kp " << new_kp;

  // // The `conf set` diagnostic command returns nothing.
  // moteus_controller->DiagnosticCommand(ostr.str());

  // std::cout << "Changed kp from " << old_kp << " to " << new_kp << "\n";


  // start servo
  moteus::PositionMode::Command pos_cmd;
  pos_cmd.position = std::numeric_limits<float>::quiet_NaN();

  std::vector<moteus::CanFdFrame> frames;
  frames.push_back(moteus_controller->MakePosition(pos_cmd));

  std::vector<moteus::CanFdFrame> replies;
  moteus::BlockingCallback cbk;

  pi3hat_transport->Cycle(frames.data(), frames.size(),
                   &replies, nullptr,
                   nullptr, nullptr,
                   cbk.callback());
  cbk.Wait();

  for (const auto& frame : replies) {
    if (frame.source == servo_id) {  // Check if it's from servo ID
      const auto result = moteus::Query::Parse(frame.data, frame.size);
      ::printf("Servo %d  p/v/t=(%7.3f,%7.3f,%7.3f)\n",
               frame.source, result.position, result.velocity, result.torque);
    }
  }

  // pi3hat::Attitude attitude;
  // pi3hat_transport->Cycle(frames.data(), frames.size(),
  //                  &replies, &attitude,
  //                  nullptr, nullptr,
  //                  cbk.callback());
  // cbk.Wait();

  // const auto& a = attitude.attitude;
  // ::printf("Attitude quaternion: (%4.2f,%4.2f,%4.2f,%4.2f)\n",
  //          a.x, a.y, a.z, a.w);

  // for (const auto& frame : replies) {
  //   if (frame.source == servo_id) {  // Check if it's from servo ID
  //     const auto result = moteus::Query::Parse(frame.data, frame.size);
  //     ::printf("Servo %d  p/v/t=(%7.3f,%7.3f,%7.3f)\n",
  //              frame.source, result.position, result.velocity, result.torque);
  //   }
  // }

  // stop servo
  moteus_controller->DiagnosticWrite("tel stop\n");
  moteus_controller->DiagnosticFlush();
  servo_status = moteus_controller->DiagnosticCommand("d stop", moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  if (servo_status != "OK") {
    std::cerr << "Error stopping servo, returned output: " << servo_status << std::endl;
    return 1;
  }

  return 0;
}
