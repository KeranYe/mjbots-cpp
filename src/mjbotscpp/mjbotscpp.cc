#include <vector>
#include <memory>
#include <string>
#include <ostream>

#include "moteus.h"
// #include "pi3hat/pi3hat_moteus_transport.h"

#include "mjbotscpp.h"

using namespace mjbots;
// using Transport = pi3hat::Pi3HatMoteusTransport;

namespace mjbotscpp {


int ClearServoError(std::shared_ptr<mjbots::moteus::Controller> moteus_controller) {
  moteus_controller->DiagnosticWrite("tel stop\n");
  moteus_controller->DiagnosticFlush();
  return 0;
}


int StopServo(std::shared_ptr<mjbots::moteus::Controller> moteus_controller) {
  moteus_controller->DiagnosticWrite("tel stop\n");
  moteus_controller->DiagnosticFlush();
  auto status = moteus_controller->DiagnosticCommand("d stop", moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  if (status != "OK") {
    std::cerr << "Error stopping servo, returned output: " << status << std::endl;
    return 1;
  }
  return 0;
}


int StopServos(std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers) {
  for (auto& controller : moteus_controllers) {
    if (StopServo(controller) != 0) {
      return 1;
    }
  }
  return 0;
}


int RezeroServo(std::shared_ptr<mjbots::moteus::Controller> moteus_controller) {
  auto status = moteus_controller->DiagnosticCommand("d cfg-set-output 0", moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  if (status != "OK") {
    std::cerr << "Error rezeroing servo, returned output: " << status  << std::endl;
    return 1;
  }
  return 0;
}


int RezeroServos(std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers) {
  for (auto& controller : moteus_controllers) {
    if (RezeroServo(controller) != 0) {
      return 1;
    }
  }
  return 0;
}


std::string ServoInfo(std::shared_ptr<mjbots::moteus::Controller> moteus_controller) {
  const auto servo_id = std::stoi(
      moteus_controller->DiagnosticCommand("conf get id.id", moteus::Controller::kExpectSingleLine)
  );
  moteus_controller->DiagnosticFlush();

  const auto servo_gear_ratio = std::stod(
      moteus_controller->DiagnosticCommand("conf get motor_position.rotor_to_output_ratio", moteus::Controller::kExpectSingleLine)
  );
  moteus_controller->DiagnosticFlush();

  const auto servo_direction = std::stod(
      moteus_controller->DiagnosticCommand("conf get motor_position.output.sign", moteus::Controller::kExpectSingleLine)
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

  // todo: add can-bus, direction

  std::ostringstream ostr;
  ostr << "Servo ID: " << servo_id << "; " << "Gear Ratio: " << servo_gear_ratio << "; " << "Direction: " << servo_direction << "\n"
      << "PID: kp=" << servo_kp << ", ki=" << servo_ki << ", kd=" << servo_kd << "\n"
      << "Position Limits: min=" << servo_limit_posmin << ", max=" << servo_limit_posmax
      << std::endl;
  
  return ostr.str();

}


std::string ServosInfo(std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers) {
  std::ostringstream ostr;
  for (auto& controller : moteus_controllers) {
    ostr << ServoInfo(controller);
  }
  return ostr.str();
}


int ServoConfSet(const ServoConfig& new_config, std::shared_ptr<mjbots::moteus::Controller> moteus_controller) {
  std::ostringstream ostr;
  std::string response;

  ostr << "conf set servo.pid_position.kp " << new_config.kp;
  response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  ostr.str("");
  if (response != "OK") {
    std::cerr << "Error setting kp, response: " << response << std::endl;
    return 1;
  }

  ostr << "conf set servo.pid_position.ki " << new_config.ki;
  response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  ostr.str("");
  if (response != "OK") {
    std::cerr << "Error setting ki, response: " << response << std::endl;
    return 1;
  }

  ostr << "conf set servo.pid_position.kd " << new_config.kd;
  response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  ostr.str("");
  if (response != "OK") {
    std::cerr << "Error setting kd, response: " << response << std::endl;
    return 1;
  }

  ostr << "conf set servopos.position_min " << new_config.position_min;
  response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  ostr.str("");
  if (response != "OK") {
    std::cerr << "Error setting position_min, response: " << response << std::endl;
    return 1;
  }

  ostr << "conf set servopos.position_max " << new_config.position_max;
  response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
  moteus_controller->DiagnosticFlush();
  ostr.str("");
  if (response != "OK") {
    std::cerr << "Error setting position_max, response: " << response << std::endl;
    return 1;
  }

  // optional parameters
  if (new_config.override_direction) {
    ostr << "conf set motor_position.output.sign " << new_config.direction;
    response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    ostr.str("");
    if (response != "OK") {
      std::cerr << "Error setting direction, response: " << response << std::endl;
      return 1;
    }
  }

  if (new_config.override_reduction_ratio) {
    ostr << "conf set motor_position.rotor_to_output_ratio " << new_config.reduction_ratio;
    response = moteus_controller->DiagnosticCommand(ostr.str(), moteus::Controller::kExpectSingleLine);
    moteus_controller->DiagnosticFlush();
    ostr.str("");
    if (response != "OK") {
      std::cerr << "Error setting reduction ratio, response: " << response << std::endl;
      return 1;
    }
  }

  return 0;
}


int ServosConfSet(const ServoConfig& new_config, std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers) {
  for (auto& controller : moteus_controllers) {
    if (ServoConfSet(new_config, controller) != 0) {
      return 1;
    }
  }
  return 0;
}

int ServosConfSet(const std::vector<ServoConfigWithID>& new_configs, std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers) {
  for (const auto& new_config_with_id : new_configs) {
    const int servo_id = new_config_with_id.id;

    const auto it = std::find_if(moteus_controllers.begin(), moteus_controllers.end(),
                                 [servo_id](const std::shared_ptr<mjbots::moteus::Controller>& controller) {
                                   return controller->options().id == servo_id;
                                 });

    if (it == moteus_controllers.end()) {
      std::cerr << "Error: No controller found with servo ID " << servo_id << std::endl;
      return 1;
    }

    if (ServoConfSet(new_config_with_id.config, *it) != 0) {
      return 1;
    }
  }

  return 0;
}

} // namespace mjbotscpp