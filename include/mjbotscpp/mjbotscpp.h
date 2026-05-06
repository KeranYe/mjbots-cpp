#pragma once

#include <vector>
#include <memory>
#include <string>
#include <limits>
// #include <ostream>

#include "moteus.h"
// #include "pi3hat_moteus_transport.h"

using namespace mjbots;

namespace mjbotscpp {

// struct ServoConfig {

//   double kp = 400.0;
//   double ki = 1.0;
//   double kd = 2.0;
//   double position_min = -3.0; // rot
//   double position_max = 3.0; // rot

//   // optional
//   bool override_direction = false; // whether to override the default direction from the servo config
//   int direction = 1; // default to 1, set to -1 for reversed
//   bool override_reduction_ratio = false; // whether to override the default reduction ratio from the servo config
//   double reduction_ratio = 1.0/6.0; // default to 6:1 reduction, which is used in qdd100 servos
// };

struct ServoConfig {

  double kp = std::numeric_limits<double>::quiet_NaN();
  double ki = std::numeric_limits<double>::quiet_NaN();
  double kd = std::numeric_limits<double>::quiet_NaN();
  double position_min = std::numeric_limits<double>::quiet_NaN(); // rot
  double position_max = std::numeric_limits<double>::quiet_NaN(); // rot

  // optional
  bool override_direction = false; // whether to override the default direction from the servo config
  int direction = 1; // default to 1, set to -1 for reversed
  bool override_reduction_ratio = false; // whether to override the default reduction ratio from the servo config
  double reduction_ratio = 1.0/6.0; // default to 6:1 reduction, which is used in qdd100 servos
};

struct ServoConfigWithID {
  int id; // servo id
  int can; // can bus
  ServoConfig config;
};

/**
 * @brief Clear any errors on the specified servo.
 * 
 * @param moteus_controller A shared pointer to the moteus controller.
 * @return int Returns 0 on success, 1 on failure.
 */
int ClearServoError(std::shared_ptr<mjbots::moteus::Controller> moteus_controller); 
// {
//   moteus_controller->DiagnosticWrite("tel stop\n");
//   moteus_controller->DiagnosticFlush();
//   return 0;
// }

/**
 * @brief Stop the specified servo by sending a stop command and checking the response.
 * 
 * @param moteus_controller A shared pointer to the moteus controller.
 * @return int Returns 0 on success, 1 on failure.
 */
int StopServo(std::shared_ptr<mjbots::moteus::Controller> moteus_controller); 


/**
 * @brief Stop all servos with the provided controllers.
 * 
 * @param moteus_controllers A vector of shared pointers to the moteus controllers.
 * @return int Returns 0 on success, 1 on failure.
 */
int StopServos(std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers); 

/**
 * @brief Rezero the specified servo by sending a configuration command and checking the response.
 * 
 * @param moteus_controller A shared pointer to the moteus controller.
 * @return int Returns 0 on success, 1 on failure.
 */
int RezeroServo(std::shared_ptr<mjbots::moteus::Controller> moteus_controller); 


/**
 * @brief Rezero all servos with the provided controllers.
 * 
 * @param moteus_controllers A vector of shared pointers to the moteus controllers.
 * @return int Returns 0 on success, 1 on failure.
 */
int RezeroServos(std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers);  

/**
 * @brief Fetch information about the specified servo by sending diagnostic commands and checking the responses.
 * 
 * @param moteus_controller A shared pointer to the moteus controller.
 * @return std::string Returns a string with the servo information.
 */
std::string ServoInfo(std::shared_ptr<mjbots::moteus::Controller> moteus_controller); 

/**
 * @brief Fetch information about all servos with the provided controllers.
 * 
 * @param moteus_controllers A vector of shared pointers to the moteus controllers.
 * @return std::string Returns a string with the information of all servos.
 */
std::string ServosInfo(std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers);  

/**
 * @brief Set the configuration of the specified servo by sending diagnostic commands and checking the responses.
 * 
 * @param new_config A ServoConfig struct with the new configuration values.
 * @param moteus_controller A shared pointer to the moteus controller.
 * @return int Returns 0 on success, 1 on failure.
 */
int ServoConfSet(const ServoConfig& new_config, std::shared_ptr<mjbots::moteus::Controller> moteus_controller); 

/**
 * @brief Set the same configuration for all servos with the provided controllers by sending diagnostic commands and checking the responses.
 * 
 * @param new_config A ServoConfig struct with the new configuration values.
 * @param moteus_controllers A vector of shared pointers to the moteus controllers.
 * @return int Returns 0 on success, 1 on failure.
 */
int ServosConfSet(const ServoConfig& new_config, std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers); 

/**
 * @brief Set individual configurations for each servo with the provided controllers by sending diagnostic commands and checking the responses.
 * 
 * @param new_configs A vector of ServoConfig structs with the new configuration values for each servo.
 * @param moteus_controllers A vector of shared pointers to the moteus controllers.
 * @return int Returns 0 on success, 1 on failure.
 */
int ServosConfSet(const std::vector<ServoConfigWithID>& new_configs, std::vector<std::shared_ptr<mjbots::moteus::Controller>> &moteus_controllers);

} // namespace mjbotscpp