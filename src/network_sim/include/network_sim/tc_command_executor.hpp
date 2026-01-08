#ifndef NETWORK_SIM__TC_COMMAND_EXECUTOR_HPP_
#define NETWORK_SIM__TC_COMMAND_EXECUTOR_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace network_sim
{

struct CommandResult {
  bool success;
  int return_code;
  std::string output;
  std::string error;
};

class TCCommandExecutor
{
public:
  explicit TCCommandExecutor(rclcpp::Logger logger);

  // Execute a shell command and return result
  CommandResult execute(const std::string& command);

  // Execute multiple commands sequentially
  bool execute_batch(const std::vector<std::string>& commands);

private:
  rclcpp::Logger logger_;
};

}  // namespace network_sim

#endif  // NETWORK_SIM__TC_COMMAND_EXECUTOR_HPP_
