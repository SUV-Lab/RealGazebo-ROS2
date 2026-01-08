#include "network_sim/tc_command_executor.hpp"

#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>

namespace network_sim
{

TCCommandExecutor::TCCommandExecutor(rclcpp::Logger logger)
: logger_(logger)
{
}

CommandResult TCCommandExecutor::execute(const std::string& command)
{
  CommandResult result;
  result.success = false;
  result.return_code = -1;
  result.output = "";
  result.error = "";

  // Prepend sudo to TC commands (needed for CAP_NET_ADMIN)
  // Add 2>&1 to capture both stdout and stderr
  std::string full_command = "sudo " + command + " 2>&1";

  // Execute command and capture output
  std::array<char, 128> buffer;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(full_command.c_str(), "r"), pclose);

  if (!pipe) {
    result.error = "popen() failed!";
    RCLCPP_ERROR(logger_, "Failed to execute command: %s", command.c_str());
    return result;
  }

  // Read output
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result.output += buffer.data();
  }

  // Get exit code
  result.return_code = pclose(pipe.release());

  // Check if command succeeded (exit code 0)
  result.success = (result.return_code == 0);

  if (!result.success) {
    result.error = result.output;
    RCLCPP_DEBUG(
      logger_,
      "Command failed with code %d: %s\nOutput: %s",
      result.return_code,
      command.c_str(),
      result.output.c_str());
  }

  return result;
}

bool TCCommandExecutor::execute_batch(const std::vector<std::string>& commands)
{
  for (const auto& cmd : commands) {
    auto result = execute(cmd);
    if (!result.success) {
      RCLCPP_ERROR(
        logger_,
        "Batch execution failed at command: %s",
        cmd.c_str());
      return false;
    }
  }
  return true;
}

}  // namespace network_sim
