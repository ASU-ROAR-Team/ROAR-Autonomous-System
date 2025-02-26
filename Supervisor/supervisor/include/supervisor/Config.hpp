#ifndef SUPERVISOR_CONFIG_HPP
#define SUPERVISOR_CONFIG_HPP
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include "supervisor/Types.hpp"

namespace supervisor {

class Config {
public:
    Config(const std::string& configPath);
    ~Config();

    // Load configuration from files
    bool loadConfig();

    // Get task configuration
    TaskConfig getTaskConfig(const std::string& taskName) const;

    // Get list of nodes for a task
    std::vector<std::string> getTaskNodeList(const std::string& taskName) const;

    // Get list of controllers for a task
    std::vector<std::string> getTaskControllers(const std::string& taskName) const;

    // Get node configuration
    NodeConfig getNodeConfig(const std::string& nodeName) const;

    // Get controller configuration
    ControllerConfig getControllerConfig(const std::string& controllerName) const;

    // Get heartbeat timeout for a node
    double getHeartbeatTimeout(const std::string& nodeName) const;

    // Get recovery strategy for a node and error type
    RecoveryStrategy getRecoveryStrategy(const std::string& nodeName, ErrorType error) const;

    // Get list of all available tasks
    std::vector<std::string> getAvailableTasks() const;

private:
    std::string configPath_;
    std::mutex configMutex_;

    // Configuration data
    std::map<std::string, TaskConfig> tasks_;
    std::map<std::string, NodeConfig> nodes_;
    std::map<std::string, ControllerConfig> controllers_;

    // Helper methods for parsing
    NodeConfig parseNodeConfig(const nlohmann::json& nodeJson);
    TaskConfig parseTaskConfig(const nlohmann::json& taskJson);
    ControllerConfig parseControllerConfig(const nlohmann::json& controllerJson);
};

}
#endif // SUPERVISOR_CONFIG_HPP
