#ifndef SUPERVISOR_TYPES_HPP
#define SUPERVISOR_TYPES_HPP
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <nlohmann/json.hpp>

namespace supervisor {

// Node status
enum class NodeStatus {
    RUNNING,
    WARNING,
    FAILED,
    UNKNOWN
};

// Error types
enum class ErrorType {
    TIMEOUT,
    CRASH,
    ABNORMAL_BEHAVIOR
};

// Recovery strategies
enum class RecoveryStrategy {
    RESTART,
    RESTART_WITH_STATE,
    ALERT_OPERATOR,
    EMERGENCY_STOP
};

// Node status information
struct NodeStatusInfo {
    NodeStatus status;
    ros::Time lastHeartbeat;
    std::string state;
    uint32_t lastSequence;
};

// Node configuration
struct NodeConfig {
    std::string launch_cmd;
    std::map<std::string, XmlRpc::XmlRpcValue> params;
    std::vector<std::string> args;
    std::map<std::string, std::string> env_vars;
    bool is_stateful;
    double heartbeat_timeout;
    std::map<ErrorType, RecoveryStrategy> recovery_strategies;
};

// Controller configuration
struct ControllerConfig {
    std::string enable_topic;
    int priority;
    bool exclusive;
};

// Task configuration
struct TaskConfig {
    std::vector<std::string> nodes;
    std::vector<std::string> controllers;
    std::string description;
};

}
#endif // SUPERVISOR_TYPES_HPP
