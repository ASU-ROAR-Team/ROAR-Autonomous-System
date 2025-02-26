#ifndef SUPERVISOR_CONTROLLER_MANAGER_HPP
#define SUPERVISOR_CONTROLLER_MANAGER_HPP
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <map>
#include <mutex>
#include "supervisor/Config.h"
#include "supervisor/Logger.h"

namespace supervisor {

class ControllerManager {
public:
    ControllerManager(ros::NodeHandle& nh, Config& config, Logger& logger);
    ~ControllerManager();

    // Initialize all controller publishers
    void init();

    // Enable a specific controller
    bool enableController(const std::string& controllerName);

    // Disable a specific controller
    bool disableController(const std::string& controllerName);

    // Enable controllers for a task, disable others
    bool configureControllersForTask(const std::string& taskName);

    // Check if a controller is enabled
    bool isControllerEnabled(const std::string& controllerName) const;

    // Get list of currently enabled controllers
    std::vector<std::string> getEnabledControllers() const;

private:
    ros::NodeHandle& nh_;
    Config& config_;
    Logger& logger_;
    std::map<std::string, ros::Publisher> controllerEnablePubs_;
    std::map<std::string, bool> controllerStates_;
    std::mutex stateMutex_;

    // Publish controller state
    void publishControllerState(const std::string& controllerName, bool enabled);

    // Handle exclusive controllers
    void disableExclusiveControllers(const std::string& enabledController);
};

}
#endif // SUPERVISOR_CONTROLLER_MANAGER_HPP
