#ifndef SUPERVISOR_SUPERVISORNODE_HPP
#define SUPERVISOR_SUPERVISORNODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mutex>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include "supervisor/HeartbeatChecker.hpp"
#include "supervisor/NodeLifecycleManager.hpp"
#include "supervisor/ErrorHandler.hpp"
#include "supervisor/StateDatabase.hpp"
#include "supervisor/ControllerManager.hpp"
#include "supervisor/Config.hpp"
#include "supervisor/Logger.hpp"
#include "supervisor/SupervisorStatus.hpp"
#include "supervisor/SetTask.hpp"

namespace supervisor {

class SupervisorNode {
public:
    SupervisorNode(const std::string& configPath);
    ~SupervisorNode();

    // Initialize the node
    bool init();

    // Main run loop
    void run();

    // Shutdown all components
    void shutdown();

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Components
    std::unique_ptr<Config> config_;
    std::unique_ptr<HeartbeatChecker> heartbeatChecker_;
    std::unique_ptr<NodeLifecycleManager> lifecycleManager_;
    std::unique_ptr<StateDatabase> stateDB_;
    std::unique_ptr<ErrorHandler> errorHandler_;
    std::unique_ptr<ControllerManager> controllerManager_;
    std::unique_ptr<Logger> logger_;

    // Parameters
    double heartbeat_check_interval_;
    std::string current_task_;
    std::string config_path_;
    std::string log_path_;
    std::string state_db_path_;

    // ROS interfaces
    ros::Subscriber system_monitor_sub_;
    ros::ServiceServer set_task_service_;
    ros::Publisher status_pub_;
    ros::Timer heartbeat_timer_;

    // Mutex for task switching
    std::mutex task_mutex_;

    // Callback methods
    void heartbeatTimerCallback(const ros::TimerEvent& event);
    bool setTaskCallback(supervisor::SetTask::Request& req,
                        supervisor::SetTask::Response& res);
    void systemMonitorCallback(const supervisor::SystemStatus::ConstPtr& msg);
    void publishStatus();

    // Task management
    bool switchToTask(const std::string& taskName);
};

}

#endif // SUPERVISOR_SUPERVISORNODE_HPP
