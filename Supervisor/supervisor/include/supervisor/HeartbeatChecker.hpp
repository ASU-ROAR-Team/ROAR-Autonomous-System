#ifndef HEARTBEAT_CHECKER_HPP
#define HEARTBEAT_CHECKER_HPP

#include <ros/ros.h>
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include "roar_msgs/Hearbeat.h"
#include "supervisor/Types.hpp"

namespace supervisor {

class HeartbeatChecker {
public:
    HeartbeatChecker(ros::NodeHandle& nh, const Config& config);
    ~HeartbeatChecker();

    // Subscribe to heartbeats for a list of nodes
    void monitorNodes(const std::vector<std::string>& nodeNames);

    // Stop monitoring specific nodes
    void stopMonitoring(const std::vector<std::string>& nodeNames);

    // Check all monitored nodes and return list of failed nodes
    std::vector<std::string> checkHeartbeats();

    // Get current status of all monitored nodes
    std::map<std::string, NodeStatusInfo> getNodeStatuses();

private:
    ros::NodeHandle& nh_;
    const Config& config_;
    std::map<std::string, ros::Subscriber> heartbeatSubs_;
    std::map<std::string, NodeStatusInfo> nodeStatuses_;
    std::mutex statusMutex_;

    // Heartbeat callback
    void heartbeatCallback(const supervisor::Heartbeat::ConstPtr& msg, const std::string& nodeName);
};

}

#endif // HEARTBEAT_CHECKER_HPP
