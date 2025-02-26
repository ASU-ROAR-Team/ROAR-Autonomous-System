#ifndef NODE_LIFECYCLE_MANAGER_HPP
#define NODE_LIFECYCLE_MANAGER_HPP

#include <string>
#include <map>
#include <vector>
#include "supervisor/Config.hpp"
#include "supervisor/StateDatabase.hpp"
#include "supervisor/Logger.hpp"

namespace supervisor
{
    class NodeLifecycleManager
    {
    public:
        NodeLifecycleManager(ros::NodeHandle &nh, Config &config, StateDatabase &stateDB, Logger &logger);
        ~NodeLifecycleManager();

        // Start a specific node
        bool startNode(const std::string &nodeName);

        // Stop a specific node
        bool stopNode(const std::string &nodeName);

        // Restart a specific node
        bool restartNode(const std::string &nodeName);

        // Restart a node with saved state
        bool restartNodeWithState(const std::string &nodeName);

        // Start all nodes required for a task
        bool startTaskNodes(const std::string &taskName);

        // Stop all nodes that are not required for a new task
        bool transitionToTask(const std::string &taskName);

        // Check if a node is currently running
        bool isNodeRunning(const std::string &nodeName) const;

        // Get list of currently running nodes
        std::vector<std::string> getRunningNodes() const;

    private:
        ros::NodeHandle &nh_;
        Config &config_;
        StateDatabase &stateDB_;
        Logger &logger_;
        std::map<std::string, bool> runningNodes_;

        // TMUX session management
        bool createTmuxSession(const std::string &sessionName);
        bool sendTmuxCommand(const std::string &sessionName, const std::string &command);
        bool killTmuxSession(const std::string &sessionName);
        bool isTmuxSessionActive(const std::string &sessionName);

        // Parameter handling
        std::string createParamFile(const std::string &nodeName,
                                    const std::map<std::string, XmlRpc::XmlRpcValue> &params);
        void writeParamValue(std::ofstream &file, const XmlRpc::XmlRpcValue &value, int indent = 0);

        // Utility methods
        std::string sanitizeNodeName(const std::string &nodeName);
        std::string getTmuxSessionName(const std::string &nodeName);
    };

}

#endif // NODE_LIFECYCLE_MANAGER_HPP
