#ifndef SUPERVISOR_STATE_DATABASE_HPP
#define SUPERVISOR_STATE_DATABASE_HPP

#include <string>
#include <map>
#include <mutex>
#include <ros/ros.h>

namespace supervisor {

// Simple state storage class
class StateDatabase {
public:
    StateDatabase(const std::string& dbPath = "");
    ~StateDatabase();

    // Save state for a node
    bool saveNodeState(const std::string& nodeName, const std::string& stateData);

    // Get state for a node
    std::string getNodeState(const std::string& nodeName);

    // Clear state for a node
    bool clearNodeState(const std::string& nodeName);

    // Check if state exists for a node
    bool hasState(const std::string& nodeName) const;

    // Initialize the database
    bool init();

private:
    std::string dbPath_;
    std::mutex dbMutex_;
    std::map<std::string, std::string> memoryCache_;

    // Save all states to disk
    bool saveToDisk();

    // Load states from disk
    bool loadFromDisk();
};

}

#endif // SUPERVISOR_STATE_DATABASE_HPP
