#ifndef SUPERVISOR_LOGGER_HPP
#define SUPERVISOR_LOGGER_HPP

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <mutex>
#include "supervisor/LogEvent.hpp"
#include "supervisor/Types.hpp"

namespace supervisor {

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger {
public:
    Logger(ros::NodeHandle& nh, const std::string& logPath = "");
    ~Logger();

    // Initialize the logger
    bool init();

    // Log a general event
    void logEvent(LogLevel level, const std::string& component,
                 const std::string& message);

    // Log a node event
    void logNodeEvent(const std::string& nodeName, LogLevel level,
                     const std::string& message);

    // Log a controller event
    void logControllerEvent(const std::string& controllerName, LogLevel level,
                          const std::string& message);

    // Log a task transition
    void logTaskTransition(const std::string& fromTask, const std::string& toTask);

    // Log an error
    void logError(const std::string& nodeName, ErrorType error,
                 const std::string& details);

private:
    ros::NodeHandle& nh_;
    std::string logPath_;
    ros::Publisher logPub_;
    std::ofstream logFile_;
    std::mutex logMutex_;

    // Helper to format and write logs
    void writeLog(LogLevel level, const std::string& component,
                 const std::string& message);

    // Convert log level to string
    std::string logLevelToString(LogLevel level);

    // Get timestamp string
    std::string getTimestamp();
};

}

#endif // SUPERVISOR_LOGGER_HPP
