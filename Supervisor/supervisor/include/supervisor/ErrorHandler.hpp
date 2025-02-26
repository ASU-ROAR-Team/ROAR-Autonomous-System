#ifndef ERROR_HANDLER_HPP
#define ERROR_HANDLER_HPP
#include <string>
#include <map>
#include "supervisor/Types.h"
#include "supervisor/Config.h"
#include "supervisor/NodeLifecycleManager.h"
#include "supervisor/StateDatabase.h"
#include "supervisor/Logger.h"

namespace supervisor {

class ErrorHandler {
public:
    ErrorHandler(Config& config, NodeLifecycleManager& lifecycleManager,
                StateDatabase& stateDB, Logger& logger);
    ~ErrorHandler();

    // Handle node error
    bool handleError(const std::string& nodeName, ErrorType error);

    // Resolve node failure based on configured strategy
    bool resolveNodeFailure(const std::string& nodeName, ErrorType error);

private:
    Config& config_;
    NodeLifecycleManager& lifecycleManager_;
    StateDatabase& stateDB_;
    Logger& logger_;
    std::map<std::string, int> errorCounts_;

    // Apply recovery strategy
    bool applyRecoveryStrategy(const std::string& nodeName, RecoveryStrategy strategy);

    // Specific recovery actions
    bool restartNode(const std::string& nodeName);
    bool restartNodeWithState(const std::string& nodeName);
    bool alertOperator(const std::string& nodeName, const std::string& message);
    bool emergencyStop(const std::string& reason);
};

}
#endif // ERROR_HANDLER_HPP
