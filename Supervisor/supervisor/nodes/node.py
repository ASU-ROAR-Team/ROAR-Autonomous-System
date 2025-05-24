# pylint: disable=all
# mypy: ignore-errors
#!/usr/bin/env python3
from supervisor import ModuleManager, SupervisorLogger
import os
import rospy
import json
import signal
import sys
import atexit
from std_msgs.msg import String, Bool


class SupervisorNode:
    def __init__(self):
        rospy.init_node("rover_supervisor")

        # Load enhanced config
        with open(
            "/home/iaminfadel/ROAR-Simulation/src/ROAR-Autonomous-System/Supervisor/supervisor/config/config.json"
        ) as f:
            self.config = json.load(f)

        # self.controller_manager = ControllerManager(self.config)
        custom_log_path = self.config.get("logDir", None)
        self.logger = SupervisorLogger(
            log_dir=custom_log_path,
            log_prefix=f"rover_supervisor_{self.config.get('taskName', 'default')}",
        )
        self.moduleManager = ModuleManager(self.config, self.logger)

        # Register cleanup handlers
        rospy.on_shutdown(self.shutdown_hook)
        atexit.register(self.shutdown_hook)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # Set up heartbeat subscriber
        self.heartbeat_sub = rospy.Subscriber(
            "/system/heartbeat", String, self.heartbeat_callback, queue_size=10
        )

        for name, data in self.moduleManager.modules.items():
            self.moduleManager.start_module(name)

        # Set up monitoring timers
        self.resource_timer = rospy.Timer(rospy.Duration(1), self.monitor_resources)
        self.heartbeat_timer = rospy.Timer(rospy.Duration(0.5), self.check_heartbeats)

    def heartbeat_callback(self, msg):
        """Process heartbeat messages from modules"""
        try:
            # Expected format: "module_name:status"
            parts = msg.data.split(":", 1)
            if len(parts) == 2:
                module_name, status = parts
                self.moduleManager.update_heartbeat(module_name, status)
            else:
                # If no status provided, assume healthy
                self.moduleManager.update_heartbeat(parts[0])
        except Exception as e:
            rospy.logerr(f"Error processing heartbeat message: {e}")

    def monitor_resources(self, event):
        self.moduleManager.monitor_resources()

    def check_heartbeats(self, event):
        """Check for heartbeat timeouts"""
        failed_module = self.moduleManager.check_heartbeats()
        if failed_module:
            self.handle_module_failure(failed_module)

    def handle_module_failure(self, name):
        if self.moduleManager.modules[name].get("critical", False):
            rospy.logerr("Critical module failure - initiating emergency stop")
            # self.controller_manager.activate_backup()

            # Implement rover stop before restarting critical module
            stop_pub = rospy.Publisher("/emergency/stop", Bool, queue_size=1)
            stop_pub.publish(Bool(data=True))

            # Then restart the failed module
            self.moduleManager.restart_module(name)
        else:
            # For non-critical modules, just restart
            self.moduleManager.restart_module(name)

    def shutdown_hook(self):
        """Clean shutdown procedure to terminate all child processes"""
        rospy.loginfo("Shutdown requested - terminating all managed processes")
        if hasattr(self, "moduleManager"):
            self.moduleManager.terminate_all()

    def signal_handler(self, sig, frame):
        """Handle termination signals"""
        rospy.loginfo(f"Received signal {sig} - shutting down")
        self.shutdown_hook()
        sys.exit(0)


if __name__ == "__main__":
    try:
        node = SupervisorNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received - shutting down")
    finally:
        # Ensure cleanup happens even if there's an error during initialization
        if "node" in locals() and hasattr(node, "shutdown_hook"):
            node.shutdown_hook()
