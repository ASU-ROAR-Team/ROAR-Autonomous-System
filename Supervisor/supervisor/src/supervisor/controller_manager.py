# pylint: disable=all
# mypy: ignore-errors
from supervisor import ModuleManager


class ControllerManager:
    def __init__(self, config):
        self.controllers = {c["controllerName"]: c for c in config["controllers"]}
        self.active_controller = None
        self.backup_engaged = False

    def switch_controller(self, new_controller):
        required_modules = self.controllers[new_controller].get("requiredModules", [])
        if not all(m in module_manager.processes for m in required_modules):
            rospy.logerr("Missing required modules for controller activation")
            self.activate_backup()
            return

        if self.active_controller:
            self.deactivate_controller(self.active_controller)

        self.active_controller = new_controller
        rospy.loginfo(f"Switched to controller: {new_controller}")

    def activate_backup(self):
        backup = self.controllers[self.active_controller].get("backupController")
        if backup:
            self.switch_controller(backup)
            self.backup_engaged = True
