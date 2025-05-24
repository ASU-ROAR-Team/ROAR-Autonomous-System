# pylint: disable=all
# mypy: ignore-errors
import json
import subprocess
import time
import psutil
import rospy
import signal
import sys
import atexit
from threading import Lock
from collections import OrderedDict
from std_msgs.msg import Bool, String


class ModuleManager:
    def __init__(self, config, logger):
        self.logger = logger
        self.modules = OrderedDict()
        self.processes = {}
        self.state_lock = Lock()
        self.load_config(config)

    def load_config(self, config):
        # Implement topological sort based on dependencies
        for node in sorted(config["nodes"], key=lambda x: len(x.get("dependencies", []))):
            self.modules[node["nodeName"]] = {
                "launchCmd": node["launchCmd"],
                "params": node.get("params", {}),
                "args": node.get("args", []),
                "dependencies": node.get("dependencies", []),
                "critical": node.get("critical", False),
                "resourceLimits": node.get("resourceLimits", {}),
                "statePersistPath": node.get("statePersistPath", None),
                "heartbeatTimeout": node.get(
                    "heartbeatTimeout", 5.0
                ),  # Default timeout of 5 seconds
            }
            self.logger.info(f"Loaded module: {node['nodeName']}")

    def start_module(self, name):
        if name in self.processes:
            return

        # Check dependencies first
        for dep in self.modules[name]["dependencies"]:
            if dep not in self.processes:
                # Check if dependency is defined in config
                if dep not in self.modules:
                    self.logger.error(f"Dependency {dep} not found for module {name}")
                    # terminate all processes
                    self.terminate_all()
                    sys.exit(1)
                self.start_module(dep)

        cmd = self.modules[name]["launchCmd"]
        if self.modules[name]["args"]:
            cmd += " " + " ".join(self.modules[name]["args"])

        # Add real-time priorities
        proc = subprocess.Popen(cmd.split(), start_new_session=True)
        self.logger.info(f"Started {name} with PID {proc.pid}")
        self.processes[name] = {
            "process": proc,
            "last_heartbeat": time.time(),
            "start_time": time.time(),
            "status": "starting",
        }

    def update_heartbeat(self, module_name, status="healthy"):
        """Update the heartbeat timestamp for a module"""
        with self.state_lock:
            if module_name in self.processes:
                self.processes[module_name]["last_heartbeat"] = time.time()
                self.processes[module_name]["status"] = status
                self.logger.debug(f"Received heartbeat for {module_name}: {status}")
            else:
                self.logger.warn(f"Received heartbeat for unknown module: {module_name}")

    def monitor_resources(self):
        for name, data in list(self.processes.items()):
            try:
                # Get the process object and ensure it exists
                proc = psutil.Process(data["process"].pid)

                # Get CPU usage without setting interval in the same call
                # This prevents the blocking behavior that caused high CPU reports
                cpu = proc.cpu_percent()

                # Normalize CPU percentage to account for multiple cores
                # Dividing by psutil.cpu_count() gives per-core usage
                cpu_normalized = cpu / psutil.cpu_count()

                mem = proc.memory_info().rss / 1024 / 1024  # MB

                self.logger.info(
                    f"{name} CPU: {cpu_normalized:.1f}% Mem: {mem:.1f}MB Status: {data['status']}"
                )

                limits = self.modules[name]["resourceLimits"]
                if cpu_normalized > limits.get("maxCpu", 80):
                    self.logger.warn(f"{name} CPU exceeded: {cpu_normalized:.1f}%")
                    self.restart_module(name)
                if mem > limits.get("maxMem", 512):
                    self.logger.warn(f"{name} Memory exceeded: {mem:.1f}MB")
                    self.restart_module(name)

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess) as e:
                self.logger.warn(f"Process monitoring error for {name}: {e}")
                self.restart_module(name)
            except Exception as e:
                self.logger.error(f"Unexpected error monitoring {name}: {e}")

    def check_heartbeats(self):
        """Check if any modules have missed heartbeats"""
        current_time = time.time()
        for name, data in list(self.processes.items()):
            timeout = self.modules[name].get("heartbeatTimeout", 5.0)
            time_since_last = current_time - data["last_heartbeat"]

            # Mark as warning if we're approaching timeout
            if time_since_last > (timeout * 0.8) and data["status"] == "healthy":
                data["status"] = "warning"
                self.logger.warn(f"Module {name} heartbeat delayed: {time_since_last:.1f}s")

            # Consider it failed if we've exceeded timeout
            if time_since_last > timeout:
                self.logger.error(
                    f"Heartbeat timeout for {name}: {time_since_last:.1f}s > {timeout}s"
                )
                data["status"] = "failed"
                return name

        return None  # No failed heartbeats

    def restart_module(self, name):
        self.logger.warn(f"Restarting {name}")
        self.terminate_process(name)
        self.start_module(name)

    def terminate_process(self, name):
        """Terminate a specific process by name"""
        if name in self.processes:
            try:
                process_data = self.processes[name]
                proc = process_data["process"]

                # Try graceful termination first
                if proc.poll() is None:  # If process is still running
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)  # Wait up to 3 seconds for termination
                    except subprocess.TimeoutExpired:
                        self.logger.warn(f"Process {name} did not terminate gracefully, killing")
                        proc.kill()  # Force kill if it doesn't terminate

                self.processes.pop(name)
                self.logger.info(f"Terminated process {name}")
            except Exception as e:
                self.logger.error(f"Error terminating process {name}: {e}")

    def terminate_all(self):
        """Terminate all managed processes"""
        self.logger.info("Terminating all managed processes...")
        # Terminate in reverse dependency order
        for name in reversed(list(self.processes.keys())):
            self.terminate_process(name)
