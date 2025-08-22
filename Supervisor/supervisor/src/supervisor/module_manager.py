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
import paramiko


class ModuleManager:
    def __init__(self, config, logger):
        self.logger = logger
        self.modules = OrderedDict()
        self.processes = {}
        self.network_peers = []
        self.network_status = {}
        self.hostname = "xavier" #This is the host that the supervisor runs on
        self.state_lock = Lock()
        self.ssh_clients = {}  # persistent SSH connections
        self.load_config(config)
        self.init_ssh_clients()
        # Register cleanup function
        atexit.register(self.cleanup_ssh_connections)

    def init_ssh_clients(self):
        """Establish persistent SSH connections to remote hosts"""
        # Get all unique hosts from modules
        hosts = {node.get("host", self.hostname) for node in self.modules.values()}
        for host in hosts:
            if host != self.hostname:
                try:
                    client = paramiko.SSHClient()
                    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    client.connect(hostname=host, username="rover")
                    self.ssh_clients[host] = client
                    self.logger.info(f"SSH connection established to {host}")
                except Exception as e:
                    self.logger.error(f"Failed to establish SSH connection to {host}: {e}")

    def run_remote_cmd(self, host, cmd):
        """Helper to run a remote command using persistent SSH"""
        client = self.ssh_clients.get(host)
        if not client:
            self.logger.error(f"No SSH client for {host}, cannot run command: {cmd}")
            return "", 1
        
        try:
            # Check if connection is still alive and reconnect if needed
            if not client.get_transport() or not client.get_transport().is_active():
                self.logger.warn(f"SSH connection to {host} is inactive, reconnecting...")
                client.connect(hostname=host, username="rover")
                
            stdin, stdout, stderr = client.exec_command(cmd)
            out = stdout.read().decode().strip()
            err = stderr.read().decode().strip()
            if err:
                self.logger.debug(f"[{host}] stderr: {err}")
            return out, 0
        except Exception as e:
            self.logger.error(f"SSH command failed on {host}: {e}")
            # Try to reconnect on failure
            try:
                client.connect(hostname=host, username="rover")
                self.logger.info(f"Reconnected to {host} after failure")
            except Exception as reconnect_error:
                self.logger.error(f"Failed to reconnect to {host}: {reconnect_error}")
            return "", 1

    def cleanup_ssh_connections(self):
        """Close all SSH connections on exit"""
        for host, client in self.ssh_clients.items():
            try:
                client.close()
                self.logger.info(f"Closed SSH connection to {host}")
            except Exception as e:
                self.logger.error(f"Error closing SSH connection to {host}: {e}")

    def load_config(self, config):
        # Implement topological sort based on dependencies
        for node in sorted(config["nodes"], key=lambda x: len(x.get("dependencies", []))):
            self.modules[node["nodeName"]] = {
                "launchCmd": node["launchCmd"],
                "host": node["host"],
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
        self.network_peers = config.get("networkPeers", [])
        if self.network_peers:
            for p in self.network_peers:
                # seed runtime status
                self.network_status[p["name"]] = {
                    "last_ok": time.time(),
                    "consecutive_failures": 0
                }
            self.logger.info(f"Loaded {len(self.network_peers)} network peers for comm checks")


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

        module = self.modules[name]
        cmd = module["launchCmd"]

        if module.get("args"):
            cmd += " " + " ".join(module["args"])

        # Add parameters
        if "params" in module:
            for param_name, param_value in module["params"].items():
                cmd += f" _{param_name}:={param_value}"

        host = module.get("host", self.hostname)

        if host == self.hostname:
            # Local launch
            self.logger.info(f"Starting module {name} locally with command: {cmd}")
            proc = subprocess.Popen(cmd.split(), start_new_session=True)
            self.logger.info(f"Started {name} on {host}")
            self.processes[name] = {
                "process": proc,
                "last_heartbeat": time.time(),
                "start_time": time.time(),
                "status": "starting",
            }
        else:
            # Remote launch using persistent SSH
            self.logger.info(f"Starting module {name} on {host}")
            self.run_remote_cmd(host, f"nohup {cmd} >/dev/null 2>&1 &")
            self.logger.info(f"Started {name} on {host}")
            self.processes[name] = {
                "process": None,   # no PID for remote
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
            module = self.modules[name]
            host = module.get("host", self.hostname)

            try:
                if host == self.hostname:
                    # --- LOCAL monitoring with psutil ---
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

                else:
                    # --- REMOTE monitoring via persistent SSH + ps ---
                    launch_cmd = module["launchCmd"]
                    out, rc = self.run_remote_cmd(
                        host,
                        f"pgrep -f \"{launch_cmd}\" | xargs -r ps -o %cpu=,rss -p | tail -n +2"
                    )
                    if rc != 0 or not out.strip():
                        self.logger.warn(f"[REMOTE {host}] {name} not found running, restarting...")
                        self.restart_module(name)
                        continue

                    cpu_str, mem_str = out.strip().split()
                    cpu_normalized = float(cpu_str)  # already percentage across system
                    mem = float(mem_str) / 1024  # KB→MB (fixed conversion)

                    self.logger.info(
                        f"[REMOTE {host}] {name} CPU: {cpu_normalized:.1f}% Mem: {mem:.1f}MB Status: {data['status']}"
                    )

                # --- Limit enforcement ---
                limits = module.get("resourceLimits", {})
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


    def monitor_communication(self):
        """
        Check configured network peers using ping.
        Returns a list of peers that exceeded failureThreshold:
          [ (peer_dict, consecutive_failures), ... ]
        """
        failed_peers = []
        for peer in self.network_peers:
            name = peer.get("name")
            ip = peer.get("ip")
            timeout = peer.get("timeout", 1)
            threshold = peer.get("failureThreshold", 3)

            status = self.network_status.setdefault(name, {"last_ok": time.time(), "consecutive_failures": 0})
            ok = self.ping_host(ip, timeout)

            if ok:
                # success -> reset counters
                status["last_ok"] = time.time()
                if status["consecutive_failures"] != 0:
                    self.logger.info(f"Network restored: {name} ({ip}) after {status['consecutive_failures']} failures")
                status["consecutive_failures"] = 0
            else:
                status["consecutive_failures"] += 1
                self.logger.warn(f"Network ping failed for {name} ({ip}). Consecutive failures: {status['consecutive_failures']}")
                # Restart any local modules mapped to this peer after reaching threshold
                if status["consecutive_failures"] >= threshold:
                    failed_peers.append((peer, status["consecutive_failures"]))
                    # for mod in affected:
                    #     if mod in self.processes:
                    #         self.logger.warn(f"Restarting module '{mod}' due to network failure to peer '{name}'")
                    #         try:
                    #             self.restart_module(mod)
                    #         except Exception as e:
                    #             self.logger.error(f"Error restarting module {mod}: {e}")
        return failed_peers

    def ping_host(self, ip, timeout_sec=1):
        """Ping a host once. Return True if reachable."""
        try:
            # -c 1 : one packet, -W <timeout> : timeout in seconds (Linux ping)
            res = subprocess.run(
                ["ping", "-c", "1", "-W", str(int(timeout_sec)), ip],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            return res.returncode == 0
        except Exception as e:
            self.logger.warn(f"ping_host exception for {ip}: {e}")
            return False


    def get_running_cmds(self, host):
        """Return list of running commands from a host."""
        if host == self.hostname:
            return [" ".join(p.info["cmdline"]) 
                    for p in psutil.process_iter(attrs=["cmdline"]) if p.info["cmdline"]]
        else:
            out, rc = self.run_remote_cmd(host, "ps -eo cmd")
            if rc == 0:
                return out.splitlines()
            else:
                self.logger.error(f"Could not get process list from {host}")
                return []

    def restart_missing(self, host):
        """Restart modules that should be running on host but are not."""
        running_cmds = self.get_running_cmds(host)

        for name, module in self.modules.items():
            if module.get("host", self.hostname) != host:
                continue

            expected_cmd = module["launchCmd"]

            # Look for the expected command in the process list
            found = any(expected_cmd.split()[0] in cmd for cmd in running_cmds)

            if not found:
                self.logger.warn(f"{name} missing on {host}, restarting...")
                if host == self.hostname:
                    # Local restart
                    subprocess.Popen(expected_cmd.split(), start_new_session=True)
                else:
                    # Remote restart using persistent SSH
                    self.run_remote_cmd(host, f"nohup {expected_cmd} >/dev/null 2>&1 &")
                self.logger.info(f"Restarted {name} on {host}")

    def recover_all_hosts(self):
        """Call this when communication is restored."""
        hosts = {node.get("host", self.hostname) for node in self.modules.values()}
        for host in hosts:
            self.restart_missing(host)

    def check_heartbeats(self):
        """Check if any modules have missed heartbeats"""
        current_time = time.time()
        for name, data in list(self.processes.items()):
            if self.modules[name].get("heartBeat", False) is False:
                continue
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
        if name not in self.processes:
            return

        try:
            process_data = self.processes[name]
            host = self.modules[name].get("host", self.hostname)
            launch_cmd = self.modules[name]["launchCmd"]

            if host == self.hostname:
                # Local process termination
                proc = process_data["process"]
                if proc and proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self.logger.warn(f"Process {name} did not terminate gracefully, killing")
                        proc.kill()  # Force kill if it doesn't terminate

                    self.processes.pop(name)
                    self.logger.info(f"Terminated process {name}")

            else:
                # Remote process termination over persistent SSH
                self.run_remote_cmd(host, f"pgrep -fx \"{launch_cmd}\" | xargs -r kill -9")
                self.logger.info(f"Sent remote kill for {name} on {host}")
        
            self.processes.pop(name, None)
            self.logger.info(f"Terminated process {name} (on {host})")
        except Exception as e:
            self.logger.error(f"Error terminating process {name}: {e}")

    def terminate_all(self):
        """Terminate all managed processes"""
        self.logger.info("Terminating all managed processes...")
        # Terminate in reverse dependency order
        for name in reversed(list(self.processes.keys())):
            self.terminate_process(name)
        # Clean up SSH connections
        self.cleanup_ssh_connections()