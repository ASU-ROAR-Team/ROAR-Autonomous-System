# pylint: disable=all
# mypy: ignore-errors
import os
import logging
import rospy
from logging.handlers import RotatingFileHandler
import traceback
from datetime import datetime


class SupervisorLogger:
    def __init__(
        self, log_dir=None, log_prefix="rover_supervisor", max_log_size_mb=10, backup_count=5
    ):
        """
        Initialize a comprehensive logging system

        Args:
            log_dir (str, optional): Custom directory for log files.
                                     Defaults to ~/.ros/log/supervisor
            log_prefix (str, optional): Prefix for log files
            max_log_size_mb (int, optional): Maximum log file size before rotation
            backup_count (int, optional): Number of backup log files to keep
        """
        # Determine log directory
        if log_dir is None:
            log_dir = os.path.expanduser("~/.ros/log/supervisor")

        # Ensure log directory exists
        os.makedirs(log_dir, exist_ok=True)

        # Create timestamp for unique log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"{log_prefix}_{timestamp}.log"
        self.log_path = os.path.join(log_dir, log_filename)

        # Configure logging
        self.logger = logging.getLogger("SupervisorLogger")
        self.logger.setLevel(logging.DEBUG)

        # Clear any existing handlers to prevent duplicate logging
        self.logger.handlers.clear()

        # Create file handler with rotation
        file_handler = RotatingFileHandler(
            self.log_path,
            maxBytes=max_log_size_mb * 1024 * 1024,  # Convert MB to bytes
            backupCount=backup_count,
        )
        file_handler.setLevel(logging.DEBUG)

        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # Create formatters
        file_formatter = logging.Formatter(
            "%(asctime)s | %(levelname)s | %(module)s | %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
        )
        console_formatter = logging.Formatter("%(levelname)s: %(message)s")

        # Set formatters
        file_handler.setFormatter(file_formatter)
        console_handler.setFormatter(None)

        # Add handlers
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)

        # Log initialization
        self.info(f"Logger initialized. Log file: {self.log_path}")

    def debug(self, message, *args, **kwargs):
        """Debug level logging with ROS logging"""
        rospy.logdebug(message)
        self.logger.debug(message, *args, **kwargs)

    def info(self, message, *args, **kwargs):
        """Info level logging with ROS logging"""
        rospy.loginfo(message)
        self.logger.info(message, *args, **kwargs)

    def warn(self, message, *args, **kwargs):
        """Warning level logging with ROS logging"""
        rospy.logwarn(message)
        self.logger.warning(message, *args, **kwargs)

    def error(self, message, *args, **kwargs):
        """Error level logging with ROS logging"""
        rospy.logerr(message)
        self.logger.error(message, *args, **kwargs)

    def critical(self, message, *args, **kwargs):
        """Critical level logging with ROS logging"""
        rospy.logerr(message)  # ROS doesn't have a distinct critical log level
        self.logger.critical(message, *args, **kwargs)

    def log_exception(self, message=None):
        """
        Log an exception with full traceback

        Args:
            message (str, optional): Custom message to prepend to exception details
        """
        exc_info = traceback.format_exc()
        full_message = f"{message or 'An exception occurred:'}\n{exc_info}"

        rospy.logerr(full_message)
        self.logger.error(full_message)

    def get_log_path(self):
        """
        Returns the current log file path

        Returns:
            str: Path to the current log file
        """
        return self.log_path

    def close(self):
        """
        Close the logger handlers
        """
        for handler in self.logger.handlers[:]:
            handler.close()
            self.logger.removeHandler(handler)
