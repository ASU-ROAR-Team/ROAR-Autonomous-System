"""
Config manager for the supervisor node

This module provides a class to manage configuration files for the supervisor node.
It allows loading, retrieving, and updating configuration data in JSON format.
"""
from typing import Dict, Any, Optional, cast
import json
import rospy


class ConfigManager:
    """
    Manager class for config files.

    Attributes
    ----------
    configFilePath : str
        Path to the config file.
    configData : Optional[Dict[str, Any]]
        JSON object of the config file. None if the file fails to load.
    """

    def __init__(self, configFilePath: str) -> None:
        """
        Initializes the ConfigManager with the path to the config file.

        Parameters
        ----------
        configFilePath : str
            The file path to the configuration file.
        """
        self.configFilePath: str = configFilePath
        self.configData: Optional[Dict[str, Any]] = self.loadConfig()

    def loadConfig(self) -> Optional[Dict[str, Any]]:
        """
        Loads the JSON object from the specified config file.

        Returns
        -------
        Optional[Dict[str, Any]]
            JSON object of the config file. Returns None if the file fails to load.

        Raises
        ------
        FileNotFoundError
            If the config file does not exist at the specified path.
        json.JSONDecodeError
            If the config file contains invalid JSON.
        IOError
            If there is an error reading the file.
        """
        try:
            with open(self.configFilePath, "r", encoding="utf-8") as file:
                return cast(Dict[str, Any], json.load(file))
        except FileNotFoundError as e:
            rospy.logerr(f"Configuration file not found: {e}")
            return None
        except json.JSONDecodeError as e:
            rospy.logerr(f"Invalid JSON in configuration file: {e}")
            return None
        except IOError as e:
            rospy.logerr(f"Error reading configuration file: {e}")
            return None

    def getTaskConfig(self, taskName: str) -> Optional[Dict[str, Any]]:
        """
        Retrieves the configuration for a specific task.

        Parameters
        ----------
        taskName : str
            The name of the task to retrieve the configuration for.

        Returns
        -------
        Optional[Dict[str, Any]]
            The configuration for the specified task if found, otherwise None.

        Raises
        ------
        KeyError
            If the task name is not found in the configuration.
        """
        if (
            self.configData
            and "taskName" in self.configData
            and self.configData["taskName"] == taskName
        ):
            return self.configData
        rospy.logerr(f"Task '{taskName}' not found in configuration.")
        return None

    def updateConfig(self, newConfig: Dict[str, Any]) -> None:
        """
        Updates the configuration data.

        Parameters
        ----------
        newConfig : Dict[str, Any]
            The new configuration data to be updated.

        Raises
        ------
        TypeError
            If the new configuration is not a dictionary.
        """
        if not isinstance(newConfig, dict):
            raise TypeError("New configuration must be a dictionary.")
        self.configData = newConfig
        rospy.loginfo("Configuration updated successfully.")
