#!/usr/bin/env python3
"""
Parameter Checker Script for ZED Obstacle Detector
Checks if all parameters used in C++ code are present in YAML files and vice versa.
Supports namespaced parameters (e.g., processing/voxel_leaf_size).
"""

import os
import re
import yaml
import glob
from pathlib import Path

class ParameterChecker:
    def __init__(self, package_root):
        self.package_root = Path(package_root)
        self.cpp_params = set()
        self.yaml_params = set()
        
    def extract_cpp_parameters(self):
        """Extract all parameter names from C++ files"""
        cpp_files = [
            self.package_root / "zed_obstacle_detector_node.cpp",
            self.package_root / "src" / "*.cpp",
            self.package_root / "include" / "zed_obstacle_detector" / "*.h"
        ]
        
        for pattern in cpp_files:
            for file_path in glob.glob(str(pattern)):
                with open(file_path, 'r') as f:
                    content = f.read()
                    
                # Find nh.param and nh.getParam calls with namespaced parameters
                param_patterns = [
                    r'nh\.param\s*\(\s*["\']([^"\']+)["\']',  # nh.param("param_name", ...)
                    r'nh\.getParam\s*\(\s*["\']([^"\']+)["\']',  # nh.getParam("param_name", ...)
                ]
                
                for pattern in param_patterns:
                    matches = re.findall(pattern, content)
                    self.cpp_params.update(matches)
    
    def extract_nested_params(self, data, prefix=""):
        """Recursively extract all nested parameter names from a dictionary"""
        params = set()
        for key, value in data.items():
            current_key = f"{prefix}/{key}" if prefix else key
            if isinstance(value, dict):
                # Recursively extract nested parameters
                params.update(self.extract_nested_params(value, current_key))
            else:
                # This is a leaf parameter
                params.add(current_key)
        return params
    
    def extract_yaml_parameters(self):
        """Extract all parameter names from YAML files (including nested namespaces)"""
        yaml_files = [
            self.package_root / "config" / "params.yaml",
            self.package_root / "config" / "test_params.yaml", 
            self.package_root / "config" / "high_performance_params.yaml"
        ]
        
        for yaml_file in yaml_files:
            if yaml_file.exists():
                with open(yaml_file, 'r') as f:
                    try:
                        data = yaml.safe_load(f)
                        if data and 'zed_obstacle_detector' in data:
                            params = data['zed_obstacle_detector']
                            # Extract all nested parameters
                            nested_params = self.extract_nested_params(params)
                            self.yaml_params.update(nested_params)
                            print(f"📁 Loaded {len(nested_params)} parameters from {yaml_file.name}")
                    except yaml.YAMLError as e:
                        print(f"❌ Error parsing {yaml_file}: {e}")
    
    def check_parameters(self):
        """Check for missing parameters"""
        print("=== Parameter Checker Report ===\n")
        
        # Parameters in C++ but not in YAML
        missing_in_yaml = self.cpp_params - self.yaml_params
        if missing_in_yaml:
            print("❌ Parameters used in C++ but missing from YAML files:")
            for param in sorted(missing_in_yaml):
                print(f"   - {param}")
        else:
            print("✅ All C++ parameters are present in YAML files")
        
        print()
        
        # Parameters in YAML but not in C++
        unused_in_yaml = self.yaml_params - self.cpp_params
        if unused_in_yaml:
            print("⚠️  Parameters in YAML but not used in C++ (potential dead config):")
            for param in sorted(unused_in_yaml):
                print(f"   - {param}")
        else:
            print("✅ All YAML parameters are used in C++ code")
        
        print()
        print(f"📊 Summary:")
        print(f"   C++ parameters: {len(self.cpp_params)}")
        print(f"   YAML parameters: {len(self.yaml_params)}")
        print(f"   Missing in YAML: {len(missing_in_yaml)}")
        print(f"   Unused in YAML: {len(unused_in_yaml)}")
        
        # Show some examples of the parameters found
        if self.cpp_params:
            print(f"\n📝 Sample C++ parameters found:")
            for param in sorted(list(self.cpp_params)[:5]):
                print(f"   - {param}")
            if len(self.cpp_params) > 5:
                print(f"   ... and {len(self.cpp_params) - 5} more")
        
        if self.yaml_params:
            print(f"\n📝 Sample YAML parameters found:")
            for param in sorted(list(self.yaml_params)[:5]):
                print(f"   - {param}")
            if len(self.yaml_params) > 5:
                print(f"   ... and {len(self.yaml_params) - 5} more")
        
        return len(missing_in_yaml) == 0

def main():
    # Get the package root directory
    script_dir = Path(__file__).parent
    package_root = script_dir.parent
    
    checker = ParameterChecker(package_root)
    checker.extract_cpp_parameters()
    checker.extract_yaml_parameters()
    
    success = checker.check_parameters()
    
    if success:
        print("\n🎉 All parameters are properly configured!")
        return 0
    else:
        print("\n❌ Parameter configuration issues found!")
        return 1

if __name__ == "__main__":
    exit(main()) 