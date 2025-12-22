# ROS2 Python Coding Standards Guide

This guide provides coding standards and best practices for writing Python code in ROS2 packages that will pass all linter tests (copyright, flake8, pep257).

---

## Table of Contents

1. [File Structure](#file-structure)
2. [Copyright Headers](#copyright-headers)
3. [Import Statements](#import-statements)
4. [Code Formatting](#code-formatting)
5. [Docstrings](#docstrings)
6. [Classes](#classes)
7. [Functions and Methods](#functions-and-methods)
8. [Common Errors and Fixes](#common-errors-and-fixes)
9. [Auto-formatting Tools](#auto-formatting-tools)

---

## File Structure

Every Python file should follow this structure:

```python
# Copyright header (see below)

"""Module-level docstring describing the file."""

# Standard library imports
import os
import sys
from typing import Optional, List, Dict

# Third-party imports
import numpy as np
import rclpy
from rclpy.node import Node

# Local/package imports
from robogpt_node_stack.block_programming.tools.tool_loader import ToolLoader


# Constants (if any)
MAX_ITERATIONS = 100


# Classes and functions
class MyClass:
    """Class description."""
    pass


def my_function():
    """Function description."""
    pass
```

**Key Rules:**
- Copyright header at the very top
- Module docstring after copyright
- All imports at the top (before any other code)
- Group imports: standard lib → third-party → local
- 2 blank lines before classes/functions at module level

---

## Copyright Headers

**Every Python file must start with a copyright header.**

### Template:

```python
# Copyright 2025 Orangewood Labs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

**Note:** Update the year (2024) to the current year for new files.

---

## Import Statements

### Order and Grouping

```python
# 1. Standard library imports (alphabetical)
import json
import os
import sys
from typing import Dict, List, Optional

# 2. Third-party imports (alphabetical)
import numpy as np
import rclpy
from rclpy.node import Node

# 3. Local/application imports (alphabetical)
from robogpt_node_stack.block_programming.node_flow.flow_parser import JsonFlowParser
from robogpt_tools.applications.utilities.robotiqgripper import RobotiqGripperClient
```

### Rules:

✅ **DO:**
- Put all imports at the top of the file
- Group imports by category (standard → third-party → local)
- Use absolute imports, not relative
- Import only what you need
- Keep imports alphabetical within each group

❌ **DON'T:**
- Import after code execution
- Use `import *` (star imports)
- Import unused modules
- Mix import groups

### Examples:

```python
# ❌ WRONG - imports after code
import sys
sys.path.append("/some/path")
import rclpy  # E402: module level import not at top

# ✅ CORRECT - all imports first
import sys
import rclpy

# Later in the code
sys.path.append("/some/path")
```

```python
# ❌ WRONG - star imports
from tool_initializers import *  # F403: unable to detect undefined names

# ✅ CORRECT - explicit imports
from tool_initializers import MasterNode, robot_home_file_path
```

---

## Code Formatting

### Line Length

**Maximum 99 characters per line.**

```python
# ❌ WRONG - line too long (E501)
self.config_path = f"/home/{getpass.getuser()}/orangewood_ws/src/robogpt_tools/robot_config/bot_details.json"

# ✅ CORRECT - split long lines
self.config_path = (
    f"/home/{getpass.getuser()}/orangewood_ws/src/"
    "robogpt_tools/robot_config/bot_details.json"
)

# ✅ CORRECT - for function calls
result = my_function(
    argument1=value1,
    argument2=value2,
    argument3=value3
)
```

### Spacing

#### Around Operators

```python
# ❌ WRONG
x=5+3  # E225: missing whitespace around operator
y = 10*2

# ✅ CORRECT
x = 5 + 3
y = 10 * 2
```

#### After Commas

```python
# ❌ WRONG
my_list = [1,2,3]  # E231: missing whitespace after ','
func(a,b,c)

# ✅ CORRECT
my_list = [1, 2, 3]
func(a, b, c)
```

#### In Function Definitions

```python
# ❌ WRONG - spaces around = in default arguments
def func(arg1 = None, arg2 = 5):  # E251: unexpected spaces around =
    pass

# ✅ CORRECT - no spaces around = for defaults
def func(arg1=None, arg2=5):
    pass

# ✅ CORRECT - but spaces around = in calls
func(arg1=10, arg2=20)
```

### Blank Lines

```python
# ❌ WRONG - only 1 blank line between classes
class FirstClass:
    pass
class SecondClass:  # E302: expected 2 blank lines
    pass

# ✅ CORRECT - 2 blank lines between top-level definitions
class FirstClass:
    pass


class SecondClass:
    pass


def standalone_function():
    pass
```

```python
# ✅ CORRECT - blank lines in classes
class MyClass:
    """Class docstring."""

    def __init__(self):
        """Initialize."""
        pass

    def method_one(self):
        """First method."""
        pass

    def method_two(self):
        """Second method."""
        pass
```

### Trailing Whitespace

```python
# ❌ WRONG - spaces/tabs at end of lines (invisible)
def foo():
    return True

# ✅ CORRECT - no trailing spaces
def foo():
    return True
```

**Tip:** Configure your editor to show and remove trailing whitespace.

---

## Docstrings

### Module-Level Docstrings

```python
# Copyright header...

"""
Module for parsing and executing flow programs.

This module provides classes for reading JSON flow definitions,
validating them, and executing the defined sequences.
"""

import os
# ... rest of file
```

### Class Docstrings

```python
class FlowExecutor:
    """Execute flow programs from JSON definitions."""

    def __init__(self):
        """Initialize the flow executor."""
        self.tools = []
```

**Rules:**
- One-line docstrings on a single line with quotes
- Multi-line docstrings: opening quotes on first line, content starts next line
- 1 blank line after class docstring before first method
- End with a period

### Function/Method Docstrings

#### Simple Functions (One-liner)

```python
def get_status():
    """Return the current execution status."""
    return self._status
```

#### Functions with Parameters (Google Style)

```python
def execute_node(self, node_type, arguments):
    """
    Execute a single node in the flow.

    Args:
        node_type: Type of the node to execute
        arguments: Dictionary of arguments for the node

    Returns:
        Execution result or None on failure

    Raises:
        ValueError: If node_type is invalid
    """
    pass
```

#### Functions with Parameters (NumPy Style - Alternative)

```python
def execute_node(self, node_type, arguments):
    """
    Execute a single node in the flow.

    Parameters
    ----------
    node_type : str
        Type of the node to execute
    arguments : dict
        Dictionary of arguments for the node

    Returns
    -------
    result : object or None
        Execution result or None on failure

    Raises
    ------
    ValueError
        If node_type is invalid
    """
    pass
```

### Docstring Rules

✅ **DO:**
- Start with a one-line summary ending with a period
- Use imperative mood ("Execute", not "Executes")
- Add blank line before Args/Returns sections
- Document all parameters and return values
- End first line with punctuation (. ? !)

❌ **DON'T:**
- Use multi-line format for short descriptions
- Forget the period at the end
- Omit parameter descriptions
- Use indicative mood ("This function executes...")

### Common Docstring Errors

```python
# ❌ WRONG - D200: multi-line for one-liner
class MyClass:
    """
    Simple description
    """

# ✅ CORRECT
class MyClass:
    """Simple description."""


# ❌ WRONG - D204: missing blank line after class docstring
class MyClass:
    """Simple description."""
    def __init__(self):
        pass

# ✅ CORRECT
class MyClass:
    """Simple description."""

    def __init__(self):
        pass


# ❌ WRONG - D400, D415: missing period
def my_func():
    """Do something"""

# ✅ CORRECT
def my_func():
    """Do something."""


# ❌ WRONG - D401: not imperative mood
def load_data():
    """Loads data from file."""

# ✅ CORRECT
def load_data():
    """Load data from file."""


# ❌ WRONG - D407, D413: missing formatting
def process(data):
    """
    Process the data.

    Args:
        data: Input data
    Returns:
        Processed data
    """

# ✅ CORRECT - proper spacing
def process(data):
    """
    Process the data.

    Args:
        data: Input data

    Returns:
        Processed data
    """
```

---

## Classes

### Class Template

```python
class MyRobotController:
    """
    Control robot movements and operations.

    This class provides an interface for controlling robot movements,
    managing tool operations, and handling error states.

    Attributes:
        robot_name: Name of the connected robot
        is_connected: Connection status
    """

    def __init__(self, robot_name, ip_address=None):
        """
        Initialize the robot controller.

        Args:
            robot_name: Name of the robot to control
            ip_address: Optional IP address for connection
        """
        self.robot_name = robot_name
        self.ip_address = ip_address
        self.is_connected = False

    def connect(self):
        """Establish connection to the robot."""
        # Implementation
        pass

    def disconnect(self):
        """Close connection to the robot."""
        # Implementation
        pass

    def _internal_method(self):
        """Internal helper method (private)."""
        # Implementation
        pass
```

### Class Rules:

- Class names in `PascalCase`
- 2 blank lines before class definition (at module level)
- 1 blank line after class docstring
- 1 blank line between methods
- Use `_prefix` for internal/private methods

---

## Functions and Methods

### Function Template

```python
def calculate_trajectory(start_pose, end_pose, speed=1.0):
    """
    Calculate trajectory between two poses.

    Args:
        start_pose: Starting pose as [x, y, z, rx, ry, rz]
        end_pose: Target pose as [x, y, z, rx, ry, rz]
        speed: Movement speed multiplier (default: 1.0)

    Returns:
        List of intermediate poses forming the trajectory

    Raises:
        ValueError: If poses are invalid or unreachable
    """
    if not _validate_pose(start_pose):
        raise ValueError("Invalid start pose")

    trajectory = []
    # Implementation
    return trajectory


def _validate_pose(pose):
    """Validate pose format and values (internal helper)."""
    return len(pose) == 6
```

### Function Rules:

- Function names in `snake_case`
- Use type hints in signatures (optional but recommended)
- First line of docstring: imperative mood, end with period
- Document all parameters, return values, and exceptions
- Private functions start with `_`

### With Type Hints

```python
from typing import List, Optional, Dict


def get_tool_map(tool_names: List[str]) -> Dict[str, object]:
    """
    Create a mapping of tool names to tool instances.

    Args:
        tool_names: List of tool names to map

    Returns:
        Dictionary mapping names to tool instances
    """
    return {name: load_tool(name) for name in tool_names}


def find_robot(name: str, robots: List[object]) -> Optional[object]:
    """
    Find robot by name in list.

    Args:
        name: Robot name to search for
        robots: List of robot instances

    Returns:
        Robot instance if found, None otherwise
    """
    for robot in robots:
        if robot.name == name:
            return robot
    return None
```

---

## Common Errors and Fixes

### Import Errors

| Error | Problem | Fix |
|-------|---------|-----|
| E402 | Import not at top | Move all imports to file top |
| F401 | Imported but unused | Remove or use the import |
| F403 | Star import | Use explicit imports |
| F405 | Name may be undefined (star import) | Use explicit imports |

### Spacing Errors

| Error | Problem | Fix |
|-------|---------|-----|
| E225 | Missing whitespace around operator | Add spaces: `x = 5 + 3` |
| E231 | Missing whitespace after comma | Add space: `[1, 2, 3]` |
| E251 | Unexpected spaces around = | Remove: `func(arg=5)` |
| E252 | Missing whitespace around = | Add in assignments: `x = 5` |

### Whitespace Errors

| Error | Problem | Fix |
|-------|---------|-----|
| W291 | Trailing whitespace | Remove spaces at line end |
| W293 | Blank line contains whitespace | Make blank lines completely empty |
| W292 | No newline at end of file | Add blank line at file end |

### Blank Line Errors

| Error | Problem | Fix |
|-------|---------|-----|
| E302 | Expected 2 blank lines | Add blank line before class/function |
| E303 | Too many blank lines | Remove extra blank lines |
| E305 | Expected 2 blank lines after class | Add blank line after class |

### Line Length

| Error | Problem | Fix |
|-------|---------|-----|
| E501 | Line too long (>99 chars) | Split into multiple lines |

### Docstring Errors

| Error | Problem | Fix |
|-------|---------|-----|
| D200 | One-liner should fit on one line | Use: `"""Description."""` |
| D204 | Missing blank line after docstring | Add blank line |
| D400, D415 | Missing period | End with period |
| D401 | Not imperative mood | Use "Load" not "Loads" |
| D407 | Missing dashed underline | Not needed for Google style |
| D413 | Missing blank line after section | Add blank after Args/Returns |
| D417 | Missing argument descriptions | Document all parameters |

---

## Auto-formatting Tools

### Using Black (Code Formatter)

```bash
# Install
pip install black

# Format a file
black my_file.py

# Format entire package
black .

# Check without modifying
black --check my_file.py
```

### Using isort (Import Sorter)

```bash
# Install
pip install isort

# Sort imports in a file
isort my_file.py

# Sort entire package
isort .

# Check without modifying
isort --check-only my_file.py
```

### Using autopep8 (PEP8 Auto-fixer)

```bash
# Install
pip install autopep8

# Fix a file in place
autopep8 --in-place --aggressive --aggressive my_file.py

# Fix entire package
autopep8 --in-place --recursive --aggressive --aggressive .
```

### Pre-commit Configuration

Create `.pre-commit-config.yaml` in repository root:

```yaml
repos:
  - repo: https://github.com/psf/black
    rev: 23.12.1
    hooks:
      - id: black
        language_version: python3

  - repo: https://github.com/pycqa/isort
    rev: 5.13.2
    hooks:
      - id: isort
        args: ["--profile", "black"]

  - repo: https://github.com/pycqa/flake8
    rev: 7.0.0
    hooks:
      - id: flake8
        args: ["--max-line-length=99"]
```

Install and use:

```bash
pip install pre-commit
pre-commit install
pre-commit run --all-files
```

---

## Quick Reference: Complete Example

```python
# Copyright 2025 Orangewood Labs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Flow executor module for robot control."""

import json
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from robogpt_node_stack.block_programming.node_flow.flow_parser import (
    JsonFlowParser
)


class FlowExecutor:
    """Execute flow programs from JSON definitions."""

    def __init__(self, node_instance=None):
        """
        Initialize the flow executor.

        Args:
            node_instance: Optional ROS2 node instance
        """
        self.node = node_instance
        self.parser = JsonFlowParser()

    def execute_from_json(self, json_file_path):
        """
        Parse and execute flow from JSON file.

        Args:
            json_file_path: Path to the JSON flow definition

        Returns:
            Execution result dictionary

        Raises:
            FileNotFoundError: If JSON file doesn't exist
            ValueError: If JSON format is invalid
        """
        if not self._validate_path(json_file_path):
            raise FileNotFoundError(f"File not found: {json_file_path}")

        flow_data = self.parser.parse(json_file_path)
        return self._execute_flow(flow_data)

    def _validate_path(self, path):
        """Validate file path exists."""
        import os
        return os.path.exists(path)

    def _execute_flow(self, flow_data):
        """Execute the parsed flow data."""
        results = []
        for step in flow_data:
            result = self._execute_step(step)
            results.append(result)
        return results

    def _execute_step(self, step):
        """Execute a single step in the flow."""
        # Implementation
        pass


def main():
    """Run the flow executor node."""
    rclpy.init()
    executor = FlowExecutor()
    # Implementation
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Testing Your Code

Run tests to check compliance:

```bash
# Build with testing enabled
colcon build --packages-select block_programming

# Run all tests
colcon test --packages-select block_programming

# View detailed results
colcon test-result --verbose

# Run specific test
colcon test --packages-select block_programming --pytest-args -k test_flake8
```

---

## Additional Resources

- [PEP 8 – Style Guide for Python Code](https://peps.python.org/pep-0008/)
- [PEP 257 – Docstring Conventions](https://peps.python.org/pep-0257/)
- [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
- [ROS 2 Python Style Guide](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)

---

**Last Updated:** December 2024
