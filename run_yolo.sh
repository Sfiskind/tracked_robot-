#!/bin/bash
# Activate virtual environment
source ~/ros2_venv/bin/activate

# Add virtual environment's Python packages to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(python -c "import site; print(site.getsitepackages()[0])")

# Source ROS2 workspace
source ~/ros2_main/install/setup.bash

# Run the node
ros2 run yolo_detector yolo_detector