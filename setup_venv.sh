#!/bin/bash
# setup_venv.sh

# Активация виртуального окружения
source ~/go2_ppo_ws/ppo_env/bin/activate

# Настройка PYTHONPATH для ROS2
export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:~/go2_ppo_ws/install/lib/python3.10/site-packages
export PYTHONPATH=$VIRTUAL_ENV/lib/python3.10/site-packages:$PYTHONPATH

# Настройка LD_LIBRARY_PATH для onnxruntime
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/go2_ppo_ws/ppo_env/lib

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace (если уже собран)
if [ -f ~/go2_ppo_ws/install/setup.bash ]; then
    source ~/go2_ppo_ws/install/setup.bash
fi

echo "Virtual environment activated and ROS2 environment configured"