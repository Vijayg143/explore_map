explore lite

# Step 1: Start Gazebo (Terminal 1)
cd ~/new_rov
source install/setup.bash
ros2 launch k12_description gazebo.launch.py

# Step 1: Start Gazebo (Terminal 2)
cd ~/new_rov
source install/setup.bash
ros2 launch k12_description rviz.launch.py

# Step 2: Start Gazebo (Terminal 3)
cd ~/new_rov
source install/setup.bash
ros2 launch k12_description online_async_launch.py

# Step 3: Start Gazebo (Terminal 4)
cd ~/new_rov
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py

# Step 4: Start Gazebo (Terminal 5)
cd ~/new_rov
source /opt/ros/humble/setup.bash
source ~/new_exp_lite/install/setup.bash
source ~/explore_map/install/setup.bash
ros2 launch k12_description explore.launch.py

# To save the map
ros2 run nav2_map_server map_saver_cli -f ~/explore_map/src/k12_description/maps/map

