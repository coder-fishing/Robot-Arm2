
#!/bin/bash

echo "===== RUN UR5 PICK & PLACE ====="

# 1. Source ROS
source /opt/ros/noetic/setup.bash
source ~/ur5_ws/devel/setup.bash

# 2. Run Gazebo + UR5
echo "Starting UR5 in Gazebo..."
gnome-terminal -- bash -c "
roslaunch ur_gazebo ur5_bringup.launch;
exec bash"

sleep 6

# 3. Spawn objects
echo "Spawning objects..."
gnome-terminal -- bash -c "
roslaunch ur5_objects spawn_objects.launch;
exec bash"

sleep 3

# 4. Run control node
echo "Running control node..."
gnome-terminal -- bash -c "
roslaunch ur5_control run_pick_place.launch;
exec bash"

echo "===== ALL DONE ====="
