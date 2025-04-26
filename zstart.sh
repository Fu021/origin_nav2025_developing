gnome-terminal -- bash -c "cd ~/origin_nav2025_test;source install/setup.bash;ros2 launch bringup startup.launch.py;exec bash;"
gnome-terminal -- bash -c "sleep 5s;cd ~/origin_nav2025_test;source install/setup.bash;ros2 launch nav bringup_launch.py;exec bash;"
gnome-terminal -- bash -c "sleep 1;cd ~/FYT;source install/setup.bash;ros2 launch rm_bringup  bringup.launch.py;exec bash;"
gnome-terminal -- bash -c "sleep 5s;cd ~/origin_nav2025_test;source install/setup.bash;ros2 run dec_tree root;exec bash;"

