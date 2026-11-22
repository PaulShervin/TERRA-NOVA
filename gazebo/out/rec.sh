source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
ros2 launch mining_sim_gz sim.launch.py world:=cave headless:=true rviz:=false explore:=false > /tmp/launch.log 2>&1 &
sleep 30
python3 /out/record_run.py /out/rec_straight2.npz 34 0.5 0.0
pkill -INT -f "ros2 launch"; sleep 4; pkill -f "gz sim"; true
