echo "Starting colcon build"
colcon build
echo "Colcon build finished"

cmds=( "ros2 launch rm_vision_bringup vision_bringup.launch.py"
	#"ros2 launch front_mindvision_camera front_mv_launch.py"
	"ros2 launch back_mindvision_camera back_mv_launch.py"
  	"ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
     )

for cmd in "${cmds[@]}"; 
do
    echo Current CMD : "$cmd"
  	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
    sleep 0.2
done

echo "Script finished"
