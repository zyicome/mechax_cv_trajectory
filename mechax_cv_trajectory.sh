#!/bin/bash
### BEGIN INIT INFO
# Provides:          watchDog
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start watchDog
# Description:       start watchDog
### END INIT INFO

sec=2
cnt=0
PROC_NAME=component_conta

cd /home/mechax/zyb/shaobin_left/hope_to_prediction/mechax_cv_trajectory
echo "Starting colcon build"
colcon build
echo "Colcon build finished"

allsource="source /opt/ros/humble/setup.bash"
shellsource="source /home/mechax/.bashrc"
source="source install/setup.bash"
cmd="ros2 launch rm_vision_bringup vision_bringup.launch.py"

echo "Starting launch"
echo "mechax2024" | sudo -S sudo chmod +777 /dev/ttyUSB0
echo "mechax2024" | sudo -S sudo chmod +777 /dev/ttyUSB1
$allsource
$source
$cmd
echo "Launch finished"


sleep $sec

#Thread='ps -ef | grep $PROC_NAME | grep -v grep' //判断用到，具体用法自行百度

while [ 1 ]
do 
count=$(ps -ef | grep $PROC_NAME | grep -v grep | wc -l)
echo "Thread count: $count"
if [ $count -eq 0 ];then
	echo "Starting $PROC_NAME"
	cd ~
	echo "mechax2024" | sudo -S sudo chmod +777 /dev/ttyUSB0
	echo "mechax2024" | sudo -S sudo chmod +777 /dev/ttyUSB1
	cd /home/mechax/zyb/shaobin_left/hope_to_prediction/mechax_cv_trajectory
	$allsource
	$source
	$cmd
	echo "$PROC_NAME has started!"
		sleep $sec
else
	echo "The $PROC_NAME is still alive!"
	sleep $sec
	
fi
done
