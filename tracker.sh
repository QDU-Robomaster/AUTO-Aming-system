#!/bin/bash

delay_time=3
while true
do
    ps -ef | grep "armor_tracker_node" | grep -v "grep"
    if ["$?" -ne 0]
    then
    sleep $delay_time
        #gnome-terminal -t "tracker" -x bash -c "cd /home/rm/tracker;source install/setup.bash;  ros2 launch rm_vision_bringup vision_bringup.launch.py;exec bash;"
        konsole -e bash -c 'cd /home/rm/tracker;source install/setup.bash;  ros2 launch rm_vision_bringup vision_bringup.launch.py'
        if [$? -eq 0]
        then
            echo "tracker started successfully!"
        else
            echo "tracker failed to start!"
        fi
    else
    echo "tracker is running!"
    fi
    sleep $delay_time
done