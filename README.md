# Robot-Programming-Final-Project

### Disclaimer

This is a project done in Robot Programming Class.

---

### Objective

Objective: Under ROS2, devise a robotic system and scenario using Turtlebot.

### Topic

Home Assistant Robot via Sign Language Recognition 

---

## Instruction

Execute these codes in order in shell to activate necessary nodes.

- On TurtleBot:
    
    ```powershell
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # bringup
    ros2 launch turtlebot3_bringup robot.launch.py
    # image streaming node
    ros2 run video_stream_get streamer_node
    # servo node
    ros2 run do_actions servo_node
    ```
    

- On PC with same ROS_DOMAIN_ID:
    
    ```powershell
    # Crawl Images streamed from TurtleBot
    ros2 run video_stream_get collector_node 
    # yolo inference node (box recognition -> action server)
    ros2 run hand_gesture yolo_inference
    # action servers group
    ros2 run do_actions action_servers
    # action clients group
    ros2 run do_actions action_clients
    # navigation launch
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/home/rail/nuri_4rd/home_realworld/map.yaml
    
    ### For Sign Language Recognition
    # hand gesture recognition node
    ros2 run hand_gesture recognition_node
    # hand gesture command_node
    ros2 run hand_gesture command_node
    ```
    

Available Commands:

- 서보 모터

```bash
# close
ros2 service call /team/servo/set std_srvs/srv/SetBool "{data: true}"
# open
ros2 service call /team/servo/set std_srvs/srv/SetBool "{data: false}"
```

```powershell
ros2 run video_stream_get collector_node
```

- Video Image Crawling Node (COM)
