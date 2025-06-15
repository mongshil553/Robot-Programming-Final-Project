# Robot-Programming-Final-Project

### Disclaimer

Check report/ for report file. <br>

Check example_images/ and example_videos/ for Sample Images. <br>

Check src/hand_gesture/hand_gesture/dataSet.txt for DataSet Used. <br>

---

### Objective

Objective: Under ROS2, devise a robotic system and scenario using Turtlebot.

### Topic

Home Assistant Robot via Sign Language Recognition 

---

## Instruction

- ChatGPT API Key
    1. Obtain ChatGPT-API key from https://platform.openai.com/api-keys
    2. Save the key as *DO_NOT_SHARE_OR_SHOW.txt* on the root folder of the project

<br><br> Execute these codes in order in shell to activate necessary nodes. <br>

- On TurtleBot:
    
    ```bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # bringup
    ros2 launch turtlebot3_bringup robot.launch.py
    # image streaming node
    ros2 run video_stream_get streamer_node
    # servo node
    ros2 run do_actions servo_node
    ```
    
<br>

- On PC with same ROS_DOMAIN_ID:
    
    ```bash
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
    

<br>

- Available Commands:
    - Servo Motor Control
        
        ```bash
        # close
        ros2 service call /team/servo/set std_srvs/srv/SetBool "{data: true}"
        # open
        ros2 service call /team/servo/set std_srvs/srv/SetBool "{data: false}"
        ```
        
    
    - Execute Action ID (1: hello, 2: spin, 3: kitchen, 4: bedroom, 5: bathroom)
        
        ```bash
        ros2 service call /team/hand_gesture/force_command rp_project_interfaces/srv/ForceCommand "{cmd: 3}"
        ```
        
    
    - Force Declare Sign Language String(connects to chagpt api)
        
        ```bash
        ros2 service call /team/hand_gesture/force_string rp_project_interfaces/srv/ForceString "{mystr: 'hplllo'}"
        ```
        This means the sign language detected is 'hplllo' and hopefully the chatgpt corrects it to 'hello'
