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

Follow these codes in shell to activate necessary nodes.

```powershell
ros2 run hand_gesture recognition_node
```

- Sign Language Recognition Node (COM)

```powershell
ros2 run hand_gesture command_node
```

- Command Parsing Node (COM)

```powershell
ros2 run do_actions action_clients
```

- Action Clients Group Node (turtlebot)

```powershell
ros2 run do_actions action_servers
```

- Action Servers Group Node (turtlebot)

```powershell
ros2 run do_actions servo_node
```

- Servo Motor Control Node (turtlebot)

```powershell
ros2 run video_stream_get streamer_node
```

- Camera Video Streaming Node (turtlebot)

```powershell
ros2 run video_stream_get collector_node
```

- Video Image Crawling Node (COM)
