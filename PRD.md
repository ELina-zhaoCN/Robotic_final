# Product Requirements Document (PRD)
## Restaurant Waiter Robot

### 1. Project Summary
Building a ROS 2 food delivery robot that drives itself from the kitchen to a customer's table. It uses a camera system to spot people and avoid crashes in a crowded restaurant.

### 2. Target Users

- **Waitstaff:** They use the computer dashboard to send the robot to a specific table and watch its status.
- **Customers:** They get their food from the robot at their table.

### 3. Key Features

- **Autonomous Movement:** The robot plans its route and drives to specific table coordinates using a digital map.
- **Human Detection:** It uses the YOLO package and depth cameras to scan the area ahead. If it spots a person in its path, it slows down or stops to avoid a crash.
- **Central Dashboard:** A visual interface to pick a target table, watch the robot's live position on a 2D map, and check the estimated arrival time.
- **Emergency Stop:** A quick cancel button in the GUI to clear all movement commands if something goes wrong.

### 4. Hardware Needs

- The base robot.
- An Oak-D camera for computer vision and 3D depth data.
- A main computer module to run the ROS 2 system.
- A top tray structure. Since you are using a Turtlebot3 with Dynamixel servos and PID control for your final project, you could add a self-leveling tray to keep the food stable while the robot turns.

### 5. Software Needs

- **Framework:** ROS 2.
- **Object Recognition:** `yolo_ros` to detect the 'person' class.
- **Routing:** The standard Nav2 package for point-to-point movement.
- **Interface:** A Python app using CustomTkinter.

### 6. Success Metrics

- The robot reaches the target table within a 20-centimeter accuracy range.
- The GUI reacts to commands in under two seconds and accurately shows the position on the map.
- The camera system successfully spots a person from two meters away and triggers a stop or avoid response.
