# Assignment for Module #1 : C++ from Robotics Perspective

### Project Structure

```
module_1_assignment/
├── src/
│   ├── task1.cpp        # Robot class implementation
│   ├── task2.cpp        # Sensor readings simulation
│   └── task3.cpp        # Main program using sensor library
├── include/
│   └── sensor_library.h  # Header file for sensor library
├── CMakeLists.txt
└── package.xml
└── README.md
```

### How to Run
- Source your workspace in `~/assignment_ws`
```bash
source install/setup.bash
```
- Perform the following command to build your workspace
  
```bash
Colcon build
```
- Run these commands to see the output in the conse for `task 1-3`
```bash
ros2 run module_1_assignment task1
```
└──
`task1 demonstrates different types of robots (Exploration, Industrial, Medical) and their movements.`



```bash
ros2 run module_1_assignment task2
```
└──
`task2 shows simulated temperature and distance sensor readings.`

```bash
ros2 run module_1_assignment task3
```
└──
`task3 Demonstrates the usage of the template-based sensor library with different sensor types.`





