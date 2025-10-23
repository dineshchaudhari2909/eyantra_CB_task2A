# Task 2A Implementation Summary

## Overview
This implementation provides a complete PID-based control system for the CropDrop Bot to pick and place colored boxes. The robot uses a state machine approach to handle different phases of the task, specifically designed based on the task environment shown in the provided images.

## Task Environment Analysis (Based on Images)

### **Figure 1: Sensor Placement**
- **Robot**: Green mobile robot with wheels and robotic arm with gripper
- **Key Sensors**: 
  - **Sensor 1 (Magenta sphere)**: Color/proximity sensor in front of robot
  - **Sensor 2 (Blue block)**: Additional sensor on front-left side
- **Pickup Zone**: Blue wireframe rectangular area where boxes spawn randomly
- **Drop Zones**: 5 distinct drop zones with green bases and blue pillars
- **Navigation**: Black strips/tracks for robot movement

### **Figure 2: Task2a Scene Layout**
- **Pickup Zone**: Grey rectangular area on the left
- **Path System**: Black line path connecting pickup zone to Node N1
- **Node N1**: Central decision point (the "1st Node" mentioned in task)
- **Drop Zones**: Three color-coded zones:
  - **Red Drop Zone** (top)
  - **Blue Drop Zone** (middle) 
  - **Green Drop Zone** (bottom)
- **Boxes**: Red, blue, and green colored boxes to be sorted

### **Task Flow (Based on Images)**
1. **Pick Box**: Robot picks up box from pickup zone
2. **Navigate to Node N1**: Follow black line path to central decision point
3. **Color Detection**: At Node N1, detect box color using RGB sensor
4. **Route Decision**: Based on color, navigate to appropriate drop zone
5. **Drop Box**: Place box in correct color-matched drop zone

## Key Features

### 1. State Machine Architecture (Updated Based on Images)
- **STATE_SEARCHING**: Robot looks for boxes in pickup zone using proximity sensor
- **STATE_APPROACHING**: Robot moves towards detected box
- **STATE_PICKING**: Robot picks up the box
- **STATE_NAVIGATING_TO_NODE**: Robot navigates to Node N1 (decision point)
- **STATE_AT_NODE**: Robot is at Node N1, detecting color and deciding drop zone
- **STATE_NAVIGATING_TO_DROP**: Robot navigates to specific drop zone based on color
- **STATE_DROPPING**: Robot drops the box in correct color-matched zone

### 2. Sensor Integration
- **Line Sensors (5 IR)**: For line following and navigation
- **Proximity Sensor**: For box detection and approach
- **Color Sensor (RGB)**: For box color identification

### 3. Control Algorithms
- **Line Following**: PID-like proportional control for smooth navigation
- **Color Detection**: Threshold-based RGB color classification
- **State Management**: Robust state transitions with error handling

## Implementation Details

### Color Detection Thresholds
```c
#define RED_THRESHOLD_R    0.7
#define RED_THRESHOLD_G    0.3
#define RED_THRESHOLD_B    0.3
#define GREEN_THRESHOLD_R  0.3
#define GREEN_THRESHOLD_G  0.7
#define GREEN_THRESHOLD_B  0.3
#define BLUE_THRESHOLD_R   0.3
#define BLUE_THRESHOLD_G   0.3
#define BLUE_THRESHOLD_B   0.7
```

### Proximity Thresholds
```c
#define BOX_DETECTION_DISTANCE 0.5  // meters
#define CLOSE_DISTANCE 0.2          // meters
```

### Motor Control
```c
#define BASE_SPEED 0.3
#define TURN_SPEED 0.2
```

## How It Works (Based on Image Analysis)

1. **Search Phase**: Robot follows lines in pickup zone and searches for boxes using proximity sensor
2. **Detection**: When box is detected (< 0.5m), robot switches to approaching mode
3. **Approach**: Robot moves forward until close enough (< 0.2m) to pick up
4. **Pick Up**: Robot picks up box and detects its color using RGB sensor
5. **Navigate to Node N1**: Robot follows black line path to central decision point
6. **Color Decision**: At Node N1, robot detects color and decides which drop zone to visit
7. **Navigate to Drop Zone**: Robot navigates to specific drop zone based on color:
   - **Red Box** → Red Drop Zone (top)
   - **Blue Box** → Blue Drop Zone (middle)
   - **Green Box** → Green Drop Zone (bottom)
8. **Drop**: Robot drops box in correct color-matched zone and returns to search mode

## Compilation Instructions

### Windows:
```bash
x86_64-w64-mingw32-g++ Task2a.c -o task2a.exe -lws2_32 -static
```

### Ubuntu/Linux:
```bash
g++ -o task2a Task2a.c
```

## Running the Program

1. Open `Task2a_scene.ttt` in CoppeliaSim
2. Run the wrapper: `.\wrapper.exe` (Windows) or `./wrapper` (Linux)
3. Compile and run: `.\task2a.exe` (Windows) or `./task2a` (Linux)

## Key Improvements Made

1. **Replaced unconditional pick/drop calls** with proper state-based logic
2. **Added comprehensive state machine** for robust task execution
3. **Implemented proper line following** with enhanced error handling
4. **Added color-based navigation** to appropriate drop zones
5. **Included debugging output** for monitoring robot behavior
6. **Added error handling** for lost boxes and state transitions

## Expected Behavior

The robot will:
- Continuously search for boxes while following lines
- Detect boxes using proximity sensor
- Approach and pick up detected boxes
- Identify box color using RGB sensor
- Navigate to appropriate drop zone based on color
- Drop box in correct zone
- Return to search mode for next box

This implementation provides a solid foundation for the pick-and-place task with room for further optimization based on specific arena layout and requirements.
