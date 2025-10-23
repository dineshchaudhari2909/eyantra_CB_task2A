# CropDrop Bot - Task 2A Algorithm

## Simple Algorithm Explanation

### What the Robot Does:
The robot picks up colored boxes and sorts them into the correct drop zones based on their color.

### Step-by-Step Algorithm:

#### 1. **SEARCH FOR BOX**
- Robot moves around the pickup zone following black lines
- Uses proximity sensor to detect when a box is nearby
- When box is found, robot moves closer to it

#### 2. **PICK UP BOX**
- Robot gets close to the box (within 0.2 meters)
- Uses robotic arm to pick up the box
- Detects the color of the box using RGB color sensor

#### 3. **GO TO DECISION POINT (Node N1)**
- Robot follows the black line path to reach Node N1
- Node N1 is like a traffic intersection where robot decides where to go
- Robot knows it reached Node N1 when all line sensors detect black lines (intersection)

#### 4. **DECIDE WHERE TO DROP**
- At Node N1, robot checks the box color:
  - **Red Box** → Go to Red Drop Zone (top)
  - **Blue Box** → Go to Blue Drop Zone (middle)  
  - **Green Box** → Go to Green Drop Zone (bottom)

#### 5. **NAVIGATE TO DROP ZONE**
- Robot follows the correct path to the chosen drop zone
- Uses line following to stay on track
- Different turning directions for different zones

#### 6. **DROP THE BOX**
- Robot reaches the correct drop zone
- Uses robotic arm to drop the box
- Returns to pickup zone to find next box

### How Robot Makes Decisions:

**Line Following:**
- If center sensor sees black line → Go straight
- If left sensors see black line → Turn left
- If right sensors see black line → Turn right
- If no line detected → Turn around to find line

**Box Detection:**
- Proximity sensor measures distance to objects
- If distance < 0.5 meters → Box detected
- If distance < 0.2 meters → Close enough to pick up

**Color Detection:**
- RGB sensor reads red, green, blue values
- If red value is high → Red box
- If green value is high → Green box  
- If blue value is high → Blue box

### Robot States:
1. **SEARCHING** - Looking for boxes
2. **APPROACHING** - Moving towards detected box
3. **PICKING** - Picking up the box
4. **GOING TO NODE** - Moving to decision point
5. **AT NODE** - Making color decision
6. **GOING TO DROP** - Moving to drop zone
7. **DROPPING** - Dropping the box

### Simple Summary:
**Find Box → Pick Up → Go to Decision Point → Check Color → Go to Right Zone → Drop Box → Repeat**

This algorithm ensures the robot correctly sorts colored boxes into their matching drop zones!