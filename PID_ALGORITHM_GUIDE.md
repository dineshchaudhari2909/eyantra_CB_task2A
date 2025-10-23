# PID Algorithm Implementation for CropDrop Bot

## Overview
This document provides a comprehensive guide to the PID (Proportional-Integral-Derivative) algorithm implementation for the CropDrop Bot Task 2A. The PID controller is used for precise control of line following, motor speed, and positioning.

## PID Controller Structure

```c
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain  
    float kd;           // Derivative gain
    float setpoint;     // Target value
    float integral;     // Integral term accumulator
    float previous_error; // Previous error for derivative
    float output_min;   // Minimum output limit
    float output_max;   // Maximum output limit
    float dt;           // Time step (in seconds)
} PIDController;
```

## PID Algorithm Implementation

### Core PID Formula
```
Output = Kp × Error + Ki × ∫Error dt + Kd × dError/dt
```

Where:
- **Error** = Setpoint - Current Value
- **Kp** = Proportional gain (immediate response to error)
- **Ki** = Integral gain (eliminates steady-state error)
- **Kd** = Derivative gain (reduces overshoot and oscillation)

### Implementation Details

#### 1. Proportional Term
```c
float proportional = pid->kp * error;
```
- Provides immediate response to current error
- Higher Kp = faster response but more oscillation

#### 2. Integral Term
```c
pid->integral += error * pid->dt;
float integral = pid->ki * pid->integral;
```
- Accumulates error over time
- Eliminates steady-state error
- Can cause windup if not properly limited

#### 3. Derivative Term
```c
float derivative = pid->kd * (error - pid->previous_error) / pid->dt;
```
- Responds to rate of change of error
- Reduces overshoot and oscillation
- Sensitive to noise

#### 4. Anti-Windup Protection
```c
if (output > pid->output_max) {
    output = pid->output_max;
    pid->integral -= error * pid->dt;  // Prevent windup
}
```

## Three PID Controllers Implemented

### 1. Line Following PID Controller
**Purpose**: Smooth line following with minimal oscillation

**Parameters**:
- Kp = 0.8 (Proportional gain)
- Ki = 0.0 (No integral - prevents windup on line following)
- Kd = 0.2 (Derivative gain for stability)
- Output limits: -0.5 to +0.5

**Algorithm**:
```c
// Calculate weighted line position (-1 to +1)
float line_position = calculate_weighted_position(sensors);
float correction = pid_compute(&line_following_pid, line_position);

// Apply correction to motors
float left_speed = BASE_SPEED - correction;
float right_speed = BASE_SPEED + correction;
```

### 2. Motor Speed PID Controller
**Purpose**: Maintain consistent motor speed

**Parameters**:
- Kp = 1.0 (Proportional gain)
- Ki = 0.1 (Small integral for steady-state accuracy)
- Kd = 0.05 (Small derivative for stability)
- Output limits: -1.0 to +1.0

**Algorithm**:
```c
pid_set_setpoint(&motor_speed_pid, target_speed);
float speed_correction = pid_compute(&motor_speed_pid, current_speed);
float motor_speed = target_speed + speed_correction;
```

### 3. Position PID Controller
**Purpose**: Precise positioning for box approach

**Parameters**:
- Kp = 2.0 (High proportional for quick response)
- Ki = 0.5 (Integral for accuracy)
- Kd = 0.3 (Derivative for stability)
- Output limits: -1.0 to +1.0

**Algorithm**:
```c
pid_set_setpoint(&position_pid, target_distance);
float position_correction = pid_compute(&position_pid, current_distance);
float left_speed = BASE_SPEED + position_correction;
float right_speed = BASE_SPEED - position_correction;
```

## Tuning Guidelines

### 1. Manual Tuning Method (Ziegler-Nichols)
1. **Start with Ki = 0, Kd = 0**
2. **Increase Kp until oscillation occurs**
3. **Record critical gain (Kc) and period (Pc)**
4. **Calculate parameters**:
   - Kp = 0.6 × Kc
   - Ki = 2 × Kp / Pc
   - Kd = Kp × Pc / 8

### 2. Trial and Error Method
1. **Start with small gains**
2. **Increase Kp for faster response**
3. **Add Ki to eliminate steady-state error**
4. **Add Kd to reduce overshoot**

### 3. Recommended Starting Values
```c
// Line Following
Kp = 0.5-1.0, Ki = 0.0, Kd = 0.1-0.3

// Motor Control  
Kp = 0.8-1.2, Ki = 0.05-0.2, Kd = 0.02-0.1

// Position Control
Kp = 1.5-3.0, Ki = 0.3-0.8, Kd = 0.2-0.5
```

## Performance Optimization

### 1. Sampling Time (dt)
- **Line Following**: 50ms (20 Hz)
- **Motor Control**: 50ms (20 Hz)  
- **Position Control**: 50ms (20 Hz)

### 2. Output Limiting
```c
// Prevent integral windup
if (output > output_max) {
    output = output_max;
    integral -= error * dt;
}
```

### 3. Derivative Filtering
```c
// Simple low-pass filter for derivative
float derivative = kd * (error - previous_error) / dt;
// In noisy environments, add filtering:
// derivative = alpha * derivative + (1-alpha) * previous_derivative;
```

## Usage Examples

### Line Following
```c
// Initialize PID controller
pid_init(&line_following_pid, 0.8, 0.0, 0.2, 0.0, 0.05);
pid_set_limits(&line_following_pid, -0.5, 0.5);

// In control loop
float line_position = calculate_line_position(sensors);
float correction = pid_compute(&line_following_pid, line_position);
set_motor(c, BASE_SPEED - correction, BASE_SPEED + correction);
```

### Box Approach
```c
// Initialize position PID
pid_init(&position_pid, 2.0, 0.5, 0.3, 0.2, 0.05);
pid_set_limits(&position_pid, -1.0, 1.0);

// Approach box
pid_position_control(c, 0.2); // Target 20cm distance
```

## Troubleshooting

### Common Issues

1. **Oscillation**
   - Reduce Kp
   - Increase Kd
   - Check sampling time

2. **Slow Response**
   - Increase Kp
   - Reduce Kd
   - Check output limits

3. **Steady-State Error**
   - Increase Ki
   - Check for integral windup

4. **Overshoot**
   - Reduce Kp
   - Increase Kd
   - Check setpoint changes

### Debugging Tips

```c
// Add debug output
printf("PID Debug - Error: %.3f, P: %.3f, I: %.3f, D: %.3f, Output: %.3f\n",
       error, proportional, integral, derivative, output);
```

## Advanced Features

### 1. Adaptive PID
- Adjust gains based on error magnitude
- Different gains for different operating regions

### 2. Feedforward Control
- Predict control action based on setpoint changes
- Reduce overshoot during setpoint changes

### 3. Cascade Control
- Outer loop: position control
- Inner loop: speed control
- Better performance for complex systems

## Conclusion

The PID implementation provides robust control for:
- **Line Following**: Smooth navigation along paths
- **Motor Control**: Consistent speed maintenance  
- **Position Control**: Precise box approach

Proper tuning and understanding of PID parameters is crucial for optimal performance. Start with conservative gains and gradually increase based on system response.
