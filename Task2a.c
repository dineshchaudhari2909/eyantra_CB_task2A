/*
*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  This script is intended to be an Boilerplate for 
*  Task 2a of CropDrop Bot (CB) Theme [eYRC 2025-26].
*
*  Filename:		Task2a.c
*  Created:		    19/08/2025
*  Last Modified:	12/09/2025
*  Author:		    Team members Name
*  Team ID:		    [ CB_xxxx ]
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
**********************************************

*/
// Platform-specific includes for Windows compatibility
#ifdef _WIN32
    #define WINVER 0x0600
    #define _WIN32_WINNT 0x0600
    #include <winsock2.h>
    #include <ws2tcpip.h>
#endif

#include "coppeliasim_client.h"  // Include our header
#include <sys/time.h>
#include <math.h>
#include <string.h>

// Global client instance for socket communication
SocketClient client;

// Robot state management
typedef enum {
    STATE_SEARCHING,     // Looking for a box to pick up
    STATE_APPROACHING,   // Moving towards detected box
    STATE_PICKING,       // Picking up the box
    STATE_NAVIGATING_TO_NODE,  // Moving to Node N1 (decision point)
    STATE_AT_NODE,       // At Node N1, detecting color and deciding drop zone
    STATE_NAVIGATING_TO_DROP, // Moving to specific drop zone
    STATE_DROPPING       // Dropping the box in correct zone
} RobotState;

// Color detection thresholds
#define RED_THRESHOLD_R    0.7
#define RED_THRESHOLD_G    0.3
#define RED_THRESHOLD_B    0.3
#define GREEN_THRESHOLD_R  0.3
#define GREEN_THRESHOLD_G  0.7
#define GREEN_THRESHOLD_B  0.3
#define BLUE_THRESHOLD_R   0.3
#define BLUE_THRESHOLD_G   0.3
#define BLUE_THRESHOLD_B   0.7

// Proximity thresholds
#define BOX_DETECTION_DISTANCE 0.5  // meters
#define CLOSE_DISTANCE 0.2          // meters

// Motor speeds
#define BASE_SPEED 0.3
#define TURN_SPEED 0.2

// Global state variables
RobotState current_state = STATE_SEARCHING;
bool has_box = false;
char detected_color = 'N'; // 'R', 'G', 'B', or 'N' for none
int state_counter = 0; // Counter for state transitions
bool at_node_n1 = false; // Flag to track if robot is at Node N1

// ----------------------
// Forward declarations
// ----------------------
void* control_loop(void* arg);
char detect_color(SocketClient* c);
void follow_line(SocketClient* c);
void search_for_box(SocketClient* c);
void navigate_to_drop_zone(SocketClient* c, char color);
bool detect_node_n1(SocketClient* c);
void navigate_to_specific_drop_zone(SocketClient* c, char color);

/**
 * @brief Establishes connection to the CoppeliaSim server
 */
int connect_to_server(SocketClient* c, const char* ip, int port) {
#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("WSAStartup failed\n");
        return 0;
    }
#endif
    
    // Create TCP socket
    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0) {
        printf("Socket creation failed\n");
        return 0;
    }

    // Setup server address structure
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);

    // Attempt to connect to server
    if (connect(c->sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("Connection failed\n");
        CLOSESOCKET(c->sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 0;
    }

    c->running = true;

    // Start the receive thread to handle incoming sensor data
#ifdef _WIN32
    c->recv_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_loop, c, 0, NULL);
#else
    pthread_create(&c->recv_thread, NULL, receive_loop, c);
#endif

    return 1;
}

/**
 * @brief Get current time in seconds
 */
double get_current_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

/**
 * @brief Detect color based on RGB values
 * @param c Pointer to SocketClient structure
 * @return 'R' for red, 'G' for green, 'B' for blue, 'N' for none
 */
char detect_color(SocketClient* c) {
    float r = c->color_r;
    float g = c->color_g;
    float b = c->color_b;
    
    // Check for red color
    if (r > RED_THRESHOLD_R && g < RED_THRESHOLD_G && b < RED_THRESHOLD_B) {
        return 'R';
    }
    // Check for green color
    else if (r < GREEN_THRESHOLD_R && g > GREEN_THRESHOLD_G && b < GREEN_THRESHOLD_B) {
        return 'G';
    }
    // Check for blue color
    else if (r < BLUE_THRESHOLD_R && g < BLUE_THRESHOLD_G && b > BLUE_THRESHOLD_B) {
        return 'B';
    }
    
    return 'N'; // No color detected
}

/**
 * @brief Simple line following algorithm using PID-like control
 * @param c Pointer to SocketClient structure
 */
void follow_line(SocketClient* c) {
    float ir1 = c->line_sensors[0];  // left_corner
    float ir2 = c->line_sensors[1];  // left
    float ir3 = c->line_sensors[2];  // middle
    float ir4 = c->line_sensors[3];  // right
    float ir5 = c->line_sensors[4];  // right_corner
    
    // Calculate error based on line sensor readings
    // Lower values indicate line detected
    float left_error = (ir1 + ir2) / 2.0;   // Average of left sensors
    float right_error = (ir4 + ir5) / 2.0;  // Average of right sensors
    float center_error = ir3;               // Center sensor
    
    float left_speed = BASE_SPEED;
    float right_speed = BASE_SPEED;
    
    // Enhanced line following with better error handling
    if (center_error < 0.4) {
        // Line detected in center, go straight
        left_speed = BASE_SPEED;
        right_speed = BASE_SPEED;
        printf("Following line - straight\n");
    }
    else if (left_error < 0.4 && right_error > 0.6) {
        // Line detected on left side, turn left
        left_speed = BASE_SPEED - TURN_SPEED;
        right_speed = BASE_SPEED + TURN_SPEED;
        printf("Following line - turning left\n");
    }
    else if (right_error < 0.4 && left_error > 0.6) {
        // Line detected on right side, turn right
        left_speed = BASE_SPEED + TURN_SPEED;
        right_speed = BASE_SPEED - TURN_SPEED;
        printf("Following line - turning right\n");
    }
    else if (left_error < 0.4 && right_error < 0.4) {
        // Line detected on both sides (intersection), go straight
        left_speed = BASE_SPEED;
        right_speed = BASE_SPEED;
        printf("Following line - intersection, going straight\n");
    }
    else {
        // No line detected, search by turning
        left_speed = TURN_SPEED;
        right_speed = -TURN_SPEED;
        printf("No line detected - searching\n");
    }
    
    set_motor(c, left_speed, right_speed);
}

/**
 * @brief Search for box by moving around
 * @param c Pointer to SocketClient structure
 */
void search_for_box(SocketClient* c) {
    // Simple search pattern - turn in place
    set_motor(c, TURN_SPEED, -TURN_SPEED);
}

/**
 * @brief Detect if robot has reached Node N1 (decision point)
 * @param c Pointer to SocketClient structure
 * @return true if at Node N1, false otherwise
 */
bool detect_node_n1(SocketClient* c) {
    // This is a simplified detection based on line sensor patterns
    // In a real implementation, you might use specific markers or coordinates
    
    float ir1 = c->line_sensors[0];  // left_corner
    float ir2 = c->line_sensors[1];  // left
    float ir3 = c->line_sensors[2];  // middle
    float ir4 = c->line_sensors[3];  // right
    float ir5 = c->line_sensors[4];  // right_corner
    
    // Node N1 detection: multiple lines detected (intersection)
    // All sensors detecting lines indicates an intersection
    if (ir1 < 0.4 && ir2 < 0.4 && ir3 < 0.4 && ir4 < 0.4 && ir5 < 0.4) {
        return true;
    }
    
    return false;
}

/**
 * @brief Navigate to appropriate drop zone based on color
 * @param c Pointer to SocketClient structure
 * @param color Detected color ('R', 'G', 'B')
 */
void navigate_to_drop_zone(SocketClient* c, char color) {
    // This is a simplified implementation
    // In a real scenario, you would have specific navigation logic
    // for each color zone (red, green, blue drop zones)
    
    switch (color) {
        case 'R':
            // Navigate to red drop zone
            printf("Navigating to RED drop zone...\n");
            follow_line(c);
            break;
        case 'G':
            // Navigate to green drop zone
            printf("Navigating to GREEN drop zone...\n");
            follow_line(c);
            break;
        case 'B':
            // Navigate to blue drop zone
            printf("Navigating to BLUE drop zone...\n");
            follow_line(c);
            break;
        default:
            // Unknown color, just follow line
            printf("Unknown color, following line...\n");
            follow_line(c);
            break;
    }
}

/**
 * @brief Navigate to specific drop zone with directional control
 * @param c Pointer to SocketClient structure
 * @param color Detected color ('R', 'G', 'B')
 */
void navigate_to_specific_drop_zone(SocketClient* c, char color) {
    // Based on the image, drop zones are arranged vertically:
    // Red (top), Blue (middle), Green (bottom)
    
    switch (color) {
        case 'R':
            // Navigate to red drop zone (top)
            printf("Navigating to RED drop zone (top)...\n");
            // Turn right and follow line to red zone
            set_motor(c, TURN_SPEED, -TURN_SPEED);
            break;
        case 'G':
            // Navigate to green drop zone (bottom)
            printf("Navigating to GREEN drop zone (bottom)...\n");
            // Turn left and follow line to green zone
            set_motor(c, -TURN_SPEED, TURN_SPEED);
            break;
        case 'B':
            // Navigate to blue drop zone (middle)
            printf("Navigating to BLUE drop zone (middle)...\n");
            // Go straight to blue zone
            follow_line(c);
            break;
        default:
            // Unknown color, just follow line
            printf("Unknown color, following line...\n");
            follow_line(c);
            break;
    }
}


/**
 * @brief Main control loop thread for robot behavior
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    
    printf("Starting robot control loop...\n");
    printf("Current state: %d\n", current_state);
    
    while (c->running) {
        // Read sensor values
        float proximity = c->proximity_distance;
        char detected_color_val = detect_color(c);
        bool at_node = detect_node_n1(c);
        
        // Print sensor readings for debugging
        printf("State: %d, Proximity: %.3f, Color: %c, Has Box: %s, At Node: %s\n", 
               current_state, proximity, detected_color_val, has_box ? "Yes" : "No", at_node ? "Yes" : "No");
        
        // State machine logic based on task flow from images
        switch (current_state) {
            case STATE_SEARCHING:
                // Look for a box to pick up in pickup zone
                if (proximity < BOX_DETECTION_DISTANCE && proximity > 0.1) {
                    // Box detected, move to approaching state
                    current_state = STATE_APPROACHING;
                    printf("Box detected! Switching to APPROACHING state.\n");
                } else {
                    // No box detected, search by following line
                    follow_line(c);
                }
                break;
                
            case STATE_APPROACHING:
                // Move towards the detected box
                if (proximity < CLOSE_DISTANCE) {
                    // Close enough to pick up
                    current_state = STATE_PICKING;
                    printf("Close to box! Switching to PICKING state.\n");
                } else if (proximity > BOX_DETECTION_DISTANCE) {
                    // Box moved away or disappeared
                    current_state = STATE_SEARCHING;
                    printf("Box lost! Switching back to SEARCHING state.\n");
                } else {
                    // Move forward towards box
                    set_motor(c, BASE_SPEED, BASE_SPEED);
                }
                break;
                
            case STATE_PICKING:
                // Pick up the box
                printf("Attempting to pick up box...\n");
                if (pick_box(c)) {
                    has_box = true;
                    detected_color = detected_color_val;
                    current_state = STATE_NAVIGATING_TO_NODE;
                    printf("Box picked up! Color: %c. Switching to NAVIGATING_TO_NODE state.\n", detected_color);
                } else {
                    // Retry picking
                    SLEEP(100); // Wait a bit before retry
                }
                break;
                
            case STATE_NAVIGATING_TO_NODE:
                // Navigate to Node N1 (decision point)
                if (!has_box) {
                    // Box was dropped somehow, go back to searching
                    current_state = STATE_SEARCHING;
                    printf("Box lost during navigation! Switching to SEARCHING state.\n");
                } else if (at_node) {
                    // Reached Node N1, switch to decision state
                    current_state = STATE_AT_NODE;
                    at_node_n1 = true;
                    printf("Reached Node N1! Switching to AT_NODE state.\n");
                } else {
                    // Follow line to Node N1
                    follow_line(c);
                }
                break;
                
            case STATE_AT_NODE:
                // At Node N1, detect color and decide drop zone
                if (!has_box) {
                    // Box was dropped somehow, go back to searching
                    current_state = STATE_SEARCHING;
                    at_node_n1 = false;
                    printf("Box lost at node! Switching to SEARCHING state.\n");
                } else {
                    // Detect color and decide which drop zone to go to
                    if (detected_color != 'N') {
                        current_state = STATE_NAVIGATING_TO_DROP;
                        at_node_n1 = false;
                        printf("Color detected: %c. Switching to NAVIGATING_TO_DROP state.\n", detected_color);
                    } else {
                        // Wait for color detection
                        printf("Waiting for color detection at Node N1...\n");
                        SLEEP(50);
                    }
                }
                break;
                
            case STATE_NAVIGATING_TO_DROP:
                // Navigate to specific drop zone based on color
                if (!has_box) {
                    // Box was dropped somehow, go back to searching
                    current_state = STATE_SEARCHING;
                    printf("Box lost during drop navigation! Switching to SEARCHING state.\n");
                } else {
                    // Navigate to specific drop zone
                    navigate_to_specific_drop_zone(c, detected_color);
                    
                    // After some time, try to drop
                    state_counter++;
                    if (state_counter > 50) { // After some time, try to drop
                        current_state = STATE_DROPPING;
                        state_counter = 0;
                        printf("Reached drop zone. Switching to DROPPING state.\n");
                    }
                }
                break;
                
            case STATE_DROPPING:
                // Drop the box in correct zone
                printf("Attempting to drop box in %c zone...\n", detected_color);
                if (drop_box(c)) {
                    has_box = false;
                    detected_color = 'N';
                    current_state = STATE_SEARCHING;
                    printf("Box dropped! Switching back to SEARCHING state.\n");
                } else {
                    // Retry dropping
                    SLEEP(100); // Wait a bit before retry
                }
                break;
                
            default:
                // Unknown state, reset to searching
                current_state = STATE_SEARCHING;
                printf("Unknown state detected! Resetting to SEARCHING state.\n");
                break;
        }

        SLEEP(50);  // Wait 50ms before next iteration
    }
    return NULL;
}

/**
 * @brief Main function - Entry point of the program
 */
int main() {
    // Attempt to connect to CoppeliaSim server
    if (!connect_to_server(&client, "127.0.0.1", 50002)) {
        printf("Failed to connect to CoppeliaSim server. Make sure:\n");
        printf("1. CoppeliaSim is running\n");
        printf("2. The simulation scene is loaded\n");
        printf("3. The ZMQ remote API is enabled on port 50002\n");
        return -1;
    }
    
    printf("Successfully connected to CoppeliaSim server!\n");
    printf("Starting control thread...\n");
    
    // Start the control thread for robot behavior
#ifdef _WIN32
    HANDLE control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, control_loop, &client);
#endif

    // Main loop: Display sensor data continuously
    printf("Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1) {
        SLEEP(100);  // Update display every 100ms
    }

    // Cleanup
    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}





















































