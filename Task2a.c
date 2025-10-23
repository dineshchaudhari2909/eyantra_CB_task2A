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

// ----------------------
// Forward declarations
// ----------------------
void* control_loop(void* arg);

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
 * @brief Main control loop thread for robot behavior
 */
void* control_loop(void* arg) {
    SocketClient* c = (SocketClient*)arg;
    
    while (c->running) {
        // ========================================
        // LINE FOLLOWING SENSORS (IR SENSORS)
        // ========================================
        // These sensors detect black lines on white surface
        // Values typically range from 0.0 to 1.0
        // Lower values indicate darker surface (line detected)
        // Higher values indicate lighter surface (no line)
        float ir1 = c->line_sensors[0];  // left_corner sensor
        float ir2 = c->line_sensors[1];  // left sensor
        float ir3 = c->line_sensors[2];  // middle sensor (center)
        float ir4 = c->line_sensors[3];  // right sensor
        float ir5 = c->line_sensors[4];  // right_corner sensor
        printf("Line sensor readings: [%.2f, %.2f, %.2f, %.2f, %.2f]\n", ir1, ir2, ir3, ir4, ir5);
        
        // ========================================
        // PROXIMITY SENSOR (ULTRASONIC/DISTANCE)
        // ========================================
        // Detects obstacles or objects in front of the robot
        // Value represents distance in meters
        // Smaller values indicate closer obstacles
        // Use this for obstacle avoidance or box detection
        float proximity = c->proximity_distance;
        printf("Proximity sensor distance: %.3f meters\n", proximity);
        // ========================================
        // COLOR SENSOR (RGB VALUES)
        // ========================================
        // Detects color of surface beneath the sensor
        // RGB values range from 0.0 to 1.0
        // Use these values to identify colored boxes or markers
        printf("Color RGB raw values: (%.3f, %.3f, %.3f)\n", c->color_r, c->color_g, c->color_b);

        // ========================================
        // ACTUATOR COMMANDS
        // ========================================
        // Pick and drop box operations
        // Implement your logic for when to pick/drop based on sensor readings
        
        pick_box(c);
            
        drop_box(c);

        float left_speed = 0.0, right_speed = 0.0;
        set_motor(c, left_speed, right_speed);

        SLEEP(5);  // Wait 5ms before next iteration
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





















































