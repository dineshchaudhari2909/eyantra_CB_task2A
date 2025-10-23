/*
*   ===================================================
*       CropDrop Bot (CB) Theme [eYRC 2025-26]
*   ===================================================
*
*  Task 2a: Pick and Place using PID line following
*/

#ifdef _WIN32
    #define WINVER 0x0600
    #define _WIN32_WINNT 0x0600
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    typedef SOCKET SocketType;
    #define CLOSESOCKET closesocket
    #define READ(s, buf, len) recv(s, buf, len, 0)
    #define SLEEP(ms) Sleep(ms)
    #pragma comment(lib, "Ws2_32.lib")
#else
    #include <unistd.h>
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <pthread.h>
    typedef int SocketType;
    #define CLOSESOCKET close
    #define READ(s, buf, len) read(s, buf, len)
    #define SLEEP(ms) usleep((ms)*1000)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

// ==================== SocketClient ====================
typedef struct {
    SocketType sock;
    bool running;

    float line_sensors[5];   // left_corner, left, middle, right, right_corner
    float proximity_distance;
    float color_r, color_g, color_b;

#ifdef _WIN32
    HANDLE recv_thread;
#else
    pthread_t recv_thread;
#endif
} SocketClient;

SocketClient client;

// ----------- Function Declarations -----------
bool connect_to_server(SocketClient* c, const char* ip, int port);
void disconnect(SocketClient* c);
void* receive_loop(void* arg);
void set_motor(SocketClient* c, float left, float right);
int pick_box(SocketClient* c);
int drop_box(SocketClient* c);

// ==================== Implementations ====================

// Connect to CoppeliaSim
bool connect_to_server(SocketClient* c, const char* ip, int port) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) return false;
#endif

    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0) return false;

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);

    if (connect(c->sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) return false;

    c->running = true;

#ifdef _WIN32
    c->recv_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_loop, c, 0, NULL);
#else
    pthread_create(&c->recv_thread, NULL, receive_loop, c);
#endif

    return true;
}

void disconnect(SocketClient* c) {
    c->running = false;
#ifdef _WIN32
    WaitForSingleObject(c->recv_thread, INFINITE);
#else
    pthread_join(c->recv_thread, NULL);
#endif

    if(c->sock != -1) CLOSESOCKET(c->sock);
#ifdef _WIN32
    WSACleanup();
#endif
}

// Send motor commands
void set_motor(SocketClient* c, float left, float right) {
    if(c->sock == -1) return;
    char cmd[128];
    snprintf(cmd,sizeof(cmd),"L:%.2f;R:%.2f\n", left, right);
    send(c->sock, cmd, strlen(cmd), 0);
}

// Pick / Drop
int pick_box(SocketClient* c){
    if(!c->running || c->sock == -1) return 0;
    return send(c->sock,"PICK\n",5,0) > 0;
}
int drop_box(SocketClient* c){
    if(!c->running || c->sock == -1) return 0;
    return send(c->sock,"DROP\n",5,0) > 0;
}

// Receive sensor updates
void* receive_loop(void* arg){
    SocketClient* c = (SocketClient*)arg;
    char buffer[2048];
    char line[2048];
    int pos = 0;

    while(c->running){
        int n = READ(c->sock, buffer, sizeof(buffer)-1);
        if(n>0){
            buffer[n]='\0';
            for(int i=0;i<n;i++){
                if(buffer[i]=='\n'){
                    line[pos]='\0';
                    pos=0;

                    // Parse line: S:..;P:..;C:..
                    char line_copy[2048]; strcpy(line_copy,line);
                    char* seg=line_copy;
                    char* next=NULL;
                    do {
                        next = strchr(seg,';');
                        if(next){ *next='\0'; next++; }

                        if(strncmp(seg,"S:",2)==0){
                            float* arr = c->line_sensors;
                            int idx=0;
                            char* tok = strtok(seg+2,",");
                            while(tok && idx<5){ arr[idx++]=atof(tok); tok=strtok(NULL,","); }
                        } else if(strncmp(seg,"P:",2)==0){
                            c->proximity_distance = atof(seg+2);
                        } else if(strncmp(seg,"C:",2)==0){
                            char* tok = strtok(seg+2,",");
                            c->color_r = atof(tok);
                            tok = strtok(NULL,",");
                            c->color_g = atof(tok);
                            tok = strtok(NULL,",");
                            c->color_b = atof(tok);
                        }

                        seg = next;
                    } while(seg);
                } else {
                    if(pos<2047) line[pos++]=buffer[i];
                }
            }
        }
        SLEEP(1);
    }
    return NULL;
}

// ==================== Control Loop ====================
void* control_loop(void* arg){
    SocketClient* c = (SocketClient*)arg;

    float prev_error=0, integral=0;
    enum {SEARCHING, NAVIGATING, DROPPING} state=SEARCHING;
    int drop_zone=0;

    float picked_r=0, picked_g=0, picked_b=0;

    const float Kp=1.2, Ki=0.0, Kd=0.5;
    const float base_speed=2.6;
    const float proximity_threshold=1.0;  // box detection
    const int pickup_delay=500;  // ms
    const float color_tolerance=0.1;      // for dropping

    while(c->running){
        float ir[5]; for(int i=0;i<5;i++) ir[i]=c->line_sensors[i];
        float prox = c->proximity_distance;
        float r = c->color_r, g=c->color_g, b=c->color_b;

        // PID
        float w[5]={-2,-1,0,1,2}, ws=0, sum=0;
        for(int i=0;i<5;i++){ float v=1-ir[i]; ws+=w[i]*v; sum+=v;}
        float error = sum>0?ws/sum:0;
        integral+=error;
        float derivative = error-prev_error;
        float corr = Kp*error + Ki*integral + Kd*derivative;
        prev_error=error;

        // Adjust speed if carrying box
        float current_base_speed = base_speed;
        if(state == NAVIGATING || state == DROPPING) current_base_speed += 1.6;

        float left=current_base_speed + corr;
        float right=current_base_speed - corr;
        if(left>1) left=1; if(left<0) left=0;
        if(right>1) right=1; if(right<0) right=0;

        // --- State Machine ---
        switch(state){
            case SEARCHING:
                set_motor(c,left,right);
                // Pick if object detected (proximity + color)
                if(prox < proximity_threshold && (r>0.1 || g>0.1 || b>0.1)){
                    set_motor(c,0,0);
                    SLEEP(500);
                    pick_box(c);
                    SLEEP(pickup_delay);

                    // Record picked color
                    picked_r = r; picked_g = g; picked_b = b;

                    // Determine drop zone
                    if(r>g && r>b) drop_zone=1;      // RED -> Zone 1
                    else if(b>r && b>g) drop_zone=2; // BLUE -> Zone 2
                    else drop_zone=3;                // GREEN -> Zone 3

                    printf("Picked box! RGB:(%.2f,%.2f,%.2f) -> Zone %d\n",r,g,b,drop_zone);
                    state=NAVIGATING;
                }
                break;

            case NAVIGATING: {
                // Node Detection: Using ir[1..3] to detect junction
                bool at_node = (ir[1]<0.4 && ir[2]<0.4 && ir[3]<0.4);

                if(at_node){
                    // --- LEFT TURN (GREEN box / drop_zone==3) ---
                    if(drop_zone==3){
                        printf("GREEN box detected - Turning LEFT\n");
                        while(c->running){
                            float left_speed=0.1, right_speed=0.6; // start left turn

                            // Adjust speeds based on corner -> side -> middle sensors
                            if(c->line_sensors[0]<0.5) left_speed=0.3;  // left corner sees black
                            if(c->line_sensors[1]<0.5) left_speed=0.4;  // left side sees black
                            if(c->line_sensors[2]<0.5) left_speed=0.5;  // middle sees black, done

                            set_motor(c, left_speed, right_speed);

                            // Update IR sensors
                            for(int i=0;i<5;i++) ir[i]=c->line_sensors[i];

                            // Exit turn when middle sees black
                            if(c->line_sensors[2]<0.5) break;
                            SLEEP(5);
                        }
                    }

                    // --- RIGHT TURN (RED box / drop_zone==1) ---
                    else if(drop_zone==1){
                        printf("RED box detected - Turning RIGHT\n");
                        while(c->running){
                            float left_speed=0.6, right_speed=0.1; // start right turn

                            // Adjust speeds based on corner -> side -> middle sensors
                            if(c->line_sensors[4]<0.5) right_speed=0.3;  // right corner sees black
                            if(c->line_sensors[3]<0.5) right_speed=0.4;  // right side sees black
                            if(c->line_sensors[2]<0.5) right_speed=0.5;  // middle sees black, done

                            set_motor(c, left_speed, right_speed);

                            // Update IR sensors
                            for(int i=0;i<5;i++) ir[i]=c->line_sensors[i];

                            // Exit turn when middle sees black
                            if(c->line_sensors[2]<0.5) break;
                            SLEEP(5);
                        }
                    }

                    // --- BLUE box: GO STRAIGHT ---
                    else if(drop_zone==2){
                        printf("BLUE box detected - Going STRAIGHT\n");
                        // Small delay at node to clear it
                        SLEEP(100);
                    }
                }

                // Resume normal PID line following
                set_motor(c, left, right);

                // Check if destination reached (motion sensor picks color picked)
                bool color_match = fabs(r-picked_r)<color_tolerance &&
                                   fabs(g-picked_g)<color_tolerance &&
                                   fabs(b-picked_b)<color_tolerance;

                if(color_match){
                    printf("Destination reached! Color match detected\n");
                    state=DROPPING;
                }
                break;
            }

            case DROPPING: {
                // Stop robot and drop box
                set_motor(c,0,0);
                drop_box(c);
                SLEEP(1000);
                
                printf("Dropped box at zone %d\n",drop_zone);
                state=SEARCHING;
                break;
            }
        }

        printf("State:%d | L:%.2f R:%.2f | Prox:%.2f | RGB:(%.2f,%.2f,%.2f)\n",
               state,left,right,prox,r,g,b);

        SLEEP(5);
    }

    return NULL;
}

// ==================== Main ====================
int main(){
    printf("Initializing Task2a...\n");
    if(!connect_to_server(&client,"127.0.0.1",50002)){
        printf("Failed to connect!\n");
        return -1;
    }
    printf("Connected to CoppeliaSim!\n");

#ifdef _WIN32
    HANDLE t = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)control_loop,&client,0,NULL);
#else
    pthread_t t;
    pthread_create(&t,NULL,control_loop,&client);
#endif

    printf("Monitoring sensors... Ctrl+C to exit\n");
    while(1) SLEEP(100);

    disconnect(&client);
    return 0;
}
