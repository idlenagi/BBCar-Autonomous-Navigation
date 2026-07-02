#include "mbed.h"
#include "WiFiInterface.h"
#include "Pixy2/Pixy2MbedSPI.h"
#include "bbcar.h"
#include "PwmIn.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include <math.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>

// =========================================================
// BB Car hardware setup
// =========================================================
Ticker servo_ticker;
Ticker servo_feedback_ticker;

PwmIn servo0_f(D9), servo1_f(D10);
PwmOut servo0_c(D11), servo1_c(D12);

BBCar car(servo0_c, servo0_f, servo1_c, servo1_f,
          servo_ticker, servo_feedback_ticker);

Pixy2MbedSPI pixy(PD_4, PD_3, PD_1, PD_5);

// LaserPING setup
DigitalInOut pin8(D8);
parallax_laserping ping1(pin8);

DigitalOut led1(LED1);

// =========================================================
// Line-following constants
// =========================================================
static const float kCruiseSpeed = 60.0f;
static const float kIntersectionSpeed = 45.0f;
static const float kHeadingKp = 3.0f;
static const float kHeadingKd = 1.1f;
static const float kHeadingDeadband = 3.0f;
static const float kHeadingCmdLimit = 72.0f;
static const float kHeadingFilterAlpha = 0.65f;
static const float kWheelSpeedLimit = 100.0f;

static const uint32_t kBarcodeDebounceMs = 350;
static const uint32_t kPendingTurnTimeoutMs = 1000;

/// =========================================================
// LaserPING obstacle stop constants
// =========================================================
static const float kObstacleDistanceCm = 20.0f;
static const float kMinValidPingCm = 3.0f;
static const float kObstacleClearCm = 25.0f;

static const int kObstacleConfirmCount = 2;
static const int kObstacleClearConfirmCount = 5;

static const uint32_t kPingPeriodMs = 100;

// =========================================================
// MQTT setup
// =========================================================
WiFiInterface *wifi = nullptr;
NetworkInterface *net = nullptr;

MQTTNetwork *mqttNetworkPtr = nullptr;
MQTT::Client<MQTTNetwork, Countdown> *mqttClientPtr = nullptr;

const char* MQTT_HOST = "192.168.1.221";   // Change to laptop / Mosquitto IP
const int MQTT_PORT = 1883;

const char* TOPIC_STATUS = "bbcar/status";
const char* TOPIC_CMD    = "bbcar/cmd";

bool mqtt_connected = false;

volatile bool manual_mode = false;
volatile int manual_cmd = 0;
static const uint32_t kMqttYieldPeriodMs = 20;
static const uint32_t kMqttPublishPeriodMs = 1000;

uint32_t last_mqtt_yield_ms = 0;
uint32_t last_mqtt_pub_ms = 0;

// =========================================================
// Enums
// =========================================================
enum BarcodeCase {
    BARCODE_NONE = -1,
    BARCODE_LEFT = 0,
    BARCODE_STOP = 1,
    BARCODE_RIGHT = 2,
    BARCODE_STRAIGHT=15,
};

enum DriveState {
    FOLLOW_LINE,
    STOPPED,
    AVOID_OBSTACLE,
    SEARCH_LINE
};

enum AvoidPhase {
    AVOID_TURN_LEFT,
    AVOID_FORWARD,
    AVOID_TURN_RIGHT
};

// =========================================================
// Global telemetry
// =========================================================
Timer app_timer;

float g_ping_cm = 999.0f;
float g_line_error = 0.0f;
float g_steer_cmd = 0.0f;
float g_current_speed = 0.0f;
int g_last_barcode = BARCODE_NONE;


// =========================================================
// Route/map telemetry for F769
// heading: 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST
// ========================================================
int route_x = 0;
int route_y = 0;
int route_heading = 1;

// =========================================================
// Feedback360 odometry
// heading: 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST
// =========================================================
static const float kWheelDiameterM = 0.065f;   // wheel diameter, tune if needed
static const float kWheelBaseM     = 0.135f;   // distance between wheels, tune if needed
static const float kCellLengthM    = 0.25f;    // one F769 grid cell = 25 cm, tune to your track
static const float kPi             = 3.1415926f;

static const float kWheelCircumferenceM = kPi * kWheelDiameterM;

float odom_x_m = 0.0f;
float odom_y_m = 0.0f;
float odom_theta_rad = 0.0f;
float odom_total_dist_m = 0.0f;

float prev_left_angle = 0.0f;
float prev_right_angle = 0.0f;
bool odom_initialized = false;
uint32_t last_route_step_ms = 0;
static const uint32_t kRouteStepDebounceMs = 700;

static uint32_t now_ms()
{
    return chrono::duration_cast<chrono::milliseconds>(
        app_timer.elapsed_time()
    ).count();
}

// =========================================================
// Helper functions
// =========================================================
static const char *barcode_to_text(int code)
{
    switch (code) {
        case BARCODE_LEFT:
            return "Barcode 0: Turn Left";
        case BARCODE_STOP:
            return "Barcode 1: Stop";
        case BARCODE_RIGHT:
            return "Barcode 2: Turn Right";
        case BARCODE_STRAIGHT:
            return "Barcode 15: Go Straight";
        default:
            return "No Barcode";
    }
}

static const char *state_to_text(DriveState state)
{
    switch (state) {
        case FOLLOW_LINE:
            return "FOLLOW_LINE";
        case STOPPED:
            return "STOPPED";
        case AVOID_OBSTACLE:
            return "AVOID_OBSTACLE";
        case SEARCH_LINE:
            return "SEARCH_LINE";
        default:
            return "UNKNOWN";
    }
}

static int read_barcode_code(int8_t feat_res)
{
    if ((feat_res & LINE_BARCODE) &&
        pixy.line.numBarcodes > 0 &&
        pixy.line.barcodes != nullptr) {

        return static_cast<int>(pixy.line.barcodes[0].m_code);
    }

    return BARCODE_NONE;
}

static void apply_turn_command(int barcode_code)
{
    if (barcode_code == BARCODE_LEFT) {
        pixy.line.setNextTurn(-90);
        printf("Queued next turn: LEFT\r\n");
    } else if (barcode_code == BARCODE_RIGHT) {
        pixy.line.setNextTurn(90);
        printf("Queued next turn: RIGHT\r\n");
    } else if (barcode_code == BARCODE_STRAIGHT) {
        pixy.line.setNextTurn(0);
        printf("Queued next turn: STRAIGHT\r\n");
    }
}

static bool obstacle_is_valid(float d)
{
    return d >= kMinValidPingCm && d <= kObstacleDistanceCm;
}

// =========================================================
// Route/map helper
// Moves F769 map one cell per new intersection.
// =========================================================
static float read_feedback_angle_deg(PwmIn &fb)
{
    float period = fb.period();
    float pulse = fb.pulsewidth();

    if (period <= 0.0f) {
        return 0.0f;
    }

    float duty = pulse / period;

    // Approximate Parallax Feedback360 duty-to-angle conversion.
    float angle = (duty - 0.029f) * 360.0f / (0.971f - 0.029f);

    if (angle < 0.0f) angle = 0.0f;
    if (angle >= 360.0f) angle = 359.9f;

    return angle;
}

static float angle_delta_deg(float now_angle, float prev_angle)
{
    float delta = now_angle - prev_angle;

    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    return delta;
}

static void reset_odometry()
{
    odom_x_m = 0.0f;
    odom_y_m = 0.0f;
    odom_theta_rad = 0.0f;
    odom_total_dist_m = 0.0f;

    route_x = 0;
    route_y = 0;
    route_heading = 1;

    prev_left_angle = read_feedback_angle_deg(servo0_f);
    prev_right_angle = read_feedback_angle_deg(servo1_f);
    odom_initialized = true;

    printf("Odometry reset\r\n");
}

static void update_odometry()
{
    float left_angle = read_feedback_angle_deg(servo0_f);
    float right_angle = read_feedback_angle_deg(servo1_f);

    if (!odom_initialized) {
        prev_left_angle = left_angle;
        prev_right_angle = right_angle;
        odom_initialized = true;
        return;
    }

    // If the car is not supposed to move, do NOT accumulate noise.
    // But still refresh previous angles so old jitter does not pile up.
    bool car_should_move =
        (fabsf(g_current_speed) > 1.0f) ||
        (manual_mode && manual_cmd != 0);

    if (!car_should_move) {
        prev_left_angle = left_angle;
        prev_right_angle = right_angle;
        return;
    }

    float d_left_deg = angle_delta_deg(left_angle, prev_left_angle);
    float d_right_deg = angle_delta_deg(right_angle, prev_right_angle);

    prev_left_angle = left_angle;
    prev_right_angle = right_angle;

    // Stronger noise filter.
    // Ignore tiny feedback changes because Feedback360 can jitter while still.
    if (fabsf(d_left_deg) < 3.0f && fabsf(d_right_deg) < 3.0f) {
        return;
    }

    float d_left_m  =  (d_left_deg  / 360.0f) * kWheelCircumferenceM;
    float d_right_m = -(d_right_deg / 360.0f) * kWheelCircumferenceM;

    float d_center = (d_left_m + d_right_m) * 0.5f;
    float d_theta  = (d_right_m - d_left_m) / kWheelBaseM;

    // Ignore tiny distance noise.
    if (fabsf(d_center) < 0.001f && fabsf(d_theta) < 0.005f) {
        return;
    }

    odom_theta_rad += d_theta;

    odom_x_m += d_center * cosf(odom_theta_rad);
    odom_y_m += d_center * sinf(odom_theta_rad);

    odom_total_dist_m += fabsf(d_center);

    int cell_x = (int)((odom_x_m / kCellLengthM) + 0.5f);
    int cell_y = (int)((odom_y_m / kCellLengthM) + 0.5f);

    if (cell_x < 0) cell_x = 0;
    if (cell_x > 3) cell_x = 3;
    if (cell_y < 0) cell_y = 0;
    if (cell_y > 3) cell_y = 3;

    route_x = cell_x;
    route_y = cell_y;

    float deg = odom_theta_rad * 180.0f / kPi;

    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg >= 315.0f || deg < 45.0f) {
        route_heading = 1;   // EAST
    } else if (deg >= 45.0f && deg < 135.0f) {
        route_heading = 2;   // SOUTH
    } else if (deg >= 135.0f && deg < 225.0f) {
        route_heading = 3;   // WEST
    } else {
        route_heading = 0;   // NORTH
    }
}
// =========================================================
// MQTT command callback
// =========================================================
void commandArrived(MQTT::MessageData& md)
{
    MQTT::Message &message = md.message;

    char cmd[64] = {0};
    int len = message.payloadlen;

    if (len > 63) {
        len = 63;
    }

    memcpy(cmd, message.payload, len);
    cmd[len] = '\0';

    for (int i = len - 1; i >= 0; i--) {
        if (cmd[i] == '\n' || cmd[i] == '\r' || cmd[i] == ' ' || cmd[i] == '\0') {
            cmd[i] = '\0';
        } else {
            break;
        }
    }

    printf("MQTT command: %s\r\n", cmd);

    if (strncmp(cmd, "AUTO", 4) == 0) {
        manual_mode = false;
        manual_cmd = 0;
        car.stop();
        printf("AUTO mode\r\n");
    } else if (strncmp(cmd, "STOP", 4) == 0) {
        manual_mode = true;
        manual_cmd = 0;
    } else if (strncmp(cmd, "FORWARD", 7) == 0) {
        manual_mode = true;
        manual_cmd = 1;
    } else if (strncmp(cmd, "BACKWARD", 8) == 0) {
        manual_mode = true;
        manual_cmd = 2;
    } else if (strncmp(cmd, "LEFT", 4) == 0) {
        manual_mode = true;
        manual_cmd = 3;
    } else if (strncmp(cmd, "RIGHT", 5) == 0) {
        manual_mode = true;
        manual_cmd = 4;
    } else if (strncmp(cmd, "RESET_ROUTE", 11) == 0) {
        reset_odometry();
        printf("Route reset\r\n");
    }
}

// =========================================================
// Manual control from F769
// =========================================================
static void apply_manual_control()
{
    switch (manual_cmd) {
        case 0:
            car.stop();
            g_current_speed = 0.0f;
            break;

        case 1:
            car.driveLR(45.0f, 45.0f);
            g_current_speed = 45.0f;
            break;

        case 2:
            car.driveLR(-35.0f, -35.0f);
            g_current_speed = -35.0f;
            break;

        case 3:
            car.driveLR(-35.0f, 35.0f);
            g_current_speed = 0.0f;
            break;

        case 4:
            car.driveLR(35.0f, -35.0f);
            g_current_speed = 0.0f;
            break;

        default:
            car.stop();
            g_current_speed = 0.0f;
            break;
    }
}

// =========================================================
// MQTT setup
// =========================================================
bool setup_mqtt()
{
    wifi = WiFiInterface::get_default_instance();

    if (!wifi) {
        printf("ERROR: No WiFiInterface found.\r\n");
        return false;
    }

    printf("Connecting WiFi: %s\r\n", MBED_CONF_APP_WIFI_SSID);

    int ret = wifi->connect(
        MBED_CONF_APP_WIFI_SSID,
        MBED_CONF_APP_WIFI_PASSWORD,
        NSAPI_SECURITY_WPA_WPA2
    );

    if (ret != 0) {
        printf("WiFi connection error: %d\r\n", ret);
        mqtt_connected = false;
        return false;
    }

    SocketAddress ip;
    wifi->get_ip_address(&ip);

    printf("WiFi connected. IP: %s\r\n",
           ip.get_ip_address() ? ip.get_ip_address() : "None");

    net = wifi;

    static MQTTNetwork mqttNetwork(net);
    mqttNetworkPtr = &mqttNetwork;

    static MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    mqttClientPtr = &client;

    printf("Connecting MQTT broker %s:%d\r\n", MQTT_HOST, MQTT_PORT);

    int rc = mqttNetwork.connect(MQTT_HOST, MQTT_PORT);

    if (rc != 0) {
        printf("MQTT TCP connection error: %d\r\n", rc);
        mqtt_connected = false;
        return false;
    }

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = (char*)"BBCar_L4S5";

    rc = client.connect(data);

    if (rc != 0) {
        printf("MQTT client connect failed: %d\r\n", rc);
        mqtt_connected = false;
        return false;
    }

    rc = client.subscribe(TOPIC_CMD, MQTT::QOS0, commandArrived);

    if (rc != 0) {
        printf("MQTT subscribe failed: %d\r\n", rc);
        mqtt_connected = false;
        return false;
    }

    mqtt_connected = true;

    printf("MQTT connected. Subscribed to %s\r\n", TOPIC_CMD);

    return true;
}

// =========================================================
// MQTT status publish
// =========================================================
/*void publish_status(DriveState state)
{
    if (!mqtt_connected || mqttClientPtr == nullptr) {
        printf("publish_status skipped: mqtt_connected=%d ptr=%p\r\n",
               mqtt_connected ? 1 : 0,
               mqttClientPtr);
        return;
    }

    char payload[256];

    snprintf(payload, sizeof(payload),
        "{"
        "\"t\":%lu,"
        "\"state\":\"%s\","
        "\"ping\":%.1f,"
        "\"barcode\":\"%s\","
        "\"speed\":%.1f,"
        "\"err\":%.1f,"
        "\"steer\":%.1f,"
        "\"manual\":%d,"
        "\"cmd\":%d,"
        "\"x\":%d,"
        "\"y\":%d,"
        "\"heading\":%d,"
        "\"special\":0,"
        "\"dist_acc\":0.000,"
        "\"dist_fb\":0.000"
        "}",
        now_ms(),
        state_to_text(state),
        g_ping_cm,
        barcode_to_text(g_last_barcode),
        g_current_speed,
        g_line_error,
        g_steer_cmd,
        manual_mode ? 1 : 0,
        manual_cmd,
        route_x,
        route_y,
        route_heading
    );

    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)payload;
    message.payloadlen = strlen(payload);

    int rc = mqttClientPtr->publish(TOPIC_STATUS, message);

    printf("STATUS rc=%d payload=%s\r\n", rc, payload);

    if (rc != 0) {
        printf("Publish status failed rc=%d\r\n", rc);
    }
}*/

void publish_status(DriveState state)
{
    if (!mqtt_connected || mqttClientPtr == nullptr) {
        return;
    }

    char payload[180];

    int ping_i = (int)g_ping_cm;

    int n = snprintf(payload, sizeof(payload),
        "{"
        "\"state\":\"%s\","
        "\"ping\":%d,"
        "\"x\":%d,"
        "\"y\":%d,"
        "\"heading\":%d,"
        "\"dist_fb\":%.2f"
        "}",
        state_to_text(state),
        ping_i,
        route_x,
        route_y,
        route_heading,
        odom_total_dist_m
    );

    if (n < 0 || n >= (int)sizeof(payload)) {
        return;
    }

    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)payload;
    message.payloadlen = strlen(payload);

    mqttClientPtr->publish(TOPIC_STATUS, message);
}
// =========================================================
// Non-invasive MQTT maintenance
// =========================================================
/*void service_mqtt(DriveState state, uint32_t now)
{
    if (!mqtt_connected || mqttClientPtr == nullptr) {
        return;
    }

    if (now - last_mqtt_yield_ms >= kMqttYieldPeriodMs) {
        int yrc = mqttClientPtr->yield(1);

        if (yrc != 0) {
            printf("MQTT yield warning: %d\r\n", yrc);
        }

        last_mqtt_yield_ms = now;
    }

    if (now - last_mqtt_pub_ms >= kMqttPublishPeriodMs) {
        publish_status(state);
        last_mqtt_pub_ms = now;
    }
}*/
void service_mqtt(DriveState state, uint32_t now)
{
    if (!mqtt_connected || mqttClientPtr == nullptr) {
        return;
    }

    // Keep MQTT alive for remote commands.
    if (now - last_mqtt_yield_ms >= kMqttYieldPeriodMs) {
        mqttClientPtr->yield(5);
        last_mqtt_yield_ms = now;
    }

    static DriveState last_state = FOLLOW_LINE;
    static int last_ping = -1;
    static int last_x = -1;
    static int last_y = -1;
    static int last_heading = -1;
    static int last_dist_cm = -1;
    static uint32_t last_heartbeat_ms = 0;

    int ping_i = (int)g_ping_cm;
    int dist_cm = (int)(odom_total_dist_m * 100.0f);

    bool changed =
        (state != last_state) ||
        (ping_i != last_ping) ||
        (route_x != last_x) ||
        (route_y != last_y) ||
        (route_heading != last_heading) ||
        (abs(dist_cm - last_dist_cm) >= 2);

    bool heartbeat =
        (now - last_heartbeat_ms >= 3000);

    if (changed || heartbeat) {
        publish_status(state);

        last_state = state;
        last_ping = ping_i;
        last_x = route_x;
        last_y = route_y;
        last_heading = route_heading;
        last_dist_cm = dist_cm;
        last_heartbeat_ms = now;
    }
}// =========================================================
// Main
// =========================================================
int main()
{
    printf("\r\n=== Pixy2 Barcode Navigation + LaserPING + MQTT ===\r\n");

    car.stop();
    led1 = 0;
    app_timer.start();

    setup_mqtt();

    int8_t init_res = pixy.init();

    if (init_res < 0) {
        printf("Pixy2 init failed: %d\r\n", init_res);

        while (true) {
            thread_sleep_for(500);
        }
    }

    pixy.setLamp(1, 1);
    pixy.setServos(500, 1000);

    int8_t prog_res = pixy.changeProg("line");

    if (prog_res < 0) {
        printf("Failed to switch Pixy2 program to line: %d\r\n", prog_res);

        while (true) {
            thread_sleep_for(500);
        }
    }

    printf("Pixy2 ready. Frame: %u x %u\r\n",
           pixy.frameWidth,
           pixy.frameHeight);

    printf("Barcode 0=Left, 1=Stop, 2=Right, 3=Straight\r\n");

    pixy.line.setDefaultTurn(0);

    printf("Default turn set to straight\r\n");

    DriveState state = FOLLOW_LINE;

    float prev_error = 0.0f;
    float prev_heading_cmd = 0.0f;
    bool has_prev_error = false;

    int last_barcode_code = BARCODE_NONE;
    uint32_t last_barcode_seen_ms = 0;
    bool barcode_armed = true;

    int pending_turn_code = BARCODE_NONE;
    uint32_t pending_turn_set_ms = 0;
    bool prev_intersection_seen = false;

    uint32_t last_ping_ms = 0;
    int obstacle_count = 0;
    int obstacle_clear_count = 0;

    AvoidPhase avoid_phase = AVOID_TURN_LEFT;
    uint32_t avoid_start_ms = 0;
    uint32_t avoid_phase_start_ms = 0;

    while (true) {
    uint32_t now = now_ms();

    update_odometry();

    service_mqtt(state, now);

        if (manual_mode) {
            apply_manual_control();
            thread_sleep_for(20);
            continue;
        }

        if (now - last_ping_ms >= kPingPeriodMs) {
            g_ping_cm = (float)ping1;
            last_ping_ms = now;

            if (obstacle_is_valid(g_ping_cm)) {
                obstacle_count++;
            } else {
                obstacle_count = 0;
            }
        }

        if (state == FOLLOW_LINE &&
            obstacle_count >= kObstacleConfirmCount) {

            state = AVOID_OBSTACLE;
            obstacle_count = 0;
            obstacle_clear_count = 0;

            led1 = 1;
            car.stop();
            g_current_speed = 0.0f;

            has_prev_error = false;
            prev_heading_cmd = 0.0f;
            prev_error = 0.0f;

            printf("Obstacle detected. Waiting until removed.\r\n");

            publish_status(state);

            thread_sleep_for(20);
            continue;
        }

       if (state == AVOID_OBSTACLE) {
    car.stop();
    g_current_speed = 0.0f;

    bool obstacle_clear =
        (g_ping_cm > kObstacleClearCm) ||
        (g_ping_cm < kMinValidPingCm);

    if (obstacle_clear) {
        obstacle_clear_count++;
    } else {
        obstacle_clear_count = 0;
    }

    if (obstacle_clear_count >= kObstacleClearConfirmCount) {
        state = FOLLOW_LINE;
        led1 = 0;

        obstacle_count = 0;
        obstacle_clear_count = 0;

        prev_error = 0.0f;
        prev_heading_cmd = 0.0f;
        has_prev_error = false;
        prev_intersection_seen = false;

        printf("Obstacle removed. Resuming FOLLOW_LINE.\r\n");

        publish_status(state);

        thread_sleep_for(100);
    }

    thread_sleep_for(20);
    continue;
}
        /*if (state == SEARCH_LINE) {
            int8_t search_res = pixy.line.getMainFeatures(LINE_ALL_FEATURES);

            if ((search_res & LINE_VECTOR) && pixy.line.numVectors > 0) {
                state = FOLLOW_LINE;
                led1 = 0;

                car.stop();
                g_current_speed = 0.0f;

                prev_error = 0.0f;
                prev_heading_cmd = 0.0f;
                has_prev_error = false;
                prev_intersection_seen = false;

                thread_sleep_for(30);
            } else {
                car.driveLR(kSearchLeftSpeed, kSearchRightSpeed);
                g_current_speed = kSearchLeftSpeed;
            }

            thread_sleep_for(20);
            continue;
        }*/
        // =====================================================
        // Line-following logic + telemetry/map update
        // =====================================================

        int8_t feat_res = pixy.line.getMainFeatures(LINE_ALL_FEATURES);

        if (feat_res < 0) {
            printf("Pixy2 read error: %d\r\n", feat_res);
            thread_sleep_for(100);
            continue;
        }

        int barcode_code = read_barcode_code(feat_res);
        g_last_barcode = barcode_code;

        if (barcode_code != BARCODE_NONE) {
            if (barcode_code != last_barcode_code ||
                (now - last_barcode_seen_ms) > kBarcodeDebounceMs) {

                printf("Seen %s\r\n", barcode_to_text(barcode_code));

                if (barcode_code == BARCODE_STOP) {
                    state = STOPPED;
                    car.stop();
                    g_current_speed = 0.0f;
                    printf("STOPPED by barcode\r\n");
                } else if (barcode_armed &&
                          (barcode_code == BARCODE_LEFT ||
                           barcode_code == BARCODE_RIGHT ||
                           barcode_code == BARCODE_STRAIGHT)) {

                    apply_turn_command(barcode_code);
                    pending_turn_code = barcode_code;
                    pending_turn_set_ms = now;
                    barcode_armed = false;
                }
                last_barcode_code = barcode_code;
                last_barcode_seen_ms = now;
            }
        } else {
            barcode_armed = true;
        }

        if (state == STOPPED) {
            car.stop();
            g_current_speed = 0.0f;
            thread_sleep_for(50);
            continue;
        }

        if (!(feat_res & LINE_VECTOR) || pixy.line.numVectors == 0) {
            car.stop();
            g_current_speed = 0.0f;
            has_prev_error = false;
            thread_sleep_for(10);
            continue;
        }

        const Vector &v = pixy.line.vectors[0];

        const float center_x =
            (pixy.frameWidth > 0) ? (pixy.frameWidth * 0.5f) : 39.5f;

        float error = center_x - static_cast<float>(v.m_x1);

        float heading_cmd = 0.0f;

        if (has_prev_error) {
            float raw_heading_cmd =
                (kHeadingKp * error) +
                (kHeadingKd * (error - prev_error));

            heading_cmd =
                (kHeadingFilterAlpha * prev_heading_cmd) +
                ((1.0f - kHeadingFilterAlpha) * raw_heading_cmd);

            if (heading_cmd > 0.0f) {
                heading_cmd += kHeadingDeadband;
            } else if (heading_cmd < 0.0f) {
                heading_cmd -= kHeadingDeadband;
            }

            heading_cmd =
                car.clamp(heading_cmd,
                          kHeadingCmdLimit,
                          -kHeadingCmdLimit);
        }

        prev_error = error;
        prev_heading_cmd = heading_cmd;
        has_prev_error = true;

        g_line_error = error;
        g_steer_cmd = heading_cmd;

        float base_speed = kCruiseSpeed;

        if (v.m_flags & LINE_FLAG_INTERSECTION_PRESENT) {
            base_speed = kIntersectionSpeed;
        }

        float left = base_speed + heading_cmd;
        float right = base_speed - heading_cmd;

        left = car.clamp(left, kWheelSpeedLimit, -kWheelSpeedLimit);
        right = car.clamp(right, kWheelSpeedLimit, -kWheelSpeedLimit);

        car.driveLR(left, right);
        g_current_speed = base_speed;

        bool intersection_seen =
            ((v.m_flags & LINE_FLAG_INTERSECTION_PRESENT) != 0) ||
            ((feat_res & LINE_INTERSECTION) &&
             pixy.line.numIntersections > 0);

        if (intersection_seen && !prev_intersection_seen) {
    printf("Intersection detected\r\n");

    if (pending_turn_code == BARCODE_LEFT) {
        pixy.line.setNextTurn(-90);
        printf("Reinforce turn at intersection: LEFT\r\n");
    } else if (pending_turn_code == BARCODE_RIGHT) {
        pixy.line.setNextTurn(90);
        printf("Reinforce turn at intersection: RIGHT\r\n");
    } else if (pending_turn_code == BARCODE_STRAIGHT) {
        pixy.line.setNextTurn(0);
        printf("Reinforce turn at intersection: STRAIGHT\r\n");
    }

    pending_turn_code = BARCODE_NONE;
}
        if (pending_turn_code != BARCODE_NONE &&
            (now - pending_turn_set_ms) > kPendingTurnTimeoutMs) {

            pending_turn_code = BARCODE_NONE;
        }

        prev_intersection_seen = intersection_seen;

        thread_sleep_for(10);
    }
}