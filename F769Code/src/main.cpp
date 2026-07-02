#include "mbed.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// LVGL & Display Headers
#include "hal_stm_lvgl/tft/tft.h"
#include "hal_stm_lvgl/touchpad/touchpad.h"
#include "lvgl/lvgl.h"

// MQTT & Network Headers
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

// =========================================================
// F769 LVGL MQTT Remote Control UI for BB Car
// =========================================================

#define MQTT_BROKER_IP   "192.168.1.221"   
#define MQTT_BROKER_PORT 1883

const char* TOPIC_STATUS = "bbcar/status";
const char* TOPIC_CMD    = "bbcar/cmd";

// =========================================================
// Board I/O
// =========================================================
InterruptIn btn_center(BUTTON1);
DigitalOut led1(LED1);

BufferedSerial pc(USBTX, USBRX, 115200);


static Thread tick_thread(osPriorityRealtime, 2048);

// =========================================================
// MQTT globals
// =========================================================
static MQTT::Client<MQTTNetwork, Countdown>* mqttClientPtr = nullptr;
static volatile bool send_stop = false;
static bool mqtt_ready = false;


static char mqtt_payload_buf[512];

// =========================================================
// UI objects
// =========================================================
static lv_obj_t *label_title;
static lv_obj_t *label_status;
static lv_obj_t *label_ping;
static lv_obj_t *label_speed;
static lv_obj_t *label_direction;
static lv_obj_t *label_distance;
static lv_obj_t *label_barcode;
static lv_obj_t *label_manual;
static lv_obj_t *label_special;
static lv_obj_t *label_error;
static lv_obj_t *label_map[4][4];

// =========================================================
// Telemetry values from BB Car
// =========================================================
static char  g_state[32]   = "WAITING";
static char  g_barcode[32] = "NONE";

static float g_ping     = 999.0f;
static float g_speed    = 0.0f;
static float g_dist_acc = 0.0f;
static float g_dist_fb  = 0.0f;
static float g_err      = 0.0f;
static float g_steer    = 0.0f;

static int g_x       = 0;
static int g_y       = 0;
static int g_heading = 1;
static int g_manual  = 0;
static int g_cmd     = 0;
static int g_special = 0;

static volatile bool telemetry_dirty = true;

// =========================================================
// 1. LVGL tick thread
// =========================================================
void lvgl_tick_thread_fn()
{
    while (true) {
        lv_tick_inc(5);
        ThisThread::sleep_for(5ms);
    }
}

// =========================================================
// 2. Small JSON parsing helpers
// =========================================================
static bool extract_string(const char* json, const char* key, char* out, int out_size)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);

    const char* p = strstr(json, pattern);
    if (!p) return false;

    p += strlen(pattern);

    const char* q = strchr(p, '"');
    if (!q) return false;

    int len = q - p;
    if (len >= out_size) len = out_size - 1;

    memcpy(out, p, len);
    out[len] = '\0';

    return true;
}

static bool extract_float(const char* json, const char* key, float* out)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);

    const char* p = strstr(json, pattern);
    if (!p) return false;

    p += strlen(pattern);
    *out = atof(p);

    return true;
}

static bool extract_int(const char* json, const char* key, int* out)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);

    const char* p = strstr(json, pattern);
    if (!p) return false;

    p += strlen(pattern);
    *out = atoi(p);

    return true;
}

// =========================================================
// 3. Display helper functions
// =========================================================
static const char* heading_text(int h)
{
    switch (h) {
        case 0: return "NORTH";
        case 1: return "EAST";
        case 2: return "SOUTH";
        case 3: return "WEST";
        default: return "UNKNOWN";
    }
}

static const char* heading_icon(int h)
{
    switch (h) {
        case 0: return "^";
        case 1: return ">";
        case 2: return "v";
        case 3: return "<";
        default: return "?";
    }
}

static const char* cmd_text(int cmd)
{
    switch (cmd) {
        case 0: return "STOP";
        case 1: return "FORWARD";
        case 2: return "BACKWARD";
        case 3: return "LEFT";
        case 4: return "RIGHT";
        default: return "UNKNOWN";
    }
}

static void clamp_route_position()
{
    if (g_x < 0) g_x = 0;
    if (g_x > 3) g_x = 3;

    if (g_y < 0) g_y = 0;
    if (g_y > 3) g_y = 3;

    if (g_heading < 0) g_heading = 0;
    if (g_heading > 3) g_heading = 1;
}

// =========================================================
// 4. MQTT command publisher
// =========================================================
void publish_command(const char* cmd)
{
    if (mqttClientPtr == nullptr || !mqtt_ready) {
        printf("MQTT client not ready. Cannot send: %s\r\n", cmd);
        return;
    }

    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*)cmd;
    message.payloadlen = strlen(cmd) + 1;

    int rc = mqttClientPtr->publish(TOPIC_CMD, message);

    printf("Sent command: %s, rc=%d\r\n", cmd, rc);
}

// =========================================================
// 5. Hardware button emergency stop
// =========================================================
void button_pressed()
{
    send_stop = true;
}

// =========================================================
// 6. LVGL button callbacks
// =========================================================
static void btn_forward_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        publish_command("FORWARD");
    }
}

static void btn_backward_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        publish_command("BACKWARD");
    }
}

static void btn_left_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        publish_command("LEFT");
    }
}

static void btn_right_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        publish_command("RIGHT");
    }
}

static void btn_stop_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        publish_command("STOP");
    }
}

static void btn_auto_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        publish_command("AUTO");
    }
}

static void btn_reset_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        publish_command("RESET_ROUTE");
    }
}

// =========================================================
// 7. LVGL button creator
// =========================================================
static lv_obj_t* create_btn(lv_obj_t *parent,
                            const char *text,
                            int x,
                            int y,
                            int w,
                            int h,
                            lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    return btn;
}

// =========================================================
// 8. Build LVGL UI
// =========================================================
void build_ui()
{
    lv_obj_t *scr = lv_scr_act();

    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_text_color(scr, lv_color_white(), 0);

    // =========================
    // Title
    // =========================
    label_title = lv_label_create(scr);
    lv_label_set_text(label_title, "BB Car MQTT Remote");
    lv_obj_set_style_text_color(label_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_18, 0);
    lv_obj_align(label_title, LV_ALIGN_TOP_LEFT, 18, 10);

    // =========================
    // Status panel
    // =========================
    lv_obj_t *panel = lv_obj_create(scr);
    lv_obj_set_size(panel, 300, 150);
    lv_obj_set_pos(panel, 15, 45);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_radius(panel, 8, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    label_status = lv_label_create(panel);
    lv_obj_set_style_text_color(label_status, lv_color_white(), 0);
    lv_obj_set_pos(label_status, 10, 12);
    lv_label_set_text(label_status, "State: WAITING");

    label_ping = lv_label_create(panel);
    lv_obj_set_style_text_color(label_ping, lv_color_white(), 0);
    lv_obj_set_pos(label_ping, 10, 42);
    lv_label_set_text(label_ping, "LaserPING: --- cm");

  
    label_speed = lv_label_create(panel);
    lv_obj_set_style_text_color(label_speed, lv_color_white(), 0);
    lv_obj_set_pos(label_speed, 10, 72);
    lv_label_set_text(label_speed, "FB Distance: 0.00 m");


    label_distance = lv_label_create(panel);
    lv_obj_set_style_text_color(label_distance, lv_color_white(), 0);
    lv_obj_set_pos(label_distance, 10, 102);
    lv_label_set_text(label_distance, "Grid: x=0, y=0");

    label_direction = lv_label_create(panel);
    lv_obj_set_style_text_color(label_direction, lv_color_white(), 0);
    lv_obj_set_pos(label_direction, 10, 132);
    lv_label_set_text(label_direction, "Direction: EAST");

    // =========================
    // Route map panel
    // =========================
    lv_obj_t *map_panel = lv_obj_create(scr);
    lv_obj_set_size(map_panel, 300, 220);
    lv_obj_set_pos(map_panel, 15, 215);
    lv_obj_set_style_bg_color(map_panel, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(map_panel, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_width(map_panel, 1, 0);
    lv_obj_set_style_radius(map_panel, 8, 0);
    lv_obj_clear_flag(map_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *map_title = lv_label_create(map_panel);
    lv_obj_set_style_text_color(map_title, lv_color_hex(0x00FFFF), 0);
    lv_label_set_text(map_title, "4x4 Feedback Map");
    lv_obj_set_pos(map_title, 10, 8);

    lv_obj_t *map_note = lv_label_create(map_panel);
    lv_obj_set_style_text_color(map_note, lv_color_hex(0xAAAAAA), 0);
    lv_label_set_text(map_note, "Feedback360 odometry estimate");
    lv_obj_set_pos(map_note, 10, 32);

    int start_x = 35;
    int start_y = 65;
    int dx = 60;
    int dy = 35;

    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            label_map[y][x] = lv_label_create(map_panel);
            lv_obj_set_style_text_color(label_map[y][x], lv_color_white(), 0);
            lv_label_set_text(label_map[y][x], "[ ]");
            lv_obj_set_pos(label_map[y][x],
                           start_x + x * dx,
                           start_y + y * dy);
        }
    }

    // =========================
    // Control panel
    // =========================
    lv_obj_t *control_title = lv_label_create(scr);
    lv_label_set_text(control_title, "Manual Control");
    lv_obj_set_style_text_color(control_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(control_title, &lv_font_montserrat_18, 0);
    lv_obj_set_pos(control_title, 485, 35);

    create_btn(scr, "FORWARD", 505, 70, 150, 58, btn_forward_cb);

    create_btn(scr, "LEFT",    355, 150, 125, 58, btn_left_cb);
    create_btn(scr, "STOP",    505, 150, 150, 58, btn_stop_cb);
    create_btn(scr, "RIGHT",   680, 150, 100, 58, btn_right_cb);

    create_btn(scr, "BACK",    505, 230, 150, 58, btn_backward_cb);

    create_btn(scr, "AUTO",    395, 340, 150, 58, btn_auto_cb);
    create_btn(scr, "RESET",   585, 340, 160, 58, btn_reset_cb);
}
// =========================================================
// 9. Update LVGL labels
// =========================================================
void update_ui_labels()
{
    char buf[160];

    clamp_route_position();

    snprintf(buf, sizeof(buf), "State: %s", g_state);
    lv_label_set_text(label_status, buf);

    snprintf(buf, sizeof(buf), "LaserPING: %.0f cm", g_ping);
    lv_label_set_text(label_ping, buf);

    snprintf(buf, sizeof(buf), "FB Distance: %.2f m", g_dist_fb);
    lv_label_set_text(label_speed, buf);

    snprintf(buf, sizeof(buf), "Grid: x=%d, y=%d", g_x, g_y);
    lv_label_set_text(label_distance, buf);

    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            if (x == g_x && y == g_y) {
                snprintf(buf, sizeof(buf), "[%s]", heading_icon(g_heading));
                lv_label_set_text(label_map[y][x], buf);
                lv_obj_set_style_text_color(label_map[y][x],
                                            lv_color_hex(0x00FFFF),
                                            0);
            } else {
                lv_label_set_text(label_map[y][x], "[ ]");
                lv_obj_set_style_text_color(label_map[y][x],
                                            lv_color_white(),
                                            0);
            }
        }
    }
}

// =========================================================
// 10. MQTT status callback
// =========================================================

void statusArrived(MQTT::MessageData& md)
{
    MQTT::Message &message = md.message;

    int len = message.payloadlen;

    if (len > 511) {
        len = 511;
    }

    memcpy(mqtt_payload_buf, message.payload, len);
    mqtt_payload_buf[len] = '\0';

    extract_string(mqtt_payload_buf, "state", g_state, sizeof(g_state));
    extract_string(mqtt_payload_buf, "barcode", g_barcode, sizeof(g_barcode));

    extract_float(mqtt_payload_buf, "ping", &g_ping);
    extract_float(mqtt_payload_buf, "speed", &g_speed);
    extract_float(mqtt_payload_buf, "dist_acc", &g_dist_acc);
    extract_float(mqtt_payload_buf, "dist_fb", &g_dist_fb);
    extract_float(mqtt_payload_buf, "err", &g_err);
    extract_float(mqtt_payload_buf, "steer", &g_steer);

    extract_int(mqtt_payload_buf, "x", &g_x);
    extract_int(mqtt_payload_buf, "y", &g_y);
    extract_int(mqtt_payload_buf, "heading", &g_heading);
    extract_int(mqtt_payload_buf, "manual", &g_manual);
    extract_int(mqtt_payload_buf, "cmd", &g_cmd);
    extract_int(mqtt_payload_buf, "special", &g_special);

    clamp_route_position();

    telemetry_dirty = true;
}

// =========================================================
// 11. Serial keyboard fallback
// =========================================================
void handle_serial_input()
{
    char c;

    while (pc.readable()) {
        if (pc.read(&c, 1) == 1) {
            if      (c == 'w' || c == 'W') publish_command("FORWARD");
            else if (c == 's' || c == 'S') publish_command("BACKWARD");
            else if (c == 'a' || c == 'A') publish_command("LEFT");
            else if (c == 'd' || c == 'D') publish_command("RIGHT");
            else if (c == 'x' || c == 'X') publish_command("STOP");
            else if (c == 'r' || c == 'R') publish_command("AUTO");
            else if (c == 'z' || c == 'Z') publish_command("RESET_ROUTE");
        }
    }
}

// =========================================================
// 12. Main
// =========================================================
int main()
{
    printf("\r\n=== F769 LVGL BB Car Remote UI ===\r\n");

    pc.set_blocking(false);
    btn_center.rise(button_pressed);
    tick_thread.start(lvgl_tick_thread_fn);

    lv_init();
    tft_init();
    touchpad_init();

    build_ui();
    update_ui_labels();

    NetworkInterface *net = NetworkInterface::get_default_instance();

    if (!net) {
        printf("ERROR: No network interface found!\r\n");
        lv_label_set_text(label_status, "State: No network interface!");

        while (true) {
            lv_timer_handler();
            ThisThread::sleep_for(10ms);
        }
    }

    printf("Connecting to LAN...\r\n");
    lv_label_set_text(label_status, "State: Connecting to LAN...");

    int ret = net->connect();

    if (ret != 0) {
        printf("LAN connection failed! Error: %d\r\n", ret);
        lv_label_set_text(label_status, "State: LAN connection failed!");

        while (true) {
            lv_timer_handler();
            ThisThread::sleep_for(10ms);
        }
    }

    printf("LAN Connected!\r\n");

    SocketAddress a;
    net->get_ip_address(&a);

    printf("F769 IP Address: %s\r\n",
           a.get_ip_address() ? a.get_ip_address() : "None");

    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    mqttClientPtr = &client;

    printf("Connecting to Broker at %s:%d...\r\n",
           MQTT_BROKER_IP,
           MQTT_BROKER_PORT);

    lv_label_set_text(label_status, "State: Connecting to MQTT broker...");

    int rc = mqttNetwork.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT);

    if (rc != 0) {
        printf("Broker TCP connection failed! EXACT ERROR CODE: %d\r\n", rc);
        lv_label_set_text(label_status, "State: MQTT TCP failed!");
        mqtt_ready = false;
    } else {
        MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
        data.MQTTVersion = 3;
        data.clientID.cstring = (char*)"F769_LVGL_Remote";

        int mqtt_rc = client.connect(data);

        if (mqtt_rc == 0) {
            printf("MQTT Connected! Subscribing...\r\n");

            int sub_rc = client.subscribe(TOPIC_STATUS, MQTT::QOS0, statusArrived);

            if (sub_rc == 0) {
                printf("Subscribed to %s\r\n", TOPIC_STATUS);
                lv_label_set_text(label_status, "State: Connected! Waiting for car...");
                mqtt_ready = true;
            } else {
                printf("MQTT subscribe failed! Code: %d\r\n", sub_rc);
                lv_label_set_text(label_status, "State: MQTT subscribe failed!");
                mqtt_ready = false;
            }
        } else {
            printf("MQTT handshake failed! Code: %d\r\n", mqtt_rc);
            lv_label_set_text(label_status, "State: MQTT handshake failed!");
            mqtt_ready = false;
        }
    }

    Timer ui_timer;
    ui_timer.start();

    uint32_t last_ui_ms = 0;
    uint32_t last_yield_ms = 0;

    while (true) {
        uint32_t now = chrono::duration_cast<chrono::milliseconds>(
            ui_timer.elapsed_time()
        ).count();

        lv_timer_handler();
        if (mqtt_ready && now - last_yield_ms >= 20) {
            int yrc = client.yield(5);

            if (yrc != 0) {
                printf("MQTT yield failed: %d\r\n", yrc);
                mqtt_ready = false;
                lv_label_set_text(label_status, "State: MQTT disconnected!");
            }

            last_yield_ms = now;
        }

        handle_serial_input();

        if (send_stop) {
            send_stop = false;
            publish_command("STOP");
        }
        if (telemetry_dirty || now - last_ui_ms > 500) {
            telemetry_dirty = false;
            update_ui_labels();
            last_ui_ms = now;
        }

        ThisThread::sleep_for(10ms);
    }
}