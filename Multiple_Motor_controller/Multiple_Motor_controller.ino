#include <Arduino.h>
#include "driver/twai.h"

// ---------------- CONFIG ----------------
const uint32_t CAN_BAUD = 1000000;

// Motor configuratie (hier bepaal je het aantal motoren)
const uint8_t MOTOR_IDS[] = {1, 2};
const uint8_t MOTOR_COUNT = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

// Extended CAN IDs
enum ControlMode {
  SmartVelocity_Set = 0x20504C0,
  Heartbeat_Base    = 0x2052C80,
  Status_1          = 0x2051840
};

// ESP32 CAN pins
const int CAN_TX_PIN = 5;
const int CAN_RX_PIN = 4;

// Heartbeat interval
const uint32_t HEARTBEAT_INTERVAL_MS = 20;

// ---------------- VARIABLES ----------------
float motorSetpointRPM[MOTOR_COUNT] = {0};
float actualVelocityRPM[MOTOR_COUNT] = {0};

uint32_t lastHeartbeat = 0;

// ---------------- TWAI INIT ----------------
void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        GPIO_NUM_5,
        GPIO_NUM_4,
        TWAI_MODE_NORMAL
    );

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        while (true) delay(100);
    }

    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        while (true) delay(100);
    }

    Serial.println("ESP32 TWAI CAN initialized");
}

// ---------------- SERIAL PARSER ----------------
void handleSerialInput() {
    if (!Serial.available())
        return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
        return;

    char buffer[128];
    line.toCharArray(buffer, sizeof(buffer));

    char *token = strtok(buffer, " ");
    uint8_t index = 0;

    while (token != nullptr && index < MOTOR_COUNT) {
        motorSetpointRPM[index] = atof(token);
        index++;
        token = strtok(nullptr, " ");
    }

    if (index != MOTOR_COUNT) {
        Serial.println("Fout: aantal snelheden komt niet overeen met aantal motoren");
        return;
    }

    Serial.print("Nieuwe setpoints: ");
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Serial.print(motorSetpointRPM[i]);
        Serial.print(" ");
    }
    Serial.println();
}

// ---------------- CAN SEND ----------------
void sendHeartbeat() {
    twai_message_t msg = {};
    msg.identifier = Heartbeat_Base;
    msg.extd = true;
    msg.data_length_code = 8;
    memset(msg.data, 0xFF, 8);
    twai_transmit(&msg, pdMS_TO_TICKS(5));
}

void sendSmartVelocityToAll() {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        twai_message_t msg = {};
        msg.identifier = SmartVelocity_Set + MOTOR_IDS[i];
        msg.extd = true;
        msg.data_length_code = 8;

        memcpy(msg.data, &motorSetpointRPM[i], 4);
        memset(msg.data + 4, 0, 4);

        twai_transmit(&msg, pdMS_TO_TICKS(5));
    }
}

// ---------------- STATUS HANDLING ----------------
void handleStatusFrame(const twai_message_t &msg) {
    if (!msg.extd || msg.data_length_code < 4)
        return;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (msg.identifier == (Status_1 + MOTOR_IDS[i])) {
            memcpy(&actualVelocityRPM[i], msg.data, 4);

            Serial.print("Motor ");
            Serial.print(MOTOR_IDS[i]);
            Serial.print(" RPM: ");
            Serial.println(actualVelocityRPM[i]);
        }
    }
}

// ---------------- SETUP ----------------
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("ESP32 Multi-Motor Serial Velocity Control");
    Serial.println("Voer snelheden in gescheiden door spaties");

    initCAN();
}

// ---------------- LOOP ----------------
void loop() {
    uint32_t now = millis();

    handleSerialInput();

    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        sendHeartbeat();
    }

    sendSmartVelocityToAll();

    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        handleStatusFrame(rx_msg);
    }
}
