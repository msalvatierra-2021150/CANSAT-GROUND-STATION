#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cJSON.h" 

// --- RadioLib Includes ---
#include <RadioLib.h>
#include "EspHal.h"

static const char *TAG = "RX_STATION";

// ===================== LORA PINS =====================
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_CS    5
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_DIO1  33

// ===================== HELPER: PARSE & PRINT =====================
void parse_and_print(const char *json_string) {
    // 1. Parse string to JSON Object
    cJSON *root = cJSON_Parse(json_string);
    
    if (root == NULL) {
        ESP_LOGE(TAG, "JSON Parse Error. Received: %s", json_string);
        return;
    }

    // 2. Extract Data (Safety check: Use 0.0 if key is missing)
    // We use cJSON_GetObjectItem which returns NULL if key not found
    cJSON *item;

    // --- IMU (Accel) ---
    float ax = (item = cJSON_GetObjectItem(root, "ax")) ? item->valuedouble : 0.0;
    float ay = (item = cJSON_GetObjectItem(root, "ay")) ? item->valuedouble : 0.0;
    float az = (item = cJSON_GetObjectItem(root, "az")) ? item->valuedouble : 0.0;

    // --- IMU (Gyro) ---
    float gx = (item = cJSON_GetObjectItem(root, "gx")) ? item->valuedouble : 0.0;
    float gy = (item = cJSON_GetObjectItem(root, "gy")) ? item->valuedouble : 0.0;
    float gz = (item = cJSON_GetObjectItem(root, "gz")) ? item->valuedouble : 0.0;

    // --- Environmental ---
    float press = (item = cJSON_GetObjectItem(root, "press")) ? item->valuedouble : 0.0;
    float alt   = (item = cJSON_GetObjectItem(root, "alt"))   ? item->valuedouble : 0.0;
    int   co2   = (item = cJSON_GetObjectItem(root, "co2"))   ? item->valueint    : 0;

    // --- GPS ---
    float vx = (item = cJSON_GetObjectItem(root, "vx")) ? item->valuedouble : 0.0;
    float vy = (item = cJSON_GetObjectItem(root, "vy")) ? item->valuedouble : 0.0;

    // 3. Print Clean Table
    
    printf("================ CANSAT TELEMETRY ================\n");
    printf("   [ACCEL]  X: %6.2f | Y: %6.2f | Z: %6.2f (mg)\n", ax, ay, az);
    printf("   [GYRO]   X: %6.2f | Y: %6.2f | Z: %6.2f (dps)\n", gx, gy, gz);
    printf("   -----------------------------------------------\n");
    printf("   [ENV]    Alt:   %6.2f m    | Press: %6.2f hPa\n", alt, press);
    printf("            CO2:   %d ppm\n", co2);
    printf("   -----------------------------------------------\n");
    printf("   [GPS]    Vel N: %6.2f m/s  | Vel E: %6.2f m/s\n", vx, vy);
    printf("==================================================\n\n");

    // 4. Free Memory (CRITICAL in C++)
    cJSON_Delete(root);
}

// ===================== RX TASK =====================
void lora_rx_task(void *arg) {
    ESP_LOGI(TAG, "Initializing LoRa Receiver...");

    // 1. Setup HAL and Radio
    EspHal* hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);
    SX1276* radio = new SX1276(new Module(hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1));

    // 2. Start Radio
    int state = radio->begin();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio init failed! Code: %d", state);
        while(1) vTaskDelay(100); // Stop here if hardware fails
    }
    ESP_LOGI(TAG, "Radio Listening...");

    // 3. Receive Loop
    uint8_t rx_buffer[256];z
    while (1) {
        // Try to receive a packet
        // This is a blocking call unless you use interrupts, but for a simple RX station, blocking is fine.
        state = radio->receive(rx_buffer, sizeof(rx_buffer) - 1);

        if (state == RADIOLIB_ERR_NONE) {
            // Null-terminate the received data to make it a valid C-string
            size_t len = radio->getPacketLength();
            rx_buffer[len] = '\0'; 

            ESP_LOGI(TAG, "RSSI: %.2f dBm | SNR: %.2f dB", radio->getRSSI(), radio->getSNR());
            
            // Pass the string to our parser
            parse_and_print((char*)rx_buffer);

        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // No packet received within timeout, just loop back
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            ESP_LOGW(TAG, "CRC Error! Packet corrupted.");
        } else {
            ESP_LOGE(TAG, "RX Error code: %d", state);
        }
        
        // Small delay to prevent Watchdog trigger if loop is too tight
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===================== MAIN =====================
extern "C" void app_main(void) {
    // Increase stack size for JSON parsing
    xTaskCreate(lora_rx_task, "lora_rx_task", 4096 * 2, NULL, 5, NULL);
}