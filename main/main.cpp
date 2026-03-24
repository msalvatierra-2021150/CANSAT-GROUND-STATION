#include <stdio.h>
#include <string.h>
#include <math.h>  // Added for sqrt() to calculate velocity magnitude

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// --- RadioLib Includes ---
#include <RadioLib.h>
#include "EspHal.h"

static const char *TAG = "RX_STATION";

// ===================== LORA/FSK PINS (SX1276) =====================
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_CS    5
#define LORA_RST   14
#define LORA_DIO0  26   // PayloadReady
#define LORA_DIO1  33   // FIFO Level (Recommended for FSK)

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic1;   // 0xCA
    uint8_t  magic2;   // 0xFE
    uint8_t  version;  // 1
    uint8_t  count;    // wraps 0-255

    float accelX, accelY, accelZ;
    float gyroX,  gyroY,  gyroZ;
    float pressure;   // hPa
    float temp;       // °C
    float velocityX, velocityY, velocityZ;
    float altitude;   // meters
} TelemetryF32V1;
#pragma pack(pop)

// ===================== GLOBAL RADIO OBJECTS =====================
EspHal* hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);
Module* mod = new Module(hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
SX1276 radio = mod;

// Interupt flag
volatile bool receivedFlag = false;

// ISR: This runs when DIO0 goes HIGH
void IRAM_ATTR setFlag(void) {
    receivedFlag = true;
}

// ===================== RX TASK =====================
void fsk_rx_task(void *arg) {
    ESP_LOGI(TAG, "Initializing SX1276 FSK...");

    // 1) Initial Setup
    // Frequency: 915.0 MHz, Bitrate: 50.0 kbps, Dev: 25.0 kHz, BW: 100.0 kHz
    int state = radio.beginFSK(915.0, 50.0, 25.0, 100.0, 17, 40);
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "FSK init failed! Code: %d", state);
        vTaskDelete(NULL);
        return;
    }

    // 2) Configuration
    uint8_t syncWord[] = { 0x2D, 0xD4 };
    radio.setSyncWord(syncWord, 2);
    radio.setCRC(true);
    radio.fixedPacketLengthMode(sizeof(TelemetryF32V1));

    // 3) Set Interrupt Callback (Passing function pointer, no parentheses)
    radio.setDio0Action(setFlag, RISING);

    ESP_LOGI(TAG, "Starting Listen Mode...");
    state = radio.startReceive();

    uint8_t rx_buffer[sizeof(TelemetryF32V1)];
    uint32_t lastWatchdogKick = xTaskGetTickCount();

    while (1) {
        if (receivedFlag) {
            receivedFlag = false; 

            // Read the data
            state = radio.readData(rx_buffer, sizeof(TelemetryF32V1));

            if (state == RADIOLIB_ERR_NONE) {
                TelemetryF32V1 pkt;
                memcpy(&pkt, rx_buffer, sizeof(pkt));

                if (pkt.magic1 == 0xCA && pkt.magic2 == 0xFE) {
                    
                    // Calculate overall velocity magnitude from the 3 axes
                    float vel = sqrt((pkt.velocityX * pkt.velocityX) + 
                                     (pkt.velocityY * pkt.velocityY) + 
                                     (pkt.velocityZ * pkt.velocityZ));

                    // Print exactly what the Python script expects:
                    // TLM, sample_index, rssi, ax, ay, az, gx, velocity, gy, altitude, temp, press
                    printf("TLM,%u,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                           pkt.count,
                           radio.getRSSI(),
                           pkt.accelX, pkt.accelY, pkt.accelZ,
                           pkt.gyroX,
                           vel,
                           pkt.gyroY,
                           pkt.altitude,
                           pkt.temp,
                           pkt.pressure);
                }
            } else {
                // Keep ESP_LOGW here so debugging info goes to stderr/logs, 
                // but won't be parsed by your Python script as telemetry.
                ESP_LOGW(TAG, "RX Error or CRC Mismatch: %d", state);
            }

            // Put radio back into RX mode
            radio.startReceive();
            lastWatchdogKick = xTaskGetTickCount();
        }

        // --- THE FIXES FOR "FREEZING" ---

        // 1. Safety Timeout: If no packet for 10 seconds, force a radio restart
        if ((xTaskGetTickCount() - lastWatchdogKick) > pdMS_TO_TICKS(10000)) {
            // Log as debug so it doesn't clutter serial if python is listening
            ESP_LOGD(TAG, "Forcing RX restart...");
            radio.startReceive();
            lastWatchdogKick = xTaskGetTickCount();
        }

        // 2. Feed the FreeRTOS Watchdog
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ===================== MAIN =====================
extern "C" void app_main(void) {
    // Priority 5 is better to avoid starving system tasks (IDLE0)
    xTaskCreate(fsk_rx_task, "fsk_rx_task", 1024 * 6, NULL, 5, NULL);
}
