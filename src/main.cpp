#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

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

#pragma pack(push, 1)
typedef struct {
  uint8_t  magic1;   // 0xCA
  uint8_t  magic2;   // 0xFE
  uint8_t  version;  // 1
  uint8_t  count;    // wraps 0-255

  // float32 payload (IEEE-754, little-endian)
  float accelX, accelY, accelZ;
  float gyroX,  gyroY,  gyroZ;
  float pressure;   // pick unit and stick to it (hPa in your code)
  float temp;       // °C in your code
  float velocityX, velocityY, velocityZ;
  float altitude;   // meters
} TelemetryF32V1;
#pragma pack(pop)

static void print_struct_telemetry(const TelemetryF32V1 *d) {
    printf("================ CANSAT TELEMETRY (BINARY) ================\n");
    printf("   [ACCEL]  X: %8.2f | Y: %8.2f | Z: %8.2f (mg)\n", d->accelX, d->accelY, d->accelZ);
    printf("   [GYRO]   X: %8.2f | Y: %8.2f | Z: %8.2f (dps)\n", d->gyroX, d->gyroY, d->gyroZ);
    printf("   ---------------------------------------------------------\n");
    printf("   [BARO]   Alt: %8.2f m | Press: %8.2f hPa\n", d->altitude, d->pressure);
    printf("   [TEMP]   %8.2f C\n", d->temp);
    printf("   ---------------------------------------------------------\n");
    printf("   [GPS]    Vn:  %8.2f m/s | Ve:    %8.2f m/s | Vz: %8.2f m/s\n",
           d->velocityX, d->velocityY, d->velocityZ);
    printf("===========================================================\n\n");
}

// ===================== RX TASK =====================
void lora_rx_task(void *arg) {
    ESP_LOGI(TAG, "Initializing LoRa Receiver...");

    // 1. Setup HAL and Radio
    EspHal* hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);
    SX1276* radio = new SX1276(new Module(hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1));

    // 2. Start Radio (match TX config!)
    int state = radio->begin();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio init failed! Code: %d", state);
        while(1) vTaskDelay(pdMS_TO_TICKS(100));
    }

    radio->setFrequency(915.0);
    radio->setBandwidth(125.0);
    radio->setSpreadingFactor(7);
    radio->setCodingRate(5);
    radio->setSyncWord(0x12);
    radio->setCRC(true);
    radio->setPreambleLength(8);

    ESP_LOGI(TAG, "Radio Listening... Expecting %u bytes per packet.", (unsigned)sizeof(TelemetryF32V1));

    // 3. Receive Loop
    uint8_t rx_buffer[256];

    while (1) {
        state = radio->receive(rx_buffer, sizeof(rx_buffer));

        if (state == RADIOLIB_ERR_NONE) {
            size_t len = radio->getPacketLength();

            ESP_LOGI(TAG, "RX OK | Len: %u | RSSI: %.2f dBm | SNR: %.2f dB",
                     (unsigned)len, radio->getRSSI(), radio->getSNR());
            if (len == sizeof(TelemetryF32V1)) {
            TelemetryF32V1 pkt;
            memcpy(&pkt, rx_buffer, sizeof(pkt));

                if (pkt.magic1 == 0xCA && pkt.magic2 == 0xFE && pkt.version == 1) {
                    printf("=========== TELEMETRY (float32) ===========\n");
                    printf("Pkt: %u | RSSI: %.1f dBm | SNR: %.1f dB\n", pkt.count, radio->getRSSI(), radio->getSNR());

                    printf("[ACCEL] X:%7.1f Y:%7.1f Z:%7.1f mg\n", pkt.accelX, pkt.accelY, pkt.accelZ);
                    printf("[GYRO ] X:%7.2f Y:%7.2f Z:%7.2f dps\n", pkt.gyroX, pkt.gyroY, pkt.gyroZ);

                    printf("[BARO ] Alt:%7.2f m  Press:%7.2f hPa  Temp:%5.2f C\n",
                        pkt.altitude, pkt.pressure, pkt.temp);

                    printf("[GPS  ] Vn:%6.2f Ve:%6.2f Vz:%6.2f m/s\n",
                        pkt.velocityX, pkt.velocityY, pkt.velocityZ);

                    printf("===========================================\n\n");
                }
            }
        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // ignore
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            ESP_LOGW(TAG, "CRC Error! Packet corrupted.");
        } else {
            ESP_LOGE(TAG, "RX Error code: %d", state);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===================== MAIN =====================
extern "C" void app_main(void) {
    xTaskCreate(lora_rx_task, "lora_rx_task", 4096 * 2, NULL, 5, NULL);
}
