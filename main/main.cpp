#include <stdio.h>
#include <string.h>
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
#define LORA_DIO0  26   // PayloadReady / PacketSent 
#define LORA_DIO1  33

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

// ===================== RX TASK =====================
void fsk_rx_task(void *arg) {
    ESP_LOGI(TAG, "Initializing SX1276 FSK Receiver...");

    // 1) Setup HAL and Radio
    EspHal* hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);
    SX1276* radio = new SX1276(new Module(hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1));

    // 2) Start Radio in FSK mode
    float freqMHz      = 915.0;
    float bitRateKbps  = 50.0;   // 50 kbps
    float freqDevKHz   = 25.0;   // 25 kHz deviation
    float rxBwKHz      = 100.0;  // 100 kHz RX bandwidth
    int8_t powerdBm    = 17;     // doesn't matter for RX, but required by API
    uint16_t preambleBits = 40;  // 5 bytes preamble = 40 bits

    int state = radio->beginFSK(freqMHz, bitRateKbps, freqDevKHz, rxBwKHz, powerdBm, preambleBits);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "FSK init failed! Code: %d", state);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "FSK Listening... Expecting %u bytes per packet.", (unsigned)sizeof(TelemetryF32V1));

    // 4) Receive loop
    uint8_t rx_buffer[256];
    // In fixed length mode, RadioLib expects exactly that length.
    radio->fixedPacketLengthMode(sizeof(TelemetryF32V1));

    uint8_t syncWord[] = { 0x2D, 0xD4 };
    radio->setSyncWord(syncWord, 2);

    while (1) {

        state = radio->receive(rx_buffer, sizeof(TelemetryF32V1));

        if (state == RADIOLIB_ERR_NONE) {
            size_t len = radio->getPacketLength();

            ESP_LOGI(TAG, "RX OK | Len: %u | RSSI: %.2f dBm",
                     (unsigned)len, radio->getRSSI());

            if (len == sizeof(TelemetryF32V1)) {
                TelemetryF32V1 pkt;
                memcpy(&pkt, rx_buffer, sizeof(pkt));

            if (pkt.magic1 == 0xCA && pkt.magic2 == 0xFE && pkt.version == 1) {
                // Start identifier for python
                // Evan added this so the python script ignores esp logs, etc.
                printf("TELEMETRY,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                       pkt.count, radio->getRSSI(),
                       pkt.accelX, pkt.accelY, pkt.accelZ,
                       pkt.gyroX,  pkt.gyroY,  pkt.gyroZ,
                       pkt.altitude, pkt.temp);
            }
            
            else {
                    ESP_LOGW(TAG, "Bad header/magic/version (got %02X %02X v%u)",
                             pkt.magic1, pkt.magic2, pkt.version);
                }
            } 
            
            else {
                ESP_LOGW(TAG, "Unexpected length %u (expected %u)",
                         (unsigned)len, (unsigned)sizeof(TelemetryF32V1));
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
    // COMMENT OUT TO TURN LOGS BACK ON
    //esp_log_level_set("*", ESP_LOG_NONE);
    
    xTaskCreate(fsk_rx_task, "fsk_rx_task", 4096 * 2, NULL, 5, NULL);
}
