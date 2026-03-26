#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- RadioLib Includes ---
#include "EspHal.h"
#include <RadioLib.h>

static const char *TAG = "RX_STATION";

// ===================== LORA/FSK PINS (SX1276) =====================
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 5
#define LORA_RST 14
#define LORA_DIO0 26 // PayloadReady
#define LORA_DIO1 33 // FIFO Level (Recommended for FSK)

// ===================== STRUCTS =====================
#pragma pack(push, 1)
typedef struct {
  uint8_t magic1;  // 0xCA
  uint8_t magic2;  // 0xFE
  uint8_t version; // 1
  uint8_t count;   // wraps 0-255

  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure; // hPa
  float temp;     // °C
  float velocityX, velocityY, velocityZ;
  float altitude; // meters
} TelemetryF32V1;

// ADDED: The Image Chunk Struct
typedef struct {
  uint8_t magic1; // 0xBE
  uint8_t magic2; // 0xEF
  uint16_t image_id;
  uint16_t total_chunks;
  uint16_t chunk_index;
  uint8_t payload[50];
} ImageChunkPacket;
#pragma pack(pop)

// ===================== GLOBAL RADIO OBJECTS =====================
EspHal *hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);
Module *mod = new Module(hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
SX1276 radio = mod;

volatile bool receivedFlag = false;

void IRAM_ATTR setFlag(void) { receivedFlag = true; }

// ===================== RX TASK =====================
void fsk_rx_task(void *arg) {
  ESP_LOGI(TAG, "Initializing SX1276 FSK...");

  int state = radio.beginFSK(915.0, 50.0, 25.0, 125.0, 17, 40);
  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "FSK init failed! Code: %d", state);
    vTaskDelete(NULL);
    return;
  }

  uint8_t syncWord[] = {0x2D, 0xD4};
  radio.setSyncWord(syncWord, 2);
  radio.setCRC(true);

  radio.variablePacketLengthMode();

  radio.setDio0Action(setFlag, RISING);

  ESP_LOGI(TAG, "Starting Listen Mode...");
  state = radio.startReceive();

  uint8_t rx_buffer[256];
  uint32_t lastWatchdogKick = xTaskGetTickCount();

  while (1) {
    if (receivedFlag) {
      receivedFlag = false;

      // Find out exactly how big the incoming packet is
      size_t len = radio.getPacketLength();
      state = radio.readData(rx_buffer, len);

      if (state == RADIOLIB_ERR_NONE && len >= 2) {

        // ROUTER: Check Magic Bytes
        if (rx_buffer[0] == 0xCA && rx_buffer[1] == 0xFE) {
          // --- TELEMETRY ---
          if (len == sizeof(TelemetryF32V1)) {
            TelemetryF32V1 pkt;
            memcpy(&pkt, rx_buffer, sizeof(pkt));

            float vel = sqrt((pkt.velocityX * pkt.velocityX) +
                             (pkt.velocityY * pkt.velocityY) +
                             (pkt.velocityZ * pkt.velocityZ));

            printf("TLM,%u,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                   pkt.count, radio.getRSSI(), pkt.accelX, pkt.accelY,
                   pkt.accelZ, pkt.gyroX, vel, pkt.gyroY, pkt.altitude,
                   pkt.temp, pkt.pressure);
          }
        } else if (rx_buffer[0] == 0xBE && rx_buffer[1] == 0xEF) {
          // --- IMAGE CHUNK ---
          if (len == sizeof(ImageChunkPacket)) {
            ImageChunkPacket img;
            memcpy(&img, rx_buffer, sizeof(img));

            // Print safely as HEX for Python to catch
            printf("IMG,%u,%u,", img.chunk_index, img.total_chunks);
            for (int i = 0; i < 50; i++) {
              printf("%02X", img.payload[i]);
            }
            printf("\n");
          }
        }
      } else {
        ESP_LOGW(TAG, "RX Error %d or bad length: %d", state, len);
      }

      radio.startReceive();
      lastWatchdogKick = xTaskGetTickCount();
    }

    if ((xTaskGetTickCount() - lastWatchdogKick) > pdMS_TO_TICKS(10000)) {
      ESP_LOGD(TAG, "Forcing RX restart...");
      radio.startReceive();
      lastWatchdogKick = xTaskGetTickCount();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  xTaskCreate(fsk_rx_task, "fsk_rx_task", 1024 * 6, NULL, 5, NULL);
}
