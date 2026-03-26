#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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

// ===================== IMAGE BUFFER CONFIG =====================
// Adjust based on your max camera resolution. 
// 64KB is usually safe for standard ESP32 internal RAM. 
// If using PSRAM, you can easily bump this to 256KB+.
#define MAX_IMAGE_SIZE_BYTES (64 * 1024) 
#define CHUNK_PAYLOAD_SIZE 50
#define MAX_CHUNKS (MAX_IMAGE_SIZE_BYTES / CHUNK_PAYLOAD_SIZE)

uint8_t *image_buffer = NULL;
bool *chunk_map = NULL;
uint32_t last_chunk_time = 0;

uint16_t current_image_id = 0xFFFF; // 0xFFFF = No active image
uint16_t expected_total_chunks = 0;
uint16_t chunks_received = 0;

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

typedef struct {
  uint8_t magic1; // 0xBE
  uint8_t magic2; // 0xEF
  uint16_t image_id;
  uint16_t total_chunks;
  uint16_t chunk_index;
  uint8_t payload[CHUNK_PAYLOAD_SIZE];
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
        } 
        else if (rx_buffer[0] == 0xBE && rx_buffer[1] == 0xEF) {
          // --- IMAGE CHUNK ---
          if (len == sizeof(ImageChunkPacket)) {
            ImageChunkPacket img;
            memcpy(&img, rx_buffer, sizeof(img));

            // 1. Detect if this is a brand new image
            if (img.image_id != current_image_id) {
              ESP_LOGI(TAG, "New Image Incoming! ID: %u, Expected Chunks: %u", img.image_id, img.total_chunks);
              current_image_id = img.image_id;
              expected_total_chunks = img.total_chunks;
              chunks_received = 0;
              
              // Clear previous image data and map
              memset(image_buffer, 0, MAX_IMAGE_SIZE_BYTES);
              memset(chunk_map, 0, MAX_CHUNKS * sizeof(bool));
            }

            // 2. Validate chunk bounds
            if (img.chunk_index < expected_total_chunks && img.chunk_index < MAX_CHUNKS) {
              
              // 3. Prevent duplicate chunk processing
              if (!chunk_map[img.chunk_index]) {
                
                // Write payload to correct offset in main buffer
                uint32_t offset = img.chunk_index * CHUNK_PAYLOAD_SIZE;
                memcpy(&image_buffer[offset], img.payload, sizeof(img.payload));
                
                // Mark as received
                chunk_map[img.chunk_index] = true;
                chunks_received++;
                last_chunk_time = xTaskGetTickCount();
                
                // Print progress occasionally to not flood the serial output
                if (chunks_received % 20 == 0) {
                   ESP_LOGI(TAG, "Image %u RX Progress: %u / %u", current_image_id, chunks_received, expected_total_chunks);
                }

                 // 4. Check if image is complete
                if (chunks_received == expected_total_chunks) {
                   uint32_t final_size = expected_total_chunks * CHUNK_PAYLOAD_SIZE;
                   ESP_LOGI(TAG, "=== IMAGE %u COMPLETE! Size: %lu bytes ===", current_image_id, final_size);
                   
                   // --- TRANSMIT FULL IMAGE SERIALLY ---
                   // Format: CAM,<image_id>,<final_size>,<long_hex_string>
                   printf("CAM,%u,%lu,", current_image_id, final_size);
                   
                   // Stream the data byte-by-byte as Hex
                   for (uint32_t i = 0; i < final_size; i++) {
                     printf("%02X", image_buffer[i]);
                   }
                   printf("\n");
                   
                   // Flush stdout to ensure the massive string is pushed out immediately
                   fflush(stdout);
                }              
              }
            } else {
               ESP_LOGW(TAG, "Chunk index %u out of bounds!", img.chunk_index);
            }
          }
        }
      } else {
        ESP_LOGW(TAG, "RX Error %d or bad length: %d", state, len);
      }

      radio.startReceive();
      lastWatchdogKick = xTaskGetTickCount();
    }

    // --- IMAGE TIMEOUT LOGIC ---
    if (current_image_id != 0xFFFF) {
      // If 3 seconds have passed since the last chunk, dump what we have!
      if ((xTaskGetTickCount() - last_chunk_time) > pdMS_TO_TICKS(3000)) {
        
        uint32_t final_size = expected_total_chunks * CHUNK_PAYLOAD_SIZE;
        ESP_LOGW(TAG, "=== IMAGE %u TIMEOUT! Missing %u chunks. ===", 
                 current_image_id, (expected_total_chunks - chunks_received));
        
        // --- NEW: Print exact missing chunks ---
        printf("MISSING_CHUNKS: ");
        for (uint16_t i = 0; i < expected_total_chunks; i++) {
            if (!chunk_map[i]) {
                printf("%u ", i);
            }
        }
        printf("\n");
        // ---------------------------------------

        ESP_LOGW(TAG, "Dumping partial image.");
        
        // Print the partial image anyway
        printf("CAM,%u,%lu,", current_image_id, final_size);
        for (uint32_t i = 0; i < final_size; i++) {
          printf("%02X", image_buffer[i]);
        }
        printf("\n");
        fflush(stdout);

        // Reset so we are ready for the next image
        current_image_id = 0xFFFF;
      }
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
  // Allocate the global buffers on the heap
  image_buffer = (uint8_t*)malloc(MAX_IMAGE_SIZE_BYTES);
  chunk_map = (bool*)malloc(MAX_CHUNKS * sizeof(bool));

  if (image_buffer == NULL || chunk_map == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for image buffers!");
      return; // Halt if memory allocation fails
  }

  // Clear buffers
  memset(image_buffer, 0, MAX_IMAGE_SIZE_BYTES);
  memset(chunk_map, 0, MAX_CHUNKS * sizeof(bool));

  xTaskCreate(fsk_rx_task, "fsk_rx_task", 1024 * 6, NULL, 5, NULL);
}
