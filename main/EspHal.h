#ifndef ESP_HAL_H
#define ESP_HAL_H

#include <RadioLib.h>

// Check for the correct target
#if !defined(CONFIG_IDF_TARGET_ESP32)
#error This HAL is for the classic ESP32 (Ground Station chip).
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CRITICAL HEADERS FOR ESP-IDF v6.0
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "rom/gpio.h"
#include "soc/gpio_sig_map.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "hal/gpio_hal.h"
#include "soc/dport_reg.h"
#include "soc/rtc.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include <string.h>

#define LOW (0x0)
#define HIGH (0x1)
#define INPUT (0x01)
#define OUTPUT (0x03)
#define RISING (0x01)
#define FALLING (0x02)
#define NOP() asm volatile("nop")

#define MATRIX_DETACH_OUT_SIG (0x100)
#define MATRIX_DETACH_IN_LOW_PIN (0x30)
#define MHZ 1000000UL

#define ClkRegToFreq(reg)                                                      \
  (apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))

typedef union {
  uint32_t value;
  struct {
    uint32_t clkcnt_l : 6;
    uint32_t clkcnt_h : 6;
    uint32_t clkcnt_n : 6;
    uint32_t clkdiv_pre : 13;
    uint32_t clk_equ_sysclk : 1;
  };
} spiClk_t;

static uint32_t getApbFrequency() {
  rtc_cpu_freq_config_t conf;
  rtc_clk_cpu_freq_get_config(&conf);
  return (conf.freq_mhz >= 80) ? (80 * MHZ)
                               : ((conf.source_freq_mhz * MHZ) / conf.div);
}

static uint32_t spiFrequencyToClockDiv(uint32_t freq) {
  uint32_t apb_freq = getApbFrequency();
  if (freq >= apb_freq)
    return 0x80000000;
  const spiClk_t minFreqReg = {0x7FFFF000};
  uint32_t minFreq = ClkRegToFreq((spiClk_t *)&minFreqReg);
  if (freq < minFreq)
    return minFreqReg.value;

  uint8_t calN = 1;
  spiClk_t bestReg = {0};
  int32_t bestFreq = 0;
  while (calN <= 0x3F) {
    spiClk_t reg = {0};
    int32_t calFreq;
    int32_t calPre;
    reg.clkcnt_n = calN;
    int8_t calPreVari = -2;
    while (calPreVari++ <= 1) {
      calPre = (((apb_freq / (reg.clkcnt_n + 1)) / freq) - 1) + calPreVari;
      reg.clkdiv_pre = (calPre > 0x1FFF) ? 0x1FFF : (calPre <= 0 ? 0 : calPre);
      reg.clkcnt_l = ((reg.clkcnt_n + 1) / 2);
      calFreq = ClkRegToFreq(&reg);
      if (calFreq == (int32_t)freq) {
        memcpy(&bestReg, &reg, sizeof(bestReg));
        break;
      } else if (calFreq < (int32_t)freq) {
        if (abs((int)freq - calFreq) < abs((int)freq - bestFreq)) {
          bestFreq = calFreq;
          memcpy(&bestReg, &reg, sizeof(bestReg));
        }
      }
    }
    if (calFreq == (int32_t)freq)
      break;
    calN++;
  }
  return (bestReg.value);
}

class EspHal : public RadioLibHal {
public:
  EspHal(int8_t sck, int8_t miso, int8_t mosi)
      : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING), spiSCK(sck),
        spiMISO(miso), spiMOSI(mosi) {}

  void init() override { spiBegin(); }
  void term() override { spiEnd(); }

  void pinMode(uint32_t pin, uint32_t mode) override {
    if (pin == RADIOLIB_NC)
      return;
    gpio_config_t conf = {.pin_bit_mask = (1ULL << pin),
                          .mode = (gpio_mode_t)mode,
                          .pull_up_en = GPIO_PULLUP_DISABLE,
                          .pull_down_en = GPIO_PULLDOWN_DISABLE,
                          .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&conf);
  }

  void digitalWrite(uint32_t pin, uint32_t value) override {
    if (pin != RADIOLIB_NC)
      gpio_set_level((gpio_num_t)pin, value);
  }

  uint32_t digitalRead(uint32_t pin) override {
    return (pin == RADIOLIB_NC) ? 0 : gpio_get_level((gpio_num_t)pin);
  }

  void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void),
                       uint32_t mode) override {
    if (interruptNum == RADIOLIB_NC)
      return;
    gpio_install_isr_service(0);
    gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));
    gpio_isr_handler_add((gpio_num_t)interruptNum,
                         (void (*)(void *))interruptCb, NULL);
  }

  void detachInterrupt(uint32_t interruptNum) override {
    if (interruptNum == RADIOLIB_NC)
      return;
    gpio_isr_handler_remove((gpio_num_t)interruptNum);
    gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
  }

  void delay(unsigned long ms) override { vTaskDelay(pdMS_TO_TICKS(ms)); }

  void delayMicroseconds(unsigned long us) override {
    uint64_t start = esp_timer_get_time();
    while ((uint64_t)esp_timer_get_time() - start < us) {
      NOP();
    }
  }

  unsigned long millis() override {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
  }
  unsigned long micros() override {
    return (unsigned long)(esp_timer_get_time());
  }

  long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
    if (pin == RADIOLIB_NC)
      return 0;
    this->pinMode(pin, INPUT);
    uint32_t start = this->micros();
    while (this->digitalRead(pin) != state) {
      if ((this->micros() - start) > timeout)
        return 0;
    }
    uint32_t startPulse = this->micros();
    while (this->digitalRead(pin) == state) {
      if ((this->micros() - startPulse) > timeout)
        return 0;
    }
    return (this->micros() - startPulse);
  }

  void spiBegin() {
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI2_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI2_RST);
    this->spi->slave.val = 0;
    this->spi->pin.val = 0;
    this->spi->user.val = 0;
    this->spi->user.usr_mosi = 1;
    this->spi->user.usr_miso = 1;
    this->spi->user.doutdin = 1;
    for (uint8_t i = 0; i < 16; i++)
      this->spi->data_buf[i] = 0;
    this->spi->clock.val = spiFrequencyToClockDiv(2000000);

    this->pinMode(this->spiSCK, OUTPUT);
    this->pinMode(this->spiMISO, INPUT);
    this->pinMode(this->spiMOSI, OUTPUT);

    // USING THE v6.0 ROM NAMESPACES
    rom_gpio_matrix_out(this->spiSCK, HSPICLK_OUT_IDX, false, false);
    rom_gpio_matrix_in(this->spiMISO, HSPIQ_OUT_IDX, false);
    rom_gpio_matrix_out(this->spiMOSI, HSPID_IN_IDX, false,
                        false); // Add this one too if it's there
  }

  void spiBeginTransaction() {}

  uint8_t spiTransferByte(uint8_t b) {
    this->spi->mosi_dlen.usr_mosi_dbitlen = 7;
    this->spi->miso_dlen.usr_miso_dbitlen = 7;
    this->spi->data_buf[0] = b;
    this->spi->cmd.usr = 1;
    while (this->spi->cmd.usr)
      ;
    return (this->spi->data_buf[0] & 0xFF);
  }

  void spiTransfer(uint8_t *out, size_t len, uint8_t *in) {
    for (size_t i = 0; i < len; i++)
      in[i] = spiTransferByte(out[i]);
  }

  void spiEndTransaction() {}

  void spiEnd() {
    rom_gpio_matrix_out(this->spiSCK, MATRIX_DETACH_OUT_SIG, false, false);
    rom_gpio_matrix_in(this->spiMISO, MATRIX_DETACH_IN_LOW_PIN, false);
    rom_gpio_matrix_out(this->spiMOSI, MATRIX_DETACH_OUT_SIG, false, false);
  }

private:
  int8_t spiSCK, spiMISO, spiMOSI;
  volatile spi_dev_t *spi = (volatile spi_dev_t *)(DR_REG_SPI2_BASE);
};

#endif