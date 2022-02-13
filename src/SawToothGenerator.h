#pragma once
#include "TaskData.h"
#include <array>
#include <driver/dac_common.h>
#include <driver/gpio.h>
#include <driver/timer.h>
//#include <freertos/FreeRTOS.h>
//#include <freertos/

namespace Generator {

typedef struct {
  timer_group_t timer_group;
  timer_idx_t timer_idx;
  size_t alarm_interval;
  timer_autoreload_t auto_reload;
} TimerInfo;

class SawToothGenerator {
public:
  struct GeneratorData {
    gpio_num_t mGpioNum;
    dac_channel_t mDacChannel;
    uint8_t mCurrentValue{0};
    portMUX_TYPE mTimerMux = portMUX_INITIALIZER_UNLOCKED;
    TimerInfo mTimerInfo;
  };

private:
  // std::array<GeneratorData, 2> mGeneratorData;
  GeneratorData mGeneratorData;
  static constexpr uint8_t TIMER_DIVIDER = 2u;
  static constexpr size_t TIMER_SCALER_S = TIMER_BASE_CLK / TIMER_DIVIDER;
  static constexpr double TIMER_SCALER_US =
      0.000001 * TIMER_BASE_CLK / TIMER_DIVIDER;

  void initTimer(timer_group_t group, timer_idx_t timer,
                 timer_autoreload_t auto_reload, size_t timer_interval_usec);
  void initGenerator(size_t freqInHz);

public:
  SawToothGenerator(gpio_num_t gpioNumber, uint8_t startValue);
  void start(size_t freqInHz, timer_idx_t timerIdx);
};
} // namespace Generator