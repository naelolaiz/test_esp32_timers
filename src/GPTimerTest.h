#pragma once

#include "TaskData.h"
#include <cmath>
#include <driver/timer.h>

namespace Timer {

/////////////
namespace GeneralPurposeTimerTest {
typedef struct {
  timer_group_t timer_group;
  timer_idx_t timer_idx;
  size_t alarm_interval;
  timer_autoreload_t auto_reload;
} example_timer_info_t;

static bool IRAM_ATTR timer_group_isr_callback(void *args);
class TestGeneralPurposeTimerOldAPI {

  TaskData mTaskData = {.mOutputConfigForISR = {.mGpioPin = GPIO_NUM_21},
                        .mOutputConfigForTask = {.mGpioPin = GPIO_NUM_22},
                        .mTimerMux = portMUX_INITIALIZER_UNLOCKED};

public:
  static constexpr uint8_t TIMER_DIVIDER = 64u;
  static constexpr size_t TIMER_SCALER_S = TIMER_BASE_CLK / TIMER_DIVIDER;
  static constexpr double TIMER_SCALER_US =
      0.000001 * TIMER_BASE_CLK / TIMER_DIVIDER;
  TaskData &getTaskData() { return mTaskData; }

private:
  void example_tg_timer_init(timer_group_t group, timer_idx_t timer,
                             timer_autoreload_t auto_reload,
                             size_t timer_interval_usec);

public:
  void initGenericTimerOldAPI(size_t timeout);
  TestGeneralPurposeTimerOldAPI();
};

} // namespace GeneralPurposeTimerTest
} // namespace Timer
