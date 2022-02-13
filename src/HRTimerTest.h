#pragma once

#include "TaskData.h"
#include <driver/timer.h>
namespace Timer {
namespace HighResolutionTimerTest {
class HRTimerTest {
  TaskData mTaskData = {.mOutputConfigForISR = {.mGpioPin = GPIO_NUM_19},
                        .mOutputConfigForTask = {.mGpioPin = GPIO_NUM_18},
                        .mTimerMux = portMUX_INITIALIZER_UNLOCKED};

public:
  TaskData &getTaskData();

private:
  bool mCurrentLedStatus{true};
  static void periodic_timer_callback(void *arg);

public:
  HRTimerTest();
  void initPeriodicTimer(size_t us);
};

} // namespace HighResolutionTimerTest
} // namespace Timer