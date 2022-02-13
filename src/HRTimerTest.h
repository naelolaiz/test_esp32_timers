#pragma once

#include "TaskData.h"
#include <driver/timer.h>
namespace Timer {
namespace HighResolutionTimerTest {
class HRTimerTest {
  static TaskData mTaskData;

public:
  static TaskData &getTaskData();

private:
  bool mCurrentLedStatus{true};
  static void periodic_timer_callback(void *arg);

public:
  HRTimerTest();
  void initPeriodicTimer(size_t us);
};

} // namespace HighResolutionTimerTest
} // namespace Timer