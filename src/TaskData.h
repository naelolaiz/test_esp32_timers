#pragma once
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace Timer {
struct TaskData {
  struct OutputConfig {
    gpio_num_t mGpioPin;
    bool mCurrentOutputStatus = false;
  };
  OutputConfig mOutputConfigForISR;
  OutputConfig mOutputConfigForTask;
  SemaphoreHandle_t mSemaphore;
  portMUX_TYPE mTimerMux;
  void *mExtraCtx;
};
} // namespace Timer