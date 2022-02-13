#include <array>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "GPTimerTest.h"
#include "HRTimerTest.h"
#include "SawToothGenerator.h"
#include "TaskData.h"

static void myTask(void *arg) {
  auto *taskData = static_cast<Timer::TaskData *>(arg);
  while (1) {
    while (true) {
      if (xSemaphoreTake(taskData->mSemaphore, 0) == pdTRUE) {
        gpio_set_level(taskData->mOutputConfigForTask.mGpioPin,
                       taskData->mOutputConfigForTask.mCurrentOutputStatus);
        taskData->mOutputConfigForTask.mCurrentOutputStatus ^= 1;
      }
      // vTaskDelay(1);
    }
  }
}

extern "C" {
void app_main();
}
void app_main() {
  using namespace Timer;

  constexpr size_t timeInUs = 500;
  auto *gpTimerTest =
      new GeneralPurposeTimerTest::TestGeneralPurposeTimerOldAPI;
  gpTimerTest->initGenericTimerOldAPI(timeInUs);
  auto *hrTimerTest = new HighResolutionTimerTest::HRTimerTest;
  hrTimerTest->initPeriodicTimer(timeInUs);

  xTaskCreate(myTask, "blinking_led_task_gp", 4096, &gpTimerTest->getTaskData(),
              6, nullptr);
  xTaskCreate(myTask, "blinking_led_task_gp", 4096, &hrTimerTest->getTaskData(),
              5, nullptr);

  // static constexpr int taskCore = 0;
  // xTaskCreatePinnedToCore(myTask, /* Function to implement the task */
  //                         "blinking_led_task", /* Name of the task */
  //                         4096,                /* Stack size in words */
  //                         &led,                /* Task input parameter */
  //                         10,                  /* Priority of the task */
  //                         nullptr,             /* Task handle. */
  //                         taskCore); /* Core where the task should run */
}
