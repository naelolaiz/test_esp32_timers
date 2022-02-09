#if 0
#include "LedCode.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Misc::OnBoardLedManager::~OnBoardLedManager() {
  gpio_reset_pin(mOnBoardLedPin);
}

bool Misc::OnBoardLedManager::mCurrentValue =
    Misc::OnBoardLedManager::mInitialValue;

Misc::OnBoardLedManager::OnBoardLedManager() {
  gpio_reset_pin(mOnBoardLedPin);
  gpio_set_direction(mOnBoardLedPin, GPIO_MODE_OUTPUT);
  gpio_set_level(mOnBoardLedPin, mCurrentValue);
}
void Misc::OnBoardLedManager::toggleStatus() {
  const bool currentValueToShow = !mCurrentValue;
  gpio_set_level(mOnBoardLedPin, currentValueToShow ? 1 : 0);
}
void Misc::OnBoardLedManager::set(bool level) {
  mCurrentValue = level;
  gpio_set_level(mOnBoardLedPin, level);
}
void Misc::OnBoardLedManager::BlinkingLedTask(void *pvParameters) {
  constexpr size_t msToWait = 500;
  while (true) {
    OnBoardLedManager *onBoardLedManager =
        static_cast<OnBoardLedManager *>(pvParameters);

    struct DataForTimer {
      OnBoardLedManager *ledManager = nullptr;
    } dataForTimer;
    dataForTimer.ledManager = onBoardLedManager;

    TaskHandle_t xHandle = nullptr;
    xTaskCreate(
        [](void *pvP) {
          DataForTimer *dataForTimer = static_cast<DataForTimer *>(pvP);
          while (true) {
            (void)dataForTimer;
            // auto optional =
            // dataForTimer->ledManager->mRequestedValue.load();

            //      if (optional.has_value()) {
            // dataForTimer->newRequestAvailable.store(true);
            //}
            vTaskDelay(1 / portTICK_PERIOD_MS);
          }
        },
        "long sleep unless new request", 2048, &dataForTimer, 5, &xHandle);

    int counter = 5000;
    while (counter > 0) // 0 && !dataForTimer.newRequestAvailable.load()) {

      vTaskDelay(1 / portTICK_PERIOD_MS);
    counter--;
  }

  //  if (xHandle != nullptr) {
  // vTaskDelete(xHandle);
  //}
  //}
  // else {
  //   toggleStatus();
  //   vTaskDelay(msToWait / portTICK_PERIOD_MS);
  // }
  //}
}

void Misc::OnBoardLedManager::setRequestedValue(bool value) {
  mRequestedValue.store(value);
}
#endif