#include "HRTimerTest.h"
Timer::TaskData &Timer::HighResolutionTimerTest::HRTimerTest::getTaskData() {
  return mTaskData;
}

void Timer::HighResolutionTimerTest::HRTimerTest::periodic_timer_callback(
    void *arg) {

  auto taskData = static_cast<Timer::TaskData *>(arg);

  gpio_set_level(taskData->mOutputConfigForISR.mGpioPin,
                 taskData->mOutputConfigForISR.mCurrentOutputStatus);
  taskData->mOutputConfigForISR.mCurrentOutputStatus ^= 1;

  xSemaphoreGiveFromISR(taskData->mSemaphore, nullptr);
  //  xSemaphoreGive(timerSemaphore);
}

Timer::HighResolutionTimerTest::HRTimerTest::HRTimerTest() {
  mTaskData.mSemaphore = xSemaphoreCreateBinary();
  gpio_reset_pin(mTaskData.mOutputConfigForISR.mGpioPin);
  gpio_set_direction(mTaskData.mOutputConfigForISR.mGpioPin, GPIO_MODE_OUTPUT);
  gpio_set_level(mTaskData.mOutputConfigForISR.mGpioPin,
                 mTaskData.mOutputConfigForISR.mCurrentOutputStatus);
  gpio_reset_pin(mTaskData.mOutputConfigForTask.mGpioPin);
  gpio_set_direction(mTaskData.mOutputConfigForTask.mGpioPin, GPIO_MODE_OUTPUT);
  gpio_set_level(mTaskData.mOutputConfigForTask.mGpioPin,
                 mTaskData.mOutputConfigForTask.mCurrentOutputStatus);
}

void Timer::HighResolutionTimerTest::HRTimerTest::initPeriodicTimer(size_t us) {
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &periodic_timer_callback,
      .arg = &mTaskData,
      /* name is optional, but may help identify the timer when
         debugging */
      .name = "periodic_timer",
      .skip_unhandled_events = false};

  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, us));
  /* The timer has been created but is not running yet */
}
Timer::TaskData Timer::HighResolutionTimerTest::HRTimerTest::mTaskData = {
    .mOutputConfigForISR = {.mGpioPin = GPIO_NUM_19},
    .mOutputConfigForTask = {.mGpioPin = GPIO_NUM_18},
    .mTimerMux = portMUX_INITIALIZER_UNLOCKED};