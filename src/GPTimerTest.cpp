#include "GPTimerTest.h"
bool Timer::GeneralPurposeTimerTest::timer_group_isr_callback(void *args) {
  BaseType_t high_task_awoken = pdFALSE;
  ///
  TaskData *taskData = static_cast<TaskData *>(args);
  example_timer_info_t *info =
      static_cast<example_timer_info_t *>(taskData->mExtraCtx);

  uint64_t timer_counter_value =
      timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
  //
  /* Now just send the event data back to the main program task */
  portENTER_CRITICAL_ISR(&taskData->mTimerMux);

  gpio_set_level(taskData->mOutputConfigForISR.mGpioPin,
                 taskData->mOutputConfigForISR.mCurrentOutputStatus);

  taskData->mOutputConfigForISR.mCurrentOutputStatus ^= 1;

  //  Critical code here
  portEXIT_CRITICAL_ISR(&taskData->mTimerMux);
  if (!info->auto_reload) {
    timer_counter_value += info->alarm_interval;
    timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx,
                                       timer_counter_value);
  }
  //  Give a semaphore that we can check in the loop
  //  TIMERG0.hw_timer[0].config.alarm_en = 1;
  xSemaphoreGiveFromISR(taskData->mSemaphore, &high_task_awoken);

  return high_task_awoken ==
         pdTRUE; // return whether we need to yield at the end of ISR
}

void Timer::GeneralPurposeTimerTest::TestGeneralPurposeTimerOldAPI::
    example_tg_timer_init(timer_group_t group, timer_idx_t timer,
                          timer_autoreload_t auto_reload,
                          size_t timer_interval_usec) {
  /* Select and initialize basic parameters of the timer */
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload =
          auto_reload, // ? TIMER_AUTORELOAD_EN : TIMER_AUTORELOAD_DIS,
      .divider = TIMER_DIVIDER}; // default clock source is APB
  timer_init(group, timer, &config);

  /* Timer's counter will initially start from value below.
     Also, if auto_reload is set, this value will be automatically reload on
     alarm */
  timer_set_counter_value(group, timer, 0);

  const size_t scaledInterval =
      std::round(timer_interval_usec * TIMER_SCALER_US);
  /* Configure the alarm value and the interrupt on alarm. */
  timer_set_alarm_value(group, timer, scaledInterval);
  timer_enable_intr(group, timer);

  example_timer_info_t *timer_info = static_cast<example_timer_info_t *>(
      calloc(1, sizeof(example_timer_info_t)));
  timer_info->timer_group = group;
  timer_info->timer_idx = timer;
  timer_info->auto_reload = auto_reload;
  timer_info->alarm_interval = scaledInterval;
  mTaskData.mExtraCtx = static_cast<void *>(timer_info);
  timer_isr_callback_add(group, timer,
                         GeneralPurposeTimerTest::timer_group_isr_callback,
                         &mTaskData, 0);

  timer_start(group, timer);
}

void Timer::GeneralPurposeTimerTest::TestGeneralPurposeTimerOldAPI::
    initGenericTimerOldAPI(size_t timeout) {
  example_tg_timer_init(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN, timeout);
}

Timer::GeneralPurposeTimerTest::TestGeneralPurposeTimerOldAPI::
    TestGeneralPurposeTimerOldAPI() {
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