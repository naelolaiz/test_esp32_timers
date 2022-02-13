#include "SawToothGenerator.h"
#include <cmath>
#include <driver/dac.h>
#include <driver/i2s.h>
#include <driver/timer.h>
#include <esp_log.h>

namespace Generator {

static bool IRAM_ATTR timer_group_isr_callback(void *args) {
  BaseType_t high_task_awoken = pdFALSE;
  ///
  SawToothGenerator::GeneratorData *generatorData =
      static_cast<SawToothGenerator::GeneratorData *>(args);

  uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(
      generatorData->mTimerInfo.timer_group,
      generatorData->mTimerInfo.timer_idx);
  //
  /* Now just send the event data back to the main program task */
  portENTER_CRITICAL_ISR(&generatorData->mTimerMux);

  // gpio_set_level(generatorData->mOutputConfigForISR.mGpioPin,
  //                generatorData->mOutputConfigForISR.mCurrentOutputStatus);
  //
  //  generatorData->mOutputConfigForISR.mCurrentOutputStatus ^= 1;

  dac_output_voltage(generatorData->mDacChannel,
                     ++generatorData->mCurrentValue);
  //  Critical code here
  portEXIT_CRITICAL_ISR(&generatorData->mTimerMux);
  if (!generatorData->mTimerInfo.auto_reload) {
    timer_counter_value += generatorData->mTimerInfo.alarm_interval;
    timer_group_set_alarm_value_in_isr(generatorData->mTimerInfo.timer_group,
                                       generatorData->mTimerInfo.timer_idx,
                                       timer_counter_value);
  }
  //  Give a semaphore that we can check in the loop
  //  TIMERG0.hw_timer[0].config.alarm_en = 1;
  // xSemaphoreGiveFromISR(generatorData->mSemaphore, &high_task_awoken);

  return high_task_awoken ==
         pdTRUE; // return whether we need to yield at the end of ISR
}

} // namespace Generator

void Generator::SawToothGenerator::initTimer(timer_group_t group,
                                             timer_idx_t timer,
                                             timer_autoreload_t auto_reload,
                                             size_t timer_interval_usec) {
  using namespace Generator;
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

  mGeneratorData.mTimerInfo.timer_group = group;
  mGeneratorData.mTimerInfo.timer_idx = timer;
  mGeneratorData.mTimerInfo.auto_reload = auto_reload;
  mGeneratorData.mTimerInfo.alarm_interval = scaledInterval;
  timer_isr_callback_add(group, timer, Generator::timer_group_isr_callback,
                         &mGeneratorData, 0);

  timer_start(group, timer);
}
void Generator::SawToothGenerator::initGenerator(size_t freqInHz) {

  ESP_ERROR_CHECK(gpio_reset_pin(mGeneratorData.mGpioNum));
  ESP_ERROR_CHECK(
      gpio_set_direction(mGeneratorData.mGpioNum, GPIO_MODE_OUTPUT));

  ESP_ERROR_CHECK(dac_output_enable(mGeneratorData.mDacChannel));
  ESP_ERROR_CHECK(dac_output_voltage(mGeneratorData.mDacChannel,
                                     mGeneratorData.mCurrentValue));
}

Generator::SawToothGenerator::SawToothGenerator(gpio_num_t gpioNumber,
                                                uint8_t startValue) {
  if (gpioNumber != GPIO_NUM_25 && gpioNumber != GPIO_NUM_26) {
    ESP_LOGI("SweepManager", "Incorrect DAC gpio requested. Aborting");
    return;
  }
  mGeneratorData.mGpioNum = gpioNumber;
  mGeneratorData.mDacChannel =
      mGeneratorData.mGpioNum == GPIO_NUM_25 ? DAC_CHANNEL_1 : DAC_CHANNEL_2;
  mGeneratorData.mCurrentValue = startValue;
}

void Generator::SawToothGenerator::start(size_t freqInHz,
                                         timer_idx_t timerIdx) {
  initGenerator(freqInHz);
  const size_t periodInUs = round((1000000. / freqInHz) / 255.);
  ESP_LOGE("SweepGenerator", "setting timer to %u uS", periodInUs);
  initTimer(TIMER_GROUP_1, timerIdx, TIMER_AUTORELOAD_EN, periodInUs);
}

// extern Generator::SawToothGenerator sawToothGenerator(GPIO_NUM_25, 1000);