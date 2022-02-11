#if 0
#include <array>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_intr_alloc.h>
#include <soc/soc.h>

volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  //  Critical code here
  portEXIT_CRITICAL_ISR(&timerMux);
  //  Give a semaphore that we can check in the loop
  //  TIMERG0.hw_timer[0].config.alarm_en = 1;
  xSemaphoreGiveFromISR(timerSemaphore, nullptr);
}

class Led {
  const std::array<gpio_num_t, 2> mLedGpioNumbers{GPIO_NUM_5, GPIO_NUM_22};
  const bool mInitialState{false};
  bool mCurrentState{false};

public:
  Led() = default;
  void setup() {
    for (auto gpioNumber : mLedGpioNumbers) {
      gpio_reset_pin(gpioNumber);
      gpio_set_direction(gpioNumber, GPIO_MODE_OUTPUT);
    }
  }
  void setValue(bool value) {
    for (auto gpioNumber : mLedGpioNumbers) {
      gpio_set_level(gpioNumber, value);
    }
  }
  void toggle() {
    mCurrentState = !mCurrentState;
    setValue(mCurrentState);
  }
};
static void periodic_timer_callback(void *arg) {
  xSemaphoreGiveFromISR(timerSemaphore, nullptr);
  //  xSemaphoreGive(timerSemaphore);
}

static void myTask(void *arg) {
  auto led = static_cast<Led *>(arg);
  while (1) {
    while (true) {
      if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
        led->toggle();
      }
      vTaskDelay(1);
    }
  }
}

extern "C" {
void app_main();
}
static Led led;
void app_main() {
  led.setup();
  timerSemaphore = xSemaphoreCreateBinary();
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &periodic_timer_callback,
      .arg = nullptr,
      /* name is optional, but may help identify the timer when
         debugging */
      .name = "periodic",
      .skip_unhandled_events = false};

  esp_timer_handle_t periodic_timer;

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 2));
  /* The timer has been created but is not running yet */

  xTaskCreate(myTask, "blinking_led_task", 4096, &led, 5, NULL);
}
#endif