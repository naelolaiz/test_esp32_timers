#include <array>
#include <driver/gpio.h>
#include <driver/timer.h>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_intr_alloc.h>
#include <esp_task_wdt.h>
#include <soc/soc.h>

#define USE_GPTIMER 1

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

/////////////
typedef struct {
  timer_group_t timer_group;
  timer_idx_t timer_idx;
  size_t alarm_interval;
  timer_autoreload_t auto_reload;
} example_timer_info_t;
typedef struct {
  example_timer_info_t info;
  uint64_t timer_counter_value;
} example_timer_event_t;

static constexpr uint8_t TIMER_DIVIDER = 16u; // =5MHz
static constexpr size_t TIMER_SCALE = (TIMER_BASE_CLK / TIMER_DIVIDER);
static bool IRAM_ATTR timer_group_isr_callback(void *args) {
  BaseType_t high_task_awoken = pdFALSE;
  ///
  example_timer_info_t *info = (example_timer_info_t *)args;

  uint64_t timer_counter_value =
      timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
  //
  /* Now just send the event data back to the main program task */
  portENTER_CRITICAL_ISR(&timerMux);
  //  Critical code here
  portEXIT_CRITICAL_ISR(&timerMux);
  if (!info->auto_reload) {
    timer_counter_value += info->alarm_interval; //  *TIMER_SCALE / 5;
    timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx,
                                       timer_counter_value);
  }
  //  Give a semaphore that we can check in the loop
  //  TIMERG0.hw_timer[0].config.alarm_en = 1;
  xSemaphoreGiveFromISR(timerSemaphore, &high_task_awoken);

  return high_task_awoken ==
         pdTRUE; // return whether we need to yield at the end of ISR
#if 0

    /* Prepare basic event data that will be then sent back to task */
    example_timer_event_t evt = {.info.timer_group = info->timer_group,
                                 .info.timer_idx = info->timer_idx,
                                 .info.auto_reload = info->auto_reload,
                                 .info.alarm_interval = info->alarm_interval,
                                 .timer_counter_value = timer_counter_value};

    if (!info->auto_reload) {
      timer_counter_value += info->alarm_interval * TIMER_SCALE;
      timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx,
                                         timer_counter_value);
    }

#endif
}

class TestGeneralPurposeTimerOldAPI {
  static void example_tg_timer_init(timer_group_t group, timer_idx_t timer,
                                    timer_autoreload_t auto_reload,
                                    size_t timer_interval_sec) {
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

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer,
                          timer_interval_sec); // * TIMER_SCALE); // / 5);
    timer_enable_intr(group, timer);

    example_timer_info_t *timer_info = static_cast<example_timer_info_t *>(
        calloc(1, sizeof(example_timer_info_t)));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info,
                           0);

    timer_start(group, timer);
  }

public:
  void initGenericTimerOldAPI(size_t timeout) {
    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN, timeout);
  }
  TestGeneralPurposeTimerOldAPI() = default;
};

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
      // vTaskDelay(1);
    }
  }
}

void initPeriodicTimer() {
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
}

extern "C" {
void app_main();
}
static Led led;
void app_main() {
  led.setup();
  timerSemaphore = xSemaphoreCreateBinary();
#ifdef USE_GPTIMER
  TestGeneralPurposeTimerOldAPI *t = new TestGeneralPurposeTimerOldAPI;
  t->initGenericTimerOldAPI(50);
#else
  initPeriodicTimer();
#endif
  TaskHandle_t createdTaskHandle;
  xTaskCreate(myTask, "blinking_led_task", 4096, &led, 5, &createdTaskHandle);
  //  ESP_ERROR_CHECK(esp_task_wdt_delete(
  //      createdTaskHandle)); // TODO: temporal solution to make the routine
  //      work
  // without vTaskDelay
}
