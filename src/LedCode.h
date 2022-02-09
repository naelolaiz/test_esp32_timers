#ifndef __ONBOARD_LED_MANAGER_H__
#define __ONBOARD_LED_MANAGER_H__

#include <driver/gpio.h>

namespace Misc {
class OnBoardLedManager {
  static constexpr gpio_num_t mOnBoardLedPin = GPIO_NUM_5; // 16;
  static constexpr uint32_t mInitialValue = 1; // inverted. Starting OFF
  static bool mCurrentValue;

public:
  OnBoardLedManager();
  ~OnBoardLedManager();
  static void toggleStatus();
  static void set(bool level);
  static void BlinkingLedTask(void *pvParameters);
  static void setRequestedValue(bool value);
};
} // namespace Misc
#endif // __ONBOARD_LED_MANAGER_H__
