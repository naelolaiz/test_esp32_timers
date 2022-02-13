#include "SawToothGenerator.h"
SawToothGenerator::SawToothGenerator(gpio_num_t gpioNumber)
    : mGpioNumber(gpioNumber) {}

extern SawToothGenerator sawToothGenerator(GPIO_NUM_25);