/**
 * ASTRA Programming Language
 * Hardware Abstraction Layer Factory
 * 
 * This file implements the factory function to create the appropriate HAL
 * implementation based on the current platform.
 */

#include "hal.h"

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
#include "../arduino/arduino_hal.h"
#elif defined(ARDUINO_ARCH_ESP32)
#include "../esp32/esp32_hal.h"
#elif defined(ARDUINO_ARCH_STM32)
#include "../stm32/stm32_hal.h"
#elif defined(__linux__)
#include "../raspberry_pi/raspberry_pi_hal.h"
#endif

namespace astra {
namespace hal {

/**
 * Get the HAL instance for the current platform
 */
std::shared_ptr<HAL> getHAL() {
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
    return std::make_shared<ArduinoHAL>();
#elif defined(ARDUINO_ARCH_ESP32)
    return std::make_shared<ESP32HAL>();
#elif defined(ARDUINO_ARCH_STM32)
    return std::make_shared<STM32HAL>();
#elif defined(__linux__)
    // Check if we're on a Raspberry Pi
    std::ifstream modelFile("/proc/device-tree/model");
    if (modelFile.is_open()) {
        std::string model;
        std::getline(modelFile, model);
        if (model.find("Raspberry Pi") != std::string::npos) {
            return std::make_shared<RaspberryPiHAL>();
        }
    }
    
    // Default to a generic Linux HAL (not implemented yet)
    return nullptr;
#else
    // Default to a generic HAL (not implemented yet)
    return nullptr;
#endif
}

} // namespace hal
} // namespace astra