/**
 * ASTRA Programming Language
 * Hardware Abstraction Layer (HAL)
 * 
 * This file defines the common interface for all hardware platforms
 * supported by ASTRA. It provides a unified API for accessing hardware
 * features regardless of the underlying platform.
 */

#ifndef ASTRA_HAL_H
#define ASTRA_HAL_H

#include <stdint.h>
#include <string>
#include <vector>
#include <memory>

namespace astra {
namespace hal {

/**
 * Hardware platform types
 */
enum class PlatformType {
    UNKNOWN,
    ARDUINO,
    RASPBERRY_PI,
    ESP32,
    STM32,
    LINUX,
    WINDOWS,
    MACOS
};

/**
 * Pin modes
 */
enum class PinMode {
    INPUT,
    OUTPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    ANALOG_INPUT,
    ANALOG_OUTPUT,
    PWM_OUTPUT,
    SERVO_OUTPUT,
    I2C_SDA,
    I2C_SCL,
    SPI_MOSI,
    SPI_MISO,
    SPI_SCK,
    SPI_CS,
    UART_TX,
    UART_RX
};

/**
 * Digital pin values
 */
enum class PinValue {
    LOW,
    HIGH
};

/**
 * I2C device interface
 */
class I2CDevice {
public:
    virtual ~I2CDevice() = default;
    
    virtual bool begin(uint8_t address) = 0;
    virtual void end() = 0;
    virtual bool write(uint8_t data) = 0;
    virtual bool write(const uint8_t* data, size_t length) = 0;
    virtual int read() = 0;
    virtual int read(uint8_t* buffer, size_t length) = 0;
    virtual bool writeRegister(uint8_t reg, uint8_t data) = 0;
    virtual int readRegister(uint8_t reg) = 0;
};

/**
 * SPI device interface
 */
class SPIDevice {
public:
    virtual ~SPIDevice() = default;
    
    virtual bool begin(uint8_t csPin) = 0;
    virtual void end() = 0;
    virtual void setClockDivider(uint8_t divider) = 0;
    virtual void setDataMode(uint8_t mode) = 0;
    virtual void setBitOrder(bool msbFirst) = 0;
    virtual uint8_t transfer(uint8_t data) = 0;
    virtual void transfer(const uint8_t* txData, uint8_t* rxData, size_t length) = 0;
};

/**
 * UART device interface
 */
class UARTDevice {
public:
    virtual ~UARTDevice() = default;
    
    virtual bool begin(unsigned long baudRate) = 0;
    virtual void end() = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int read(uint8_t* buffer, size_t length) = 0;
    virtual size_t write(uint8_t data) = 0;
    virtual size_t write(const uint8_t* buffer, size_t length) = 0;
    virtual void flush() = 0;
};

/**
 * Hardware Abstraction Layer interface
 */
class HAL {
public:
    virtual ~HAL() = default;
    
    // Platform information
    virtual PlatformType getPlatformType() const = 0;
    virtual std::string getPlatformName() const = 0;
    virtual std::string getVersionString() const = 0;
    
    // Time functions
    virtual uint32_t millis() const = 0;
    virtual uint32_t micros() const = 0;
    virtual void delay(uint32_t ms) const = 0;
    virtual void delayMicroseconds(uint32_t us) const = 0;
    
    // GPIO functions
    virtual void pinMode(uint8_t pin, PinMode mode) = 0;
    virtual void digitalWrite(uint8_t pin, PinValue value) = 0;
    virtual PinValue digitalRead(uint8_t pin) = 0;
    virtual uint16_t analogRead(uint8_t pin) = 0;
    virtual void analogWrite(uint8_t pin, uint16_t value) = 0;
    virtual void analogWriteResolution(uint8_t bits) = 0;
    virtual void analogReadResolution(uint8_t bits) = 0;
    
    // PWM functions
    virtual void pwmWrite(uint8_t pin, uint16_t value) = 0;
    virtual void pwmFrequency(uint8_t pin, uint32_t frequency) = 0;
    
    // Communication interfaces
    virtual std::shared_ptr<I2CDevice> getI2C(uint8_t busNum = 0) = 0;
    virtual std::shared_ptr<SPIDevice> getSPI(uint8_t busNum = 0) = 0;
    virtual std::shared_ptr<UARTDevice> getUART(uint8_t portNum = 0) = 0;
    
    // System functions
    virtual void reboot() = 0;
    virtual float getCpuTemperature() = 0;
    virtual float getSupplyVoltage() = 0;
    virtual uint32_t getFreeMemory() = 0;
    
    // File system functions (if available)
    virtual bool hasFileSystem() const = 0;
    virtual bool fileExists(const std::string& path) = 0;
    virtual std::string readFile(const std::string& path) = 0;
    virtual bool writeFile(const std::string& path, const std::string& data) = 0;
    virtual bool appendFile(const std::string& path, const std::string& data) = 0;
    virtual bool removeFile(const std::string& path) = 0;
    virtual std::vector<std::string> listDirectory(const std::string& path) = 0;
    
    // Network functions (if available)
    virtual bool hasNetwork() const = 0;
    virtual bool connectWiFi(const std::string& ssid, const std::string& password) = 0;
    virtual bool isWiFiConnected() = 0;
    virtual std::string getIPAddress() = 0;
    virtual bool httpGet(const std::string& url, std::string& response) = 0;
    virtual bool httpPost(const std::string& url, const std::string& data, std::string& response) = 0;
};

/**
 * Get the HAL instance for the current platform
 */
std::shared_ptr<HAL> getHAL();

} // namespace hal
} // namespace astra

#endif // ASTRA_HAL_H