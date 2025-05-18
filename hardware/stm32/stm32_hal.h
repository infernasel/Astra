/**
 * ASTRA Programming Language
 * STM32 Hardware Abstraction Layer
 * 
 * This file implements the HAL interface for STM32 platforms.
 */

#ifndef ASTRA_STM32_HAL_H
#define ASTRA_STM32_HAL_H

#include "../common/hal.h"

#if defined(ARDUINO_ARCH_STM32)
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareTimer.h>

namespace astra {
namespace hal {

/**
 * STM32 I2C implementation
 */
class STM32I2CDevice : public I2CDevice {
private:
    TwoWire& wire;
    uint8_t address;
    bool initialized;

public:
    STM32I2CDevice(TwoWire& wire = Wire) : wire(wire), address(0), initialized(false) {}
    
    bool begin(uint8_t address) override {
        this->address = address;
        wire.begin();
        initialized = true;
        return true;
    }
    
    void end() override {
        wire.end();
        initialized = false;
    }
    
    bool write(uint8_t data) override {
        if (!initialized) return false;
        wire.beginTransmission(address);
        wire.write(data);
        return wire.endTransmission() == 0;
    }
    
    bool write(const uint8_t* data, size_t length) override {
        if (!initialized) return false;
        wire.beginTransmission(address);
        wire.write(data, length);
        return wire.endTransmission() == 0;
    }
    
    int read() override {
        if (!initialized) return -1;
        if (wire.requestFrom(address, (uint8_t)1) != 1) return -1;
        return wire.available() ? wire.read() : -1;
    }
    
    int read(uint8_t* buffer, size_t length) override {
        if (!initialized) return -1;
        if (wire.requestFrom(address, (uint8_t)length) != length) return -1;
        size_t i = 0;
        while (wire.available() && i < length) {
            buffer[i++] = wire.read();
        }
        return i;
    }
    
    bool writeRegister(uint8_t reg, uint8_t data) override {
        if (!initialized) return false;
        wire.beginTransmission(address);
        wire.write(reg);
        wire.write(data);
        return wire.endTransmission() == 0;
    }
    
    int readRegister(uint8_t reg) override {
        if (!initialized) return -1;
        wire.beginTransmission(address);
        wire.write(reg);
        if (wire.endTransmission() != 0) return -1;
        if (wire.requestFrom(address, (uint8_t)1) != 1) return -1;
        return wire.available() ? wire.read() : -1;
    }
};

/**
 * STM32 SPI implementation
 */
class STM32SPIDevice : public SPIDevice {
private:
    SPIClass& spi;
    uint8_t csPin;
    bool initialized;

public:
    STM32SPIDevice(SPIClass& spi = SPI) : spi(spi), csPin(0), initialized(false) {}
    
    bool begin(uint8_t csPin) override {
        this->csPin = csPin;
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        spi.begin();
        initialized = true;
        return true;
    }
    
    void end() override {
        spi.end();
        initialized = false;
    }
    
    void setClockDivider(uint8_t divider) override {
        if (!initialized) return;
        // STM32 SPI uses frequency in Hz instead of divider
        uint32_t frequency = 72000000 / divider;
        spi.setClockDivider(divider);
    }
    
    void setDataMode(uint8_t mode) override {
        if (!initialized) return;
        spi.setDataMode(mode);
    }
    
    void setBitOrder(bool msbFirst) override {
        if (!initialized) return;
        spi.setBitOrder(msbFirst ? MSBFIRST : LSBFIRST);
    }
    
    uint8_t transfer(uint8_t data) override {
        if (!initialized) return 0;
        digitalWrite(csPin, LOW);
        uint8_t result = spi.transfer(data);
        digitalWrite(csPin, HIGH);
        return result;
    }
    
    void transfer(const uint8_t* txData, uint8_t* rxData, size_t length) override {
        if (!initialized) return;
        digitalWrite(csPin, LOW);
        for (size_t i = 0; i < length; i++) {
            rxData[i] = spi.transfer(txData[i]);
        }
        digitalWrite(csPin, HIGH);
    }
};

/**
 * STM32 UART implementation
 */
class STM32UARTDevice : public UARTDevice {
private:
    HardwareSerial& serial;
    bool initialized;

public:
    STM32UARTDevice(HardwareSerial& serial = Serial) : serial(serial), initialized(false) {}
    
    bool begin(unsigned long baudRate) override {
        serial.begin(baudRate);
        initialized = true;
        return true;
    }
    
    void end() override {
        serial.end();
        initialized = false;
    }
    
    int available() override {
        if (!initialized) return 0;
        return serial.available();
    }
    
    int read() override {
        if (!initialized) return -1;
        return serial.read();
    }
    
    int read(uint8_t* buffer, size_t length) override {
        if (!initialized) return 0;
        return serial.readBytes(buffer, length);
    }
    
    size_t write(uint8_t data) override {
        if (!initialized) return 0;
        return serial.write(data);
    }
    
    size_t write(const uint8_t* buffer, size_t length) override {
        if (!initialized) return 0;
        return serial.write(buffer, length);
    }
    
    void flush() override {
        if (!initialized) return;
        serial.flush();
    }
};

/**
 * STM32 HAL implementation
 */
class STM32HAL : public HAL {
private:
    std::shared_ptr<STM32I2CDevice> i2cDevice;
    std::shared_ptr<STM32SPIDevice> spiDevice;
    std::shared_ptr<STM32UARTDevice> uartDevice;
    uint8_t analogWriteRes;
    uint8_t analogReadRes;
    
    // PWM timer management
    struct PWMChannel {
        uint8_t pin;
        HardwareTimer* timer;
        uint32_t channel;
        uint32_t frequency;
        bool used;
    };
    
    static const uint8_t MAX_PWM_CHANNELS = 16;
    PWMChannel pwmChannels[MAX_PWM_CHANNELS];
    
    // Initialize PWM channels
    void initPWMChannels() {
        for (uint8_t i = 0; i < MAX_PWM_CHANNELS; i++) {
            pwmChannels[i].pin = 0;
            pwmChannels[i].timer = nullptr;
            pwmChannels[i].channel = 0;
            pwmChannels[i].frequency = 1000;
            pwmChannels[i].used = false;
        }
    }
    
    // Find or allocate a PWM channel for a pin
    int8_t getPWMChannel(uint8_t pin) {
        // Check if pin already has a channel
        for (uint8_t i = 0; i < MAX_PWM_CHANNELS; i++) {
            if (pwmChannels[i].used && pwmChannels[i].pin == pin) {
                return i;
            }
        }
        
        // Allocate a new channel
        for (uint8_t i = 0; i < MAX_PWM_CHANNELS; i++) {
            if (!pwmChannels[i].used) {
                pwmChannels[i].pin = pin;
                pwmChannels[i].used = true;
                
                // Get timer and channel for this pin
                TIM_TypeDef* instance = (TIM_TypeDef*)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
                if (instance == nullptr) return -1;
                
                uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
                if (channel == 0) return -1;
                
                pwmChannels[i].timer = new HardwareTimer(instance);
                pwmChannels[i].channel = channel;
                
                // Configure timer
                pwmChannels[i].timer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
                pwmChannels[i].timer->setOverflow(pwmChannels[i].frequency, HERTZ_FORMAT);
                pwmChannels[i].timer->setCaptureCompare(channel, 0, RESOLUTION_8B_COMPARE_FORMAT);
                pwmChannels[i].timer->resume();
                
                return i;
            }
        }
        
        return -1; // No free channels
    }

public:
    STM32HAL() : 
        i2cDevice(std::make_shared<STM32I2CDevice>()),
        spiDevice(std::make_shared<STM32SPIDevice>()),
        uartDevice(std::make_shared<STM32UARTDevice>()),
        analogWriteRes(12),
        analogReadRes(12) {
        initPWMChannels();
    }
    
    ~STM32HAL() {
        // Clean up PWM timers
        for (uint8_t i = 0; i < MAX_PWM_CHANNELS; i++) {
            if (pwmChannels[i].used && pwmChannels[i].timer != nullptr) {
                delete pwmChannels[i].timer;
            }
        }
    }
    
    // Platform information
    PlatformType getPlatformType() const override {
        return PlatformType::STM32;
    }
    
    std::string getPlatformName() const override {
        return "STM32";
    }
    
    std::string getVersionString() const override {
        return ARDUINO_BOARD;
    }
    
    // Time functions
    uint32_t millis() const override {
        return ::millis();
    }
    
    uint32_t micros() const override {
        return ::micros();
    }
    
    void delay(uint32_t ms) const override {
        ::delay(ms);
    }
    
    void delayMicroseconds(uint32_t us) const override {
        ::delayMicroseconds(us);
    }
    
    // GPIO functions
    void pinMode(uint8_t pin, PinMode mode) override {
        switch (mode) {
            case PinMode::INPUT:
                ::pinMode(pin, INPUT);
                break;
            case PinMode::OUTPUT:
                ::pinMode(pin, OUTPUT);
                break;
            case PinMode::INPUT_PULLUP:
                ::pinMode(pin, INPUT_PULLUP);
                break;
            case PinMode::INPUT_PULLDOWN:
                ::pinMode(pin, INPUT_PULLDOWN);
                break;
            case PinMode::ANALOG_INPUT:
                ::pinMode(pin, INPUT_ANALOG);
                break;
            case PinMode::ANALOG_OUTPUT:
            case PinMode::PWM_OUTPUT:
                ::pinMode(pin, OUTPUT);
                getPWMChannel(pin);
                break;
            default:
                ::pinMode(pin, INPUT);
                break;
        }
    }
    
    void digitalWrite(uint8_t pin, PinValue value) override {
        ::digitalWrite(pin, value == PinValue::HIGH ? HIGH : LOW);
    }
    
    PinValue digitalRead(uint8_t pin) override {
        return ::digitalRead(pin) == HIGH ? PinValue::HIGH : PinValue::LOW;
    }
    
    uint16_t analogRead(uint8_t pin) override {
        return ::analogRead(pin);
    }
    
    void analogWrite(uint8_t pin, uint16_t value) override {
        int8_t channel = getPWMChannel(pin);
        if (channel < 0) {
            // Fall back to standard analogWrite
            ::analogWrite(pin, value);
            return;
        }
        
        // Scale value to the current resolution
        uint32_t maxValue = (1 << analogWriteRes) - 1;
        uint32_t duty = (value * 255) / maxValue;
        
        pwmChannels[channel].timer->setCaptureCompare(
            pwmChannels[channel].channel, 
            duty, 
            RESOLUTION_8B_COMPARE_FORMAT
        );
    }
    
    void analogWriteResolution(uint8_t bits) override {
        analogWriteRes = bits;
        analogWriteResolution(bits);
    }
    
    void analogReadResolution(uint8_t bits) override {
        analogReadRes = bits;
        analogReadResolution(bits);
    }
    
    // PWM functions
    void pwmWrite(uint8_t pin, uint16_t value) override {
        analogWrite(pin, value);
    }
    
    void pwmFrequency(uint8_t pin, uint32_t frequency) override {
        int8_t channel = getPWMChannel(pin);
        if (channel < 0) return;
        
        pwmChannels[channel].frequency = frequency;
        pwmChannels[channel].timer->setOverflow(frequency, HERTZ_FORMAT);
    }
    
    // Communication interfaces
    std::shared_ptr<I2CDevice> getI2C(uint8_t busNum) override {
        if (busNum == 0) {
            i2cDevice = std::make_shared<STM32I2CDevice>(Wire);
        } else {
            i2cDevice = std::make_shared<STM32I2CDevice>(Wire1);
        }
        return i2cDevice;
    }
    
    std::shared_ptr<SPIDevice> getSPI(uint8_t busNum) override {
        if (busNum == 0) {
            spiDevice = std::make_shared<STM32SPIDevice>(SPI);
        } else {
            spiDevice = std::make_shared<STM32SPIDevice>(SPI1);
        }
        return spiDevice;
    }
    
    std::shared_ptr<UARTDevice> getUART(uint8_t portNum) override {
        switch (portNum) {
            case 0:
                uartDevice = std::make_shared<STM32UARTDevice>(Serial);
                break;
            case 1:
                uartDevice = std::make_shared<STM32UARTDevice>(Serial1);
                break;
            case 2:
                uartDevice = std::make_shared<STM32UARTDevice>(Serial2);
                break;
            default:
                uartDevice = std::make_shared<STM32UARTDevice>(Serial);
                break;
        }
        return uartDevice;
    }
    
    // System functions
    void reboot() override {
        NVIC_SystemReset();
    }
    
    float getCpuTemperature() override {
        #ifdef ARDUINO_NUCLEO_F103RB
        // STM32F103 has internal temperature sensor
        analogReadResolution(12);
        int rawTemp = analogRead(ATEMP);
        analogReadResolution(analogReadRes);
        
        // Convert raw value to temperature
        float voltage = (rawTemp * 3.3f) / 4096.0f;
        return ((voltage - 0.76f) / 0.0025f) + 25.0f;
        #else
        return 0.0f;
        #endif
    }
    
    float getSupplyVoltage() override {
        #ifdef ARDUINO_NUCLEO_F103RB
        analogReadResolution(12);
        int rawVref = analogRead(AVREF);
        analogReadResolution(analogReadRes);
        
        // Convert raw value to voltage
        return (1.2f * 4096.0f) / rawVref;
        #else
        return 0.0f;
        #endif
    }
    
    uint32_t getFreeMemory() override {
        // Not directly available on STM32
        return 0;
    }
    
    // File system functions
    bool hasFileSystem() const override {
        return false; // STM32 doesn't have built-in file system
    }
    
    bool fileExists(const std::string& path) override {
        return false;
    }
    
    std::string readFile(const std::string& path) override {
        return "";
    }
    
    bool writeFile(const std::string& path, const std::string& data) override {
        return false;
    }
    
    bool appendFile(const std::string& path, const std::string& data) override {
        return false;
    }
    
    bool removeFile(const std::string& path) override {
        return false;
    }
    
    std::vector<std::string> listDirectory(const std::string& path) override {
        return std::vector<std::string>();
    }
    
    // Network functions
    bool hasNetwork() const override {
        return false; // STM32 doesn't have built-in networking
    }
    
    bool connectWiFi(const std::string& ssid, const std::string& password) override {
        return false;
    }
    
    bool isWiFiConnected() override {
        return false;
    }
    
    std::string getIPAddress() override {
        return "";
    }
    
    bool httpGet(const std::string& url, std::string& response) override {
        return false;
    }
    
    bool httpPost(const std::string& url, const std::string& data, std::string& response) override {
        return false;
    }
};

} // namespace hal
} // namespace astra

#endif // defined(ARDUINO_ARCH_STM32)
#endif // ASTRA_STM32_HAL_H