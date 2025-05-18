/**
 * ASTRA Programming Language
 * Arduino Hardware Abstraction Layer
 * 
 * This file implements the HAL interface for Arduino platforms.
 */

#ifndef ASTRA_ARDUINO_HAL_H
#define ASTRA_ARDUINO_HAL_H

#include "../common/hal.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

namespace astra {
namespace hal {

/**
 * Arduino I2C implementation
 */
class ArduinoI2CDevice : public I2CDevice {
private:
    TwoWire& wire;
    uint8_t address;
    bool initialized;

public:
    ArduinoI2CDevice(TwoWire& wire = Wire) : wire(wire), address(0), initialized(false) {}
    
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
 * Arduino SPI implementation
 */
class ArduinoSPIDevice : public SPIDevice {
private:
    SPIClass& spi;
    uint8_t csPin;
    bool initialized;

public:
    ArduinoSPIDevice(SPIClass& spi = SPI) : spi(spi), csPin(0), initialized(false) {}
    
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
 * Arduino UART implementation
 */
class ArduinoUARTDevice : public UARTDevice {
private:
    HardwareSerial& serial;
    bool initialized;

public:
    ArduinoUARTDevice(HardwareSerial& serial = Serial) : serial(serial), initialized(false) {}
    
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
 * Arduino HAL implementation
 */
class ArduinoHAL : public HAL {
private:
    std::shared_ptr<ArduinoI2CDevice> i2cDevice;
    std::shared_ptr<ArduinoSPIDevice> spiDevice;
    std::shared_ptr<ArduinoUARTDevice> uartDevice;
    uint8_t analogWriteRes;
    uint8_t analogReadRes;

public:
    ArduinoHAL() : 
        i2cDevice(std::make_shared<ArduinoI2CDevice>()),
        spiDevice(std::make_shared<ArduinoSPIDevice>()),
        uartDevice(std::make_shared<ArduinoUARTDevice>()),
        analogWriteRes(8),
        analogReadRes(10) {}
    
    // Platform information
    PlatformType getPlatformType() const override {
        return PlatformType::ARDUINO;
    }
    
    std::string getPlatformName() const override {
        return "Arduino";
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
                #ifdef INPUT_PULLDOWN
                ::pinMode(pin, INPUT_PULLDOWN);
                #else
                ::pinMode(pin, INPUT);
                #endif
                break;
            case PinMode::ANALOG_INPUT:
                ::pinMode(pin, INPUT);
                break;
            case PinMode::ANALOG_OUTPUT:
            case PinMode::PWM_OUTPUT:
                ::pinMode(pin, OUTPUT);
                break;
            default:
                // Other modes may require specific setup for Arduino
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
        // Scale value if resolution is not 8 bits
        if (analogWriteRes != 8) {
            value = map(value, 0, (1 << analogWriteRes) - 1, 0, 255);
        }
        ::analogWrite(pin, value);
    }
    
    void analogWriteResolution(uint8_t bits) override {
        #ifdef ARDUINO_ARCH_SAM
        ::analogWriteResolution(bits);
        #endif
        #ifdef ARDUINO_ARCH_SAMD
        ::analogWriteResolution(bits);
        #endif
        #ifdef ARDUINO_ARCH_ESP32
        ::analogWriteResolution(bits);
        #endif
        analogWriteRes = bits;
    }
    
    void analogReadResolution(uint8_t bits) override {
        #ifdef ARDUINO_ARCH_SAM
        ::analogReadResolution(bits);
        #endif
        #ifdef ARDUINO_ARCH_SAMD
        ::analogReadResolution(bits);
        #endif
        #ifdef ARDUINO_ARCH_ESP32
        ::analogReadResolution(bits);
        #endif
        analogReadRes = bits;
    }
    
    // PWM functions
    void pwmWrite(uint8_t pin, uint16_t value) override {
        analogWrite(pin, value);
    }
    
    void pwmFrequency(uint8_t pin, uint32_t frequency) override {
        #ifdef ARDUINO_ARCH_ESP32
        // ESP32 specific implementation
        // ledcSetup(channel, frequency, resolution);
        #endif
        // Not supported on all Arduino platforms
    }
    
    // Communication interfaces
    std::shared_ptr<I2CDevice> getI2C(uint8_t busNum) override {
        return i2cDevice;
    }
    
    std::shared_ptr<SPIDevice> getSPI(uint8_t busNum) override {
        return spiDevice;
    }
    
    std::shared_ptr<UARTDevice> getUART(uint8_t portNum) override {
        return uartDevice;
    }
    
    // System functions
    void reboot() override {
        #ifdef ARDUINO_ARCH_ESP32
        ESP.restart();
        #else
        // Software reset for other Arduino platforms
        asm volatile ("jmp 0");
        #endif
    }
    
    float getCpuTemperature() override {
        #ifdef ARDUINO_ARCH_ESP32
        return temperatureRead();
        #else
        return 0.0f; // Not supported on all Arduino platforms
        #endif
    }
    
    float getSupplyVoltage() override {
        #ifdef ARDUINO_ARCH_ESP32
        return analogReadMilliVolts(36) / 1000.0f;
        #else
        return 0.0f; // Not supported on all Arduino platforms
        #endif
    }
    
    uint32_t getFreeMemory() override {
        #ifdef ARDUINO_ARCH_ESP32
        return ESP.getFreeHeap();
        #else
        extern int __heap_start, *__brkval;
        int v;
        return (uint32_t)&v - (__brkval == 0 ? (uint32_t)&__heap_start : (uint32_t)__brkval);
        #endif
    }
    
    // File system functions
    bool hasFileSystem() const override {
        #ifdef ARDUINO_ARCH_ESP32
        return true;
        #else
        return false;
        #endif
    }
    
    bool fileExists(const std::string& path) override {
        #ifdef ARDUINO_ARCH_ESP32
        return SPIFFS.exists(path.c_str());
        #else
        return false;
        #endif
    }
    
    std::string readFile(const std::string& path) override {
        #ifdef ARDUINO_ARCH_ESP32
        File file = SPIFFS.open(path.c_str(), "r");
        if (!file) return "";
        std::string content;
        while (file.available()) {
            content += (char)file.read();
        }
        file.close();
        return content;
        #else
        return "";
        #endif
    }
    
    bool writeFile(const std::string& path, const std::string& data) override {
        #ifdef ARDUINO_ARCH_ESP32
        File file = SPIFFS.open(path.c_str(), "w");
        if (!file) return false;
        size_t written = file.print(data.c_str());
        file.close();
        return written == data.length();
        #else
        return false;
        #endif
    }
    
    bool appendFile(const std::string& path, const std::string& data) override {
        #ifdef ARDUINO_ARCH_ESP32
        File file = SPIFFS.open(path.c_str(), "a");
        if (!file) return false;
        size_t written = file.print(data.c_str());
        file.close();
        return written == data.length();
        #else
        return false;
        #endif
    }
    
    bool removeFile(const std::string& path) override {
        #ifdef ARDUINO_ARCH_ESP32
        return SPIFFS.remove(path.c_str());
        #else
        return false;
        #endif
    }
    
    std::vector<std::string> listDirectory(const std::string& path) override {
        std::vector<std::string> result;
        #ifdef ARDUINO_ARCH_ESP32
        File root = SPIFFS.open(path.c_str());
        if (!root || !root.isDirectory()) return result;
        
        File file = root.openNextFile();
        while (file) {
            result.push_back(file.name());
            file = root.openNextFile();
        }
        #endif
        return result;
    }
    
    // Network functions
    bool hasNetwork() const override {
        #ifdef ARDUINO_ARCH_ESP32
        return true;
        #else
        return false;
        #endif
    }
    
    bool connectWiFi(const std::string& ssid, const std::string& password) override {
        #ifdef ARDUINO_ARCH_ESP32
        WiFi.begin(ssid.c_str(), password.c_str());
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            attempts++;
        }
        return WiFi.status() == WL_CONNECTED;
        #else
        return false;
        #endif
    }
    
    bool isWiFiConnected() override {
        #ifdef ARDUINO_ARCH_ESP32
        return WiFi.status() == WL_CONNECTED;
        #else
        return false;
        #endif
    }
    
    std::string getIPAddress() override {
        #ifdef ARDUINO_ARCH_ESP32
        if (WiFi.status() == WL_CONNECTED) {
            IPAddress ip = WiFi.localIP();
            char ipStr[16];
            sprintf(ipStr, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
            return std::string(ipStr);
        }
        #endif
        return "";
    }
    
    bool httpGet(const std::string& url, std::string& response) override {
        #ifdef ARDUINO_ARCH_ESP32
        HTTPClient http;
        http.begin(url.c_str());
        int httpCode = http.GET();
        if (httpCode > 0) {
            response = http.getString().c_str();
            http.end();
            return true;
        }
        http.end();
        #endif
        return false;
    }
    
    bool httpPost(const std::string& url, const std::string& data, std::string& response) override {
        #ifdef ARDUINO_ARCH_ESP32
        HTTPClient http;
        http.begin(url.c_str());
        http.addHeader("Content-Type", "application/json");
        int httpCode = http.POST(data.c_str());
        if (httpCode > 0) {
            response = http.getString().c_str();
            http.end();
            return true;
        }
        http.end();
        #endif
        return false;
    }
};

} // namespace hal
} // namespace astra

#endif // ASTRA_ARDUINO_HAL_H