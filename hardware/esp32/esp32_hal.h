/**
 * ASTRA Programming Language
 * ESP32 Hardware Abstraction Layer
 * 
 * This file implements the HAL interface for ESP32 platforms.
 */

#ifndef ASTRA_ESP32_HAL_H
#define ASTRA_ESP32_HAL_H

#include "../common/hal.h"

#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <FS.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include <esp_system.h>
#include <esp_wifi.h>

namespace astra {
namespace hal {

/**
 * ESP32 I2C implementation
 */
class ESP32I2CDevice : public I2CDevice {
private:
    TwoWire& wire;
    uint8_t address;
    bool initialized;
    uint8_t sda;
    uint8_t scl;
    uint32_t frequency;

public:
    ESP32I2CDevice(TwoWire& wire = Wire, uint8_t sda = 21, uint8_t scl = 22, uint32_t frequency = 100000) 
        : wire(wire), address(0), initialized(false), sda(sda), scl(scl), frequency(frequency) {}
    
    bool begin(uint8_t address) override {
        this->address = address;
        wire.begin(sda, scl, frequency);
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
 * ESP32 SPI implementation
 */
class ESP32SPIDevice : public SPIDevice {
private:
    SPIClass& spi;
    uint8_t csPin;
    bool initialized;
    uint8_t sck;
    uint8_t miso;
    uint8_t mosi;

public:
    ESP32SPIDevice(SPIClass& spi = SPI, uint8_t sck = 18, uint8_t miso = 19, uint8_t mosi = 23) 
        : spi(spi), csPin(0), initialized(false), sck(sck), miso(miso), mosi(mosi) {}
    
    bool begin(uint8_t csPin) override {
        this->csPin = csPin;
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        spi.begin(sck, miso, mosi);
        initialized = true;
        return true;
    }
    
    void end() override {
        spi.end();
        initialized = false;
    }
    
    void setClockDivider(uint8_t divider) override {
        if (!initialized) return;
        // ESP32 SPI uses frequency in Hz instead of divider
        uint32_t frequency = 80000000 / divider;
        spi.setFrequency(frequency);
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
        spi.transferBytes(txData, rxData, length);
        digitalWrite(csPin, HIGH);
    }
};

/**
 * ESP32 UART implementation
 */
class ESP32UARTDevice : public UARTDevice {
private:
    HardwareSerial& serial;
    bool initialized;
    int8_t rxPin;
    int8_t txPin;

public:
    ESP32UARTDevice(HardwareSerial& serial = Serial, int8_t rxPin = -1, int8_t txPin = -1) 
        : serial(serial), initialized(false), rxPin(rxPin), txPin(txPin) {}
    
    bool begin(unsigned long baudRate) override {
        if (rxPin >= 0 && txPin >= 0) {
            serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
        } else {
            serial.begin(baudRate);
        }
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
 * ESP32 HAL implementation
 */
class ESP32HAL : public HAL {
private:
    std::shared_ptr<ESP32I2CDevice> i2cDevice;
    std::shared_ptr<ESP32SPIDevice> spiDevice;
    std::shared_ptr<ESP32UARTDevice> uartDevice;
    uint8_t analogWriteRes;
    uint8_t analogReadRes;
    
    // LEDC channel mapping for PWM
    struct PWMChannel {
        uint8_t pin;
        uint8_t channel;
        uint32_t frequency;
        uint8_t resolution;
        bool used;
    };
    
    static const uint8_t MAX_PWM_CHANNELS = 16;
    PWMChannel pwmChannels[MAX_PWM_CHANNELS];
    
    // Initialize PWM channels
    void initPWMChannels() {
        for (uint8_t i = 0; i < MAX_PWM_CHANNELS; i++) {
            pwmChannels[i].pin = 0;
            pwmChannels[i].channel = i;
            pwmChannels[i].frequency = 5000;
            pwmChannels[i].resolution = 8;
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
                
                // Configure LEDC channel
                ledc_channel_config_t ledc_channel = {
                    .gpio_num = pin,
                    .speed_mode = LEDC_HIGH_SPEED_MODE,
                    .channel = (ledc_channel_t)i,
                    .intr_type = LEDC_INTR_DISABLE,
                    .timer_sel = (ledc_timer_t)(i / 4),
                    .duty = 0,
                    .hpoint = 0
                };
                ledc_channel_config(&ledc_channel);
                
                // Configure LEDC timer
                ledc_timer_config_t ledc_timer = {
                    .speed_mode = LEDC_HIGH_SPEED_MODE,
                    .duty_resolution = (ledc_timer_bit_t)pwmChannels[i].resolution,
                    .timer_num = (ledc_timer_t)(i / 4),
                    .freq_hz = pwmChannels[i].frequency,
                    .clk_cfg = LEDC_AUTO_CLK
                };
                ledc_timer_config(&ledc_timer);
                
                return i;
            }
        }
        
        return -1; // No free channels
    }

public:
    ESP32HAL() : 
        i2cDevice(std::make_shared<ESP32I2CDevice>()),
        spiDevice(std::make_shared<ESP32SPIDevice>()),
        uartDevice(std::make_shared<ESP32UARTDevice>()),
        analogWriteRes(8),
        analogReadRes(12) {
        initPWMChannels();
        SPIFFS.begin(true);
    }
    
    // Platform information
    PlatformType getPlatformType() const override {
        return PlatformType::ESP32;
    }
    
    std::string getPlatformName() const override {
        return "ESP32";
    }
    
    std::string getVersionString() const override {
        return ESP.getSdkVersion();
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
                // ESP32 ADC pins don't need special setup
                break;
            case PinMode::ANALOG_OUTPUT:
            case PinMode::PWM_OUTPUT:
                // Allocate a PWM channel for this pin
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
        if (channel < 0) return;
        
        // Scale value to the current resolution
        uint32_t maxValue = (1 << pwmChannels[channel].resolution) - 1;
        uint32_t duty = (value * maxValue) / ((1 << analogWriteRes) - 1);
        
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel);
    }
    
    void analogWriteResolution(uint8_t bits) override {
        analogWriteRes = bits;
    }
    
    void analogReadResolution(uint8_t bits) override {
        analogReadRes = bits;
    }
    
    // PWM functions
    void pwmWrite(uint8_t pin, uint16_t value) override {
        analogWrite(pin, value);
    }
    
    void pwmFrequency(uint8_t pin, uint32_t frequency) override {
        int8_t channel = getPWMChannel(pin);
        if (channel < 0) return;
        
        pwmChannels[channel].frequency = frequency;
        
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = (ledc_timer_bit_t)pwmChannels[channel].resolution,
            .timer_num = (ledc_timer_t)(channel / 4),
            .freq_hz = frequency,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ledc_timer_config(&ledc_timer);
    }
    
    // Communication interfaces
    std::shared_ptr<I2CDevice> getI2C(uint8_t busNum) override {
        if (busNum == 0) {
            i2cDevice = std::make_shared<ESP32I2CDevice>(Wire, 21, 22);
        } else {
            i2cDevice = std::make_shared<ESP32I2CDevice>(Wire1, 32, 33);
        }
        return i2cDevice;
    }
    
    std::shared_ptr<SPIDevice> getSPI(uint8_t busNum) override {
        if (busNum == 0) {
            spiDevice = std::make_shared<ESP32SPIDevice>(SPI, 18, 19, 23);
        } else if (busNum == 1) {
            spiDevice = std::make_shared<ESP32SPIDevice>(SPI1, 14, 12, 13);
        } else {
            spiDevice = std::make_shared<ESP32SPIDevice>(SPI2, 25, 26, 27);
        }
        return spiDevice;
    }
    
    std::shared_ptr<UARTDevice> getUART(uint8_t portNum) override {
        switch (portNum) {
            case 0:
                uartDevice = std::make_shared<ESP32UARTDevice>(Serial);
                break;
            case 1:
                uartDevice = std::make_shared<ESP32UARTDevice>(Serial1, 9, 10);
                break;
            case 2:
                uartDevice = std::make_shared<ESP32UARTDevice>(Serial2, 16, 17);
                break;
            default:
                uartDevice = std::make_shared<ESP32UARTDevice>(Serial);
                break;
        }
        return uartDevice;
    }
    
    // System functions
    void reboot() override {
        ESP.restart();
    }
    
    float getCpuTemperature() override {
        return temperatureRead();
    }
    
    float getSupplyVoltage() override {
        return analogReadMilliVolts(36) / 1000.0f;
    }
    
    uint32_t getFreeMemory() override {
        return ESP.getFreeHeap();
    }
    
    // File system functions
    bool hasFileSystem() const override {
        return true;
    }
    
    bool fileExists(const std::string& path) override {
        return SPIFFS.exists(path.c_str());
    }
    
    std::string readFile(const std::string& path) override {
        File file = SPIFFS.open(path.c_str(), "r");
        if (!file) return "";
        
        std::string content;
        while (file.available()) {
            content += (char)file.read();
        }
        file.close();
        return content;
    }
    
    bool writeFile(const std::string& path, const std::string& data) override {
        File file = SPIFFS.open(path.c_str(), "w");
        if (!file) return false;
        
        size_t written = file.print(data.c_str());
        file.close();
        return written == data.length();
    }
    
    bool appendFile(const std::string& path, const std::string& data) override {
        File file = SPIFFS.open(path.c_str(), "a");
        if (!file) return false;
        
        size_t written = file.print(data.c_str());
        file.close();
        return written == data.length();
    }
    
    bool removeFile(const std::string& path) override {
        return SPIFFS.remove(path.c_str());
    }
    
    std::vector<std::string> listDirectory(const std::string& path) override {
        std::vector<std::string> result;
        File root = SPIFFS.open(path.c_str());
        if (!root || !root.isDirectory()) return result;
        
        File file = root.openNextFile();
        while (file) {
            result.push_back(file.name());
            file = root.openNextFile();
        }
        return result;
    }
    
    // Network functions
    bool hasNetwork() const override {
        return true;
    }
    
    bool connectWiFi(const std::string& ssid, const std::string& password) override {
        WiFi.begin(ssid.c_str(), password.c_str());
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            attempts++;
        }
        return WiFi.status() == WL_CONNECTED;
    }
    
    bool isWiFiConnected() override {
        return WiFi.status() == WL_CONNECTED;
    }
    
    std::string getIPAddress() override {
        if (WiFi.status() == WL_CONNECTED) {
            IPAddress ip = WiFi.localIP();
            char ipStr[16];
            sprintf(ipStr, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
            return std::string(ipStr);
        }
        return "";
    }
    
    bool httpGet(const std::string& url, std::string& response) override {
        HTTPClient http;
        http.begin(url.c_str());
        int httpCode = http.GET();
        if (httpCode > 0) {
            response = http.getString().c_str();
            http.end();
            return true;
        }
        http.end();
        return false;
    }
    
    bool httpPost(const std::string& url, const std::string& data, std::string& response) override {
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
        return false;
    }
};

} // namespace hal
} // namespace astra

#endif // ARDUINO_ARCH_ESP32
#endif // ASTRA_ESP32_HAL_H