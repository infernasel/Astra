/**
 * ASTRA Programming Language
 * Raspberry Pi Hardware Abstraction Layer
 * 
 * This file implements the HAL interface for Raspberry Pi platforms.
 */

#ifndef ASTRA_RASPBERRY_PI_HAL_H
#define ASTRA_RASPBERRY_PI_HAL_H

#include "../common/hal.h"
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>

namespace astra {
namespace hal {

/**
 * Raspberry Pi I2C implementation
 */
class RaspberryPiI2CDevice : public I2CDevice {
private:
    int fd;
    uint8_t address;
    bool initialized;
    std::string busPath;

public:
    RaspberryPiI2CDevice(uint8_t busNum = 1) 
        : fd(-1), address(0), initialized(false), 
          busPath("/dev/i2c-" + std::to_string(busNum)) {}
    
    ~RaspberryPiI2CDevice() {
        if (fd >= 0) {
            close(fd);
        }
    }
    
    bool begin(uint8_t address) override {
        this->address = address;
        fd = open(busPath.c_str(), O_RDWR);
        if (fd < 0) return false;
        
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            close(fd);
            fd = -1;
            return false;
        }
        
        initialized = true;
        return true;
    }
    
    void end() override {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
        initialized = false;
    }
    
    bool write(uint8_t data) override {
        if (!initialized) return false;
        return ::write(fd, &data, 1) == 1;
    }
    
    bool write(const uint8_t* data, size_t length) override {
        if (!initialized) return false;
        return ::write(fd, data, length) == static_cast<ssize_t>(length);
    }
    
    int read() override {
        if (!initialized) return -1;
        uint8_t data;
        if (::read(fd, &data, 1) != 1) return -1;
        return data;
    }
    
    int read(uint8_t* buffer, size_t length) override {
        if (!initialized) return -1;
        return ::read(fd, buffer, length);
    }
    
    bool writeRegister(uint8_t reg, uint8_t data) override {
        if (!initialized) return false;
        uint8_t buf[2] = {reg, data};
        return ::write(fd, buf, 2) == 2;
    }
    
    int readRegister(uint8_t reg) override {
        if (!initialized) return -1;
        if (::write(fd, &reg, 1) != 1) return -1;
        uint8_t data;
        if (::read(fd, &data, 1) != 1) return -1;
        return data;
    }
};

/**
 * Raspberry Pi SPI implementation
 */
class RaspberryPiSPIDevice : public SPIDevice {
private:
    int fd;
    uint8_t csPin;
    bool initialized;
    std::string devicePath;
    uint8_t mode;
    uint8_t bitsPerWord;
    uint32_t speed;

public:
    RaspberryPiSPIDevice(uint8_t busNum = 0) 
        : fd(-1), csPin(0), initialized(false), 
          devicePath("/dev/spidev0." + std::to_string(busNum)),
          mode(0), bitsPerWord(8), speed(1000000) {}
    
    ~RaspberryPiSPIDevice() {
        if (fd >= 0) {
            close(fd);
        }
    }
    
    bool begin(uint8_t csPin) override {
        this->csPin = csPin;
        fd = open(devicePath.c_str(), O_RDWR);
        if (fd < 0) return false;
        
        // Set SPI mode
        if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
            close(fd);
            fd = -1;
            return false;
        }
        
        // Set bits per word
        if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0) {
            close(fd);
            fd = -1;
            return false;
        }
        
        // Set max speed
        if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            close(fd);
            fd = -1;
            return false;
        }
        
        initialized = true;
        return true;
    }
    
    void end() override {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
        initialized = false;
    }
    
    void setClockDivider(uint8_t divider) override {
        if (!initialized) return;
        // Convert divider to speed in Hz
        speed = 16000000 / divider;
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    }
    
    void setDataMode(uint8_t mode) override {
        if (!initialized) return;
        this->mode = mode;
        ioctl(fd, SPI_IOC_WR_MODE, &this->mode);
    }
    
    void setBitOrder(bool msbFirst) override {
        // Not directly supported in Linux SPI API
        // SPI in Linux is MSB first by default
    }
    
    uint8_t transfer(uint8_t data) override {
        if (!initialized) return 0;
        
        uint8_t rx = 0;
        struct spi_ioc_transfer tr = {0};
        tr.tx_buf = (unsigned long)&data;
        tr.rx_buf = (unsigned long)&rx;
        tr.len = 1;
        tr.speed_hz = speed;
        tr.bits_per_word = bitsPerWord;
        
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            return 0;
        }
        
        return rx;
    }
    
    void transfer(const uint8_t* txData, uint8_t* rxData, size_t length) override {
        if (!initialized) return;
        
        struct spi_ioc_transfer tr = {0};
        tr.tx_buf = (unsigned long)txData;
        tr.rx_buf = (unsigned long)rxData;
        tr.len = length;
        tr.speed_hz = speed;
        tr.bits_per_word = bitsPerWord;
        
        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    }
};

/**
 * Raspberry Pi UART implementation
 */
class RaspberryPiUARTDevice : public UARTDevice {
private:
    int fd;
    bool initialized;
    std::string devicePath;

public:
    RaspberryPiUARTDevice(uint8_t portNum = 0) 
        : fd(-1), initialized(false), 
          devicePath("/dev/ttyAMA" + std::to_string(portNum)) {}
    
    ~RaspberryPiUARTDevice() {
        if (fd >= 0) {
            close(fd);
        }
    }
    
    bool begin(unsigned long baudRate) override {
        fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) return false;
        
        // Configure UART
        struct termios options;
        tcgetattr(fd, &options);
        
        // Set baud rate
        speed_t speed;
        switch (baudRate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B9600; break;
        }
        
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);
        
        // 8N1 mode, no flow control
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;
        
        // Raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        
        // Raw output
        options.c_oflag &= ~OPOST;
        
        // Apply settings
        tcsetattr(fd, TCSANOW, &options);
        
        initialized = true;
        return true;
    }
    
    void end() override {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
        initialized = false;
    }
    
    int available() override {
        if (!initialized) return 0;
        
        int bytes;
        ioctl(fd, FIONREAD, &bytes);
        return bytes;
    }
    
    int read() override {
        if (!initialized) return -1;
        
        uint8_t data;
        if (::read(fd, &data, 1) != 1) return -1;
        return data;
    }
    
    int read(uint8_t* buffer, size_t length) override {
        if (!initialized) return 0;
        return ::read(fd, buffer, length);
    }
    
    size_t write(uint8_t data) override {
        if (!initialized) return 0;
        return ::write(fd, &data, 1);
    }
    
    size_t write(const uint8_t* buffer, size_t length) override {
        if (!initialized) return 0;
        return ::write(fd, buffer, length);
    }
    
    void flush() override {
        if (!initialized) return;
        tcdrain(fd);
    }
};

/**
 * Raspberry Pi HAL implementation
 */
class RaspberryPiHAL : public HAL {
private:
    std::shared_ptr<RaspberryPiI2CDevice> i2cDevice;
    std::shared_ptr<RaspberryPiSPIDevice> spiDevice;
    std::shared_ptr<RaspberryPiUARTDevice> uartDevice;
    
    // Initialize WiringPi
    bool initWiringPi() {
        static bool initialized = false;
        if (!initialized) {
            if (wiringPiSetup() == -1) {
                return false;
            }
            initialized = true;
        }
        return true;
    }

public:
    RaspberryPiHAL() : 
        i2cDevice(std::make_shared<RaspberryPiI2CDevice>()),
        spiDevice(std::make_shared<RaspberryPiSPIDevice>()),
        uartDevice(std::make_shared<RaspberryPiUARTDevice>()) {
        initWiringPi();
    }
    
    // Platform information
    PlatformType getPlatformType() const override {
        return PlatformType::RASPBERRY_PI;
    }
    
    std::string getPlatformName() const override {
        return "Raspberry Pi";
    }
    
    std::string getVersionString() const override {
        std::ifstream modelFile("/proc/device-tree/model");
        if (modelFile.is_open()) {
            std::string model;
            std::getline(modelFile, model);
            return model;
        }
        return "Raspberry Pi (Unknown Model)";
    }
    
    // Time functions
    uint32_t millis() const override {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch());
        return duration.count();
    }
    
    uint32_t micros() const override {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch());
        return duration.count();
    }
    
    void delay(uint32_t ms) const override {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
    
    void delayMicroseconds(uint32_t us) const override {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }
    
    // GPIO functions
    void pinMode(uint8_t pin, PinMode mode) override {
        if (!initWiringPi()) return;
        
        switch (mode) {
            case PinMode::INPUT:
                ::pinMode(pin, INPUT);
                break;
            case PinMode::OUTPUT:
                ::pinMode(pin, OUTPUT);
                break;
            case PinMode::INPUT_PULLUP:
                ::pinMode(pin, INPUT);
                ::pullUpDnControl(pin, PUD_UP);
                break;
            case PinMode::INPUT_PULLDOWN:
                ::pinMode(pin, INPUT);
                ::pullUpDnControl(pin, PUD_DOWN);
                break;
            case PinMode::PWM_OUTPUT:
                ::pinMode(pin, PWM_OUTPUT);
                break;
            default:
                ::pinMode(pin, INPUT);
                break;
        }
    }
    
    void digitalWrite(uint8_t pin, PinValue value) override {
        if (!initWiringPi()) return;
        ::digitalWrite(pin, value == PinValue::HIGH ? HIGH : LOW);
    }
    
    PinValue digitalRead(uint8_t pin) override {
        if (!initWiringPi()) return PinValue::LOW;
        return ::digitalRead(pin) == HIGH ? PinValue::HIGH : PinValue::LOW;
    }
    
    uint16_t analogRead(uint8_t pin) override {
        if (!initWiringPi()) return 0;
        // Raspberry Pi doesn't have built-in ADC
        // This would require an external ADC chip
        return 0;
    }
    
    void analogWrite(uint8_t pin, uint16_t value) override {
        if (!initWiringPi()) return;
        // Map value to 0-1024 range for PWM
        ::pwmWrite(pin, value & 0x3FF);
    }
    
    void analogWriteResolution(uint8_t bits) override {
        // Not supported on Raspberry Pi
    }
    
    void analogReadResolution(uint8_t bits) override {
        // Not supported on Raspberry Pi
    }
    
    // PWM functions
    void pwmWrite(uint8_t pin, uint16_t value) override {
        if (!initWiringPi()) return;
        ::pwmWrite(pin, value & 0x3FF);
    }
    
    void pwmFrequency(uint8_t pin, uint32_t frequency) override {
        // Not directly supported in WiringPi
    }
    
    // Communication interfaces
    std::shared_ptr<I2CDevice> getI2C(uint8_t busNum) override {
        i2cDevice = std::make_shared<RaspberryPiI2CDevice>(busNum);
        return i2cDevice;
    }
    
    std::shared_ptr<SPIDevice> getSPI(uint8_t busNum) override {
        spiDevice = std::make_shared<RaspberryPiSPIDevice>(busNum);
        return spiDevice;
    }
    
    std::shared_ptr<UARTDevice> getUART(uint8_t portNum) override {
        uartDevice = std::make_shared<RaspberryPiUARTDevice>(portNum);
        return uartDevice;
    }
    
    // System functions
    void reboot() override {
        system("sudo reboot");
    }
    
    float getCpuTemperature() override {
        std::ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
        if (tempFile.is_open()) {
            int temp;
            tempFile >> temp;
            return temp / 1000.0f;
        }
        return 0.0f;
    }
    
    float getSupplyVoltage() override {
        // Not directly available on Raspberry Pi
        return 0.0f;
    }
    
    uint32_t getFreeMemory() override {
        std::ifstream memInfo("/proc/meminfo");
        if (memInfo.is_open()) {
            std::string line;
            while (std::getline(memInfo, line)) {
                if (line.find("MemAvailable:") != std::string::npos) {
                    std::istringstream iss(line);
                    std::string label;
                    uint32_t value;
                    std::string unit;
                    iss >> label >> value >> unit;
                    return value * 1024; // Convert from KB to bytes
                }
            }
        }
        return 0;
    }
    
    // File system functions
    bool hasFileSystem() const override {
        return true;
    }
    
    bool fileExists(const std::string& path) override {
        struct stat buffer;
        return (stat(path.c_str(), &buffer) == 0);
    }
    
    std::string readFile(const std::string& path) override {
        std::ifstream file(path);
        if (!file.is_open()) return "";
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }
    
    bool writeFile(const std::string& path, const std::string& data) override {
        std::ofstream file(path);
        if (!file.is_open()) return false;
        
        file << data;
        return !file.fail();
    }
    
    bool appendFile(const std::string& path, const std::string& data) override {
        std::ofstream file(path, std::ios::app);
        if (!file.is_open()) return false;
        
        file << data;
        return !file.fail();
    }
    
    bool removeFile(const std::string& path) override {
        return remove(path.c_str()) == 0;
    }
    
    std::vector<std::string> listDirectory(const std::string& path) override {
        std::vector<std::string> result;
        DIR* dir = opendir(path.c_str());
        if (!dir) return result;
        
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            if (name != "." && name != "..") {
                result.push_back(name);
            }
        }
        
        closedir(dir);
        return result;
    }
    
    // Network functions
    bool hasNetwork() const override {
        return true;
    }
    
    bool connectWiFi(const std::string& ssid, const std::string& password) override {
        // Raspberry Pi uses wpa_supplicant for WiFi configuration
        // This is a simplified implementation
        std::string command = "wpa_passphrase \"" + ssid + "\" \"" + password + 
                             "\" | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf > /dev/null";
        system(command.c_str());
        system("sudo wpa_cli -i wlan0 reconfigure");
        
        // Wait for connection
        delay(5000);
        return isWiFiConnected();
    }
    
    bool isWiFiConnected() override {
        // Check if wlan0 has an IP address
        std::string output = executeCommand("ifconfig wlan0 | grep 'inet '");
        return !output.empty();
    }
    
    std::string getIPAddress() override {
        std::string output = executeCommand("hostname -I | awk '{print $1}'");
        if (!output.empty() && output[output.length() - 1] == '\n') {
            output.pop_back();
        }
        return output;
    }
    
    bool httpGet(const std::string& url, std::string& response) override {
        std::string command = "curl -s \"" + url + "\"";
        response = executeCommand(command);
        return !response.empty();
    }
    
    bool httpPost(const std::string& url, const std::string& data, std::string& response) override {
        std::string command = "curl -s -X POST -H \"Content-Type: application/json\" -d '" + 
                             data + "' \"" + url + "\"";
        response = executeCommand(command);
        return !response.empty();
    }
    
private:
    std::string executeCommand(const std::string& command) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
        if (!pipe) {
            return "";
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }
};

} // namespace hal
} // namespace astra

#endif // ASTRA_RASPBERRY_PI_HAL_H