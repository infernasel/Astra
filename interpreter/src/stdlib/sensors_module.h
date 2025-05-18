#ifndef ASTRA_SENSORS_MODULE_H
#define ASTRA_SENSORS_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace sensors {

/**
 * Initialize the Sensors module in the VM
 */
void initialize(VM& vm);

// GPS sensor class methods
Value gpsConstructor(const std::vector<Value>& args);
Value gpsGetPosition(const std::vector<Value>& args);
Value gpsGetVelocity(const std::vector<Value>& args);
Value gpsGetAltitude(const std::vector<Value>& args);
Value gpsGetHeading(const std::vector<Value>& args);
Value gpsGetAccuracy(const std::vector<Value>& args);
Value gpsGetSatelliteCount(const std::vector<Value>& args);
Value gpsUpdate(const std::vector<Value>& args);
Value gpsToString(const std::vector<Value>& args);

// IMU sensor class methods
Value imuConstructor(const std::vector<Value>& args);
Value imuGetAcceleration(const std::vector<Value>& args);
Value imuGetAngularVelocity(const std::vector<Value>& args);
Value imuGetMagneticField(const std::vector<Value>& args);
Value imuGetOrientation(const std::vector<Value>& args);
Value imuGetTemperature(const std::vector<Value>& args);
Value imuUpdate(const std::vector<Value>& args);
Value imuToString(const std::vector<Value>& args);

// Barometer sensor class methods
Value barometerConstructor(const std::vector<Value>& args);
Value barometerGetPressure(const std::vector<Value>& args);
Value barometerGetAltitude(const std::vector<Value>& args);
Value barometerGetTemperature(const std::vector<Value>& args);
Value barometerUpdate(const std::vector<Value>& args);
Value barometerToString(const std::vector<Value>& args);

// Radar sensor class methods
Value radarConstructor(const std::vector<Value>& args);
Value radarGetDistance(const std::vector<Value>& args);
Value radarGetVelocity(const std::vector<Value>& args);
Value radarGetAngle(const std::vector<Value>& args);
Value radarGetTargets(const std::vector<Value>& args);
Value radarUpdate(const std::vector<Value>& args);
Value radarToString(const std::vector<Value>& args);

// Lidar sensor class methods
Value lidarConstructor(const std::vector<Value>& args);
Value lidarGetPointCloud(const std::vector<Value>& args);
Value lidarGetRange(const std::vector<Value>& args);
Value lidarGetResolution(const std::vector<Value>& args);
Value lidarUpdate(const std::vector<Value>& args);
Value lidarToString(const std::vector<Value>& args);

// Camera sensor class methods
Value cameraConstructor(const std::vector<Value>& args);
Value cameraGetImage(const std::vector<Value>& args);
Value cameraGetResolution(const std::vector<Value>& args);
Value cameraGetFieldOfView(const std::vector<Value>& args);
Value cameraUpdate(const std::vector<Value>& args);
Value cameraToString(const std::vector<Value>& args);

// Star tracker class methods
Value starTrackerConstructor(const std::vector<Value>& args);
Value starTrackerGetOrientation(const std::vector<Value>& args);
Value starTrackerGetAccuracy(const std::vector<Value>& args);
Value starTrackerGetStarCount(const std::vector<Value>& args);
Value starTrackerUpdate(const std::vector<Value>& args);
Value starTrackerToString(const std::vector<Value>& args);

// Sun sensor class methods
Value sunSensorConstructor(const std::vector<Value>& args);
Value sunSensorGetSunVector(const std::vector<Value>& args);
Value sunSensorGetIntensity(const std::vector<Value>& args);
Value sunSensorUpdate(const std::vector<Value>& args);
Value sunSensorToString(const std::vector<Value>& args);

// Earth sensor class methods
Value earthSensorConstructor(const std::vector<Value>& args);
Value earthSensorGetNadirVector(const std::vector<Value>& args);
Value earthSensorGetHorizonPoints(const std::vector<Value>& args);
Value earthSensorUpdate(const std::vector<Value>& args);
Value earthSensorToString(const std::vector<Value>& args);

// Sensor fusion functions
Value fuseSensors(const std::vector<Value>& args);
Value calculateOrientation(const std::vector<Value>& args);
Value calculatePosition(const std::vector<Value>& args);
Value calculateVelocity(const std::vector<Value>& args);
Value calculateAcceleration(const std::vector<Value>& args);
Value calculateAngularVelocity(const std::vector<Value>& args);
Value calculateAngularAcceleration(const std::vector<Value>& args);

// Helper functions
ObjectPtr createGPS(double accuracy, int satelliteCount);
ObjectPtr createIMU(double accelAccuracy, double gyroAccuracy, double magAccuracy);
ObjectPtr createBarometer(double accuracy);
ObjectPtr createRadar(double range, double accuracy);
ObjectPtr createLidar(double range, double resolution);
ObjectPtr createCamera(int width, int height, double fieldOfView);
ObjectPtr createStarTracker(double accuracy);
ObjectPtr createSunSensor(double accuracy);
ObjectPtr createEarthSensor(double accuracy);
bool isGPS(const Value& value);
bool isIMU(const Value& value);
bool isBarometer(const Value& value);
bool isRadar(const Value& value);
bool isLidar(const Value& value);
bool isCamera(const Value& value);
bool isStarTracker(const Value& value);
bool isSunSensor(const Value& value);
bool isEarthSensor(const Value& value);

} // namespace sensors
} // namespace stdlib
} // namespace astra

#endif // ASTRA_SENSORS_MODULE_H