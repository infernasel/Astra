#ifndef ASTRA_NAVIGATION_MODULE_H
#define ASTRA_NAVIGATION_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace navigation {

/**
 * Initialize the Navigation module in the VM
 */
void initialize(VM& vm);

// Waypoint class methods
Value waypointConstructor(const std::vector<Value>& args);
Value waypointGetPosition(const std::vector<Value>& args);
Value waypointGetAltitude(const std::vector<Value>& args);
Value waypointGetHeading(const std::vector<Value>& args);
Value waypointGetSpeed(const std::vector<Value>& args);
Value waypointToString(const std::vector<Value>& args);

// Path class methods
Value pathConstructor(const std::vector<Value>& args);
Value pathAddWaypoint(const std::vector<Value>& args);
Value pathRemoveWaypoint(const std::vector<Value>& args);
Value pathGetWaypoint(const std::vector<Value>& args);
Value pathGetWaypointCount(const std::vector<Value>& args);
Value pathGetLength(const std::vector<Value>& args);
Value pathGetDuration(const std::vector<Value>& args);
Value pathInterpolate(const std::vector<Value>& args);
Value pathToString(const std::vector<Value>& args);

// Guidance functions
Value calculateRoute(const std::vector<Value>& args);
Value planPath(const std::vector<Value>& args);
Value optimizePath(const std::vector<Value>& args);
Value avoidObstacles(const std::vector<Value>& args);
Value followPath(const std::vector<Value>& args);
Value calculateETA(const std::vector<Value>& args);
Value calculateBearing(const std::vector<Value>& args);
Value calculateDistance(const std::vector<Value>& args);
Value calculateGreatCircleDistance(const std::vector<Value>& args);
Value calculateGreatCircleBearing(const std::vector<Value>& args);
Value calculateGreatCircleDestination(const std::vector<Value>& args);
Value calculateGreatCircleIntermediate(const std::vector<Value>& args);
Value calculateRhumbLineDistance(const std::vector<Value>& args);
Value calculateRhumbLineBearing(const std::vector<Value>& args);
Value calculateRhumbLineDestination(const std::vector<Value>& args);
Value calculateRhumbLineIntermediate(const std::vector<Value>& args);

// Coordinate conversion functions
Value convertLatLonToECEF(const std::vector<Value>& args);
Value convertECEFToLatLon(const std::vector<Value>& args);
Value convertLatLonToUTM(const std::vector<Value>& args);
Value convertUTMToLatLon(const std::vector<Value>& args);
Value convertECEFToENU(const std::vector<Value>& args);
Value convertENUToECEF(const std::vector<Value>& args);
Value convertLatLonToENU(const std::vector<Value>& args);
Value convertENUToLatLon(const std::vector<Value>& args);

// Helper functions
ObjectPtr createWaypoint(const ObjectPtr& position, double altitude, double heading, double speed);
ObjectPtr createPath(const std::vector<ObjectPtr>& waypoints);
bool isWaypoint(const Value& value);
bool isPath(const Value& value);

// Constants
constexpr double EARTH_RADIUS = 6371000.0; // Earth radius in meters
constexpr double WGS84_A = 6378137.0; // WGS84 semi-major axis in meters
constexpr double WGS84_B = 6356752.314245; // WGS84 semi-minor axis in meters
constexpr double WGS84_F = 1.0 / 298.257223563; // WGS84 flattening
constexpr double WGS84_E2 = 0.00669437999014; // WGS84 eccentricity squared

} // namespace navigation
} // namespace stdlib
} // namespace astra

#endif // ASTRA_NAVIGATION_MODULE_H