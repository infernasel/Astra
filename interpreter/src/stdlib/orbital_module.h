#ifndef ASTRA_ORBITAL_MODULE_H
#define ASTRA_ORBITAL_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace orbital {

/**
 * Initialize the Orbital module in the VM
 */
void initialize(VM& vm);

// Orbit class methods
Value orbitConstructor(const std::vector<Value>& args);
Value orbitFromStateVectors(const std::vector<Value>& args);
Value orbitFromElements(const std::vector<Value>& args);
Value orbitGetPosition(const std::vector<Value>& args);
Value orbitGetVelocity(const std::vector<Value>& args);
Value orbitGetPeriod(const std::vector<Value>& args);
Value orbitGetSemiMajorAxis(const std::vector<Value>& args);
Value orbitGetEccentricity(const std::vector<Value>& args);
Value orbitGetInclination(const std::vector<Value>& args);
Value orbitGetRaan(const std::vector<Value>& args);
Value orbitGetArgOfPerigee(const std::vector<Value>& args);
Value orbitGetTrueAnomaly(const std::vector<Value>& args);
Value orbitGetMeanAnomaly(const std::vector<Value>& args);
Value orbitGetEccentricAnomaly(const std::vector<Value>& args);
Value orbitGetApoapsis(const std::vector<Value>& args);
Value orbitGetPeriapsis(const std::vector<Value>& args);
Value orbitPropagate(const std::vector<Value>& args);
Value orbitToString(const std::vector<Value>& args);

// Celestial body methods
Value celestialBodyConstructor(const std::vector<Value>& args);
Value celestialBodyGetMass(const std::vector<Value>& args);
Value celestialBodyGetRadius(const std::vector<Value>& args);
Value celestialBodyGetGravitationalParameter(const std::vector<Value>& args);
Value celestialBodyGetEscapeVelocity(const std::vector<Value>& args);
Value celestialBodyGetSphereOfInfluence(const std::vector<Value>& args);
Value celestialBodyToString(const std::vector<Value>& args);

// Orbital mechanics functions
Value calculateOrbit(const std::vector<Value>& args);
Value propagateOrbit(const std::vector<Value>& args);
Value calculateHohmannTransfer(const std::vector<Value>& args);
Value calculateBiEllipticTransfer(const std::vector<Value>& args);
Value calculatePlaneChange(const std::vector<Value>& args);
Value calculatePhaseAngle(const std::vector<Value>& args);
Value calculateRendezvous(const std::vector<Value>& args);
Value calculateLambertSolution(const std::vector<Value>& args);
Value calculateGravityAssist(const std::vector<Value>& args);
Value calculateOrbitalElements(const std::vector<Value>& args);
Value calculateStateVectors(const std::vector<Value>& args);
Value calculateVisViva(const std::vector<Value>& args);
Value calculateTimeOfFlight(const std::vector<Value>& args);
Value solveKeplersEquation(const std::vector<Value>& args);
Value calculateTrueAnomaly(const std::vector<Value>& args);
Value calculateEccentricAnomaly(const std::vector<Value>& args);
Value calculateMeanAnomaly(const std::vector<Value>& args);

// Helper functions
ObjectPtr createOrbit(const ObjectPtr& centralBody, double semiMajorAxis, double eccentricity, 
                     double inclination, double raan, double argOfPerigee, double trueAnomaly);
ObjectPtr createCelestialBody(const std::string& name, double mass, double radius);
bool isOrbit(const Value& value);
bool isCelestialBody(const Value& value);

// Constants
constexpr double G = 6.67430e-11; // Gravitational constant (m^3 kg^-1 s^-2)

} // namespace orbital
} // namespace stdlib
} // namespace astra

#endif // ASTRA_ORBITAL_MODULE_H