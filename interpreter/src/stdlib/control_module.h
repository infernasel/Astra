#ifndef ASTRA_CONTROL_MODULE_H
#define ASTRA_CONTROL_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace control {

/**
 * Initialize the Control module in the VM
 */
void initialize(VM& vm);

// PID controller class methods
Value pidConstructor(const std::vector<Value>& args);
Value pidUpdate(const std::vector<Value>& args);
Value pidReset(const std::vector<Value>& args);
Value pidSetGains(const std::vector<Value>& args);
Value pidSetSetpoint(const std::vector<Value>& args);
Value pidSetOutputLimits(const std::vector<Value>& args);
Value pidGetProportionalTerm(const std::vector<Value>& args);
Value pidGetIntegralTerm(const std::vector<Value>& args);
Value pidGetDerivativeTerm(const std::vector<Value>& args);
Value pidToString(const std::vector<Value>& args);

// Kalman filter class methods
Value kalmanConstructor(const std::vector<Value>& args);
Value kalmanPredict(const std::vector<Value>& args);
Value kalmanUpdate(const std::vector<Value>& args);
Value kalmanGetState(const std::vector<Value>& args);
Value kalmanGetCovariance(const std::vector<Value>& args);
Value kalmanSetState(const std::vector<Value>& args);
Value kalmanSetCovariance(const std::vector<Value>& args);
Value kalmanToString(const std::vector<Value>& args);

// Attitude controller class methods
Value attitudeControllerConstructor(const std::vector<Value>& args);
Value attitudeControllerUpdate(const std::vector<Value>& args);
Value attitudeControllerSetTarget(const std::vector<Value>& args);
Value attitudeControllerGetOutput(const std::vector<Value>& args);
Value attitudeControllerToString(const std::vector<Value>& args);

// Trajectory controller class methods
Value trajectoryControllerConstructor(const std::vector<Value>& args);
Value trajectoryControllerUpdate(const std::vector<Value>& args);
Value trajectoryControllerSetPath(const std::vector<Value>& args);
Value trajectoryControllerGetOutput(const std::vector<Value>& args);
Value trajectoryControllerToString(const std::vector<Value>& args);

// Control system functions
Value calculateAttitude(const std::vector<Value>& args);
Value calculateThrust(const std::vector<Value>& args);
Value calculateTorque(const std::vector<Value>& args);
Value calculateControlAllocation(const std::vector<Value>& args);
Value calculateFeedback(const std::vector<Value>& args);
Value calculateFeedforward(const std::vector<Value>& args);
Value calculateLQR(const std::vector<Value>& args);
Value calculateMPC(const std::vector<Value>& args);
Value calculateRateLimit(const std::vector<Value>& args);
Value calculateDeadband(const std::vector<Value>& args);
Value calculateSaturation(const std::vector<Value>& args);
Value calculateAntiWindup(const std::vector<Value>& args);

// Helper functions
ObjectPtr createPIDController(double kp, double ki, double kd, double setpoint);
ObjectPtr createKalmanFilter(const ObjectPtr& initialState, const ObjectPtr& initialCovariance);
ObjectPtr createAttitudeController(const ObjectPtr& pidRoll, const ObjectPtr& pidPitch, const ObjectPtr& pidYaw);
ObjectPtr createTrajectoryController(const ObjectPtr& pidX, const ObjectPtr& pidY, const ObjectPtr& pidZ);
bool isPIDController(const Value& value);
bool isKalmanFilter(const Value& value);
bool isAttitudeController(const Value& value);
bool isTrajectoryController(const Value& value);

} // namespace control
} // namespace stdlib
} // namespace astra

#endif // ASTRA_CONTROL_MODULE_H