#ifndef ASTRA_SIGNAL_MODULE_H
#define ASTRA_SIGNAL_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>

namespace astra {
namespace stdlib {
namespace signal {

/**
 * Initialize the Signal module in the VM
 */
void initialize(VM& vm);

// Signal class methods
Value signalConstructor(const std::vector<Value>& args);
Value signalFromArray(const std::vector<Value>& args);
Value signalGetSample(const std::vector<Value>& args);
Value signalSetSample(const std::vector<Value>& args);
Value signalGetLength(const std::vector<Value>& args);
Value signalGetSampleRate(const std::vector<Value>& args);
Value signalToArray(const std::vector<Value>& args);
Value signalToString(const std::vector<Value>& args);

// Filter class methods
Value filterConstructor(const std::vector<Value>& args);
Value filterApply(const std::vector<Value>& args);
Value filterReset(const std::vector<Value>& args);
Value filterGetCoefficients(const std::vector<Value>& args);
Value filterSetCoefficients(const std::vector<Value>& args);
Value filterToString(const std::vector<Value>& args);

// Signal processing functions
Value createLowPassFilter(const std::vector<Value>& args);
Value createHighPassFilter(const std::vector<Value>& args);
Value createBandPassFilter(const std::vector<Value>& args);
Value createBandStopFilter(const std::vector<Value>& args);
Value createMovingAverageFilter(const std::vector<Value>& args);
Value createKalmanFilter(const std::vector<Value>& args);
Value createMedianFilter(const std::vector<Value>& args);
Value createButterworthFilter(const std::vector<Value>& args);
Value createChebyshevFilter(const std::vector<Value>& args);
Value createEllipticFilter(const std::vector<Value>& args);
Value createBesselFilter(const std::vector<Value>& args);

// Signal analysis functions
Value calculateFFT(const std::vector<Value>& args);
Value calculateIFFT(const std::vector<Value>& args);
Value calculatePSD(const std::vector<Value>& args);
Value calculateSpectrogram(const std::vector<Value>& args);
Value calculateCorrelation(const std::vector<Value>& args);
Value calculateConvolution(const std::vector<Value>& args);
Value calculateDeconvolution(const std::vector<Value>& args);
Value calculateMean(const std::vector<Value>& args);
Value calculateVariance(const std::vector<Value>& args);
Value calculateStandardDeviation(const std::vector<Value>& args);
Value calculateRMS(const std::vector<Value>& args);
Value calculateSNR(const std::vector<Value>& args);
Value calculateTHD(const std::vector<Value>& args);
Value calculateEnvelope(const std::vector<Value>& args);
Value calculateZeroCrossings(const std::vector<Value>& args);
Value calculatePeaks(const std::vector<Value>& args);
Value calculateThreshold(const std::vector<Value>& args);

// Signal generation functions
Value generateSine(const std::vector<Value>& args);
Value generateSquare(const std::vector<Value>& args);
Value generateTriangle(const std::vector<Value>& args);
Value generateSawtooth(const std::vector<Value>& args);
Value generateNoise(const std::vector<Value>& args);
Value generateWhiteNoise(const std::vector<Value>& args);
Value generatePinkNoise(const std::vector<Value>& args);
Value generateBrownianNoise(const std::vector<Value>& args);
Value generateChirp(const std::vector<Value>& args);
Value generatePulse(const std::vector<Value>& args);
Value generateStep(const std::vector<Value>& args);
Value generateImpulse(const std::vector<Value>& args);

// Helper functions
ObjectPtr createSignal(const std::vector<double>& samples, double sampleRate);
ObjectPtr createFilter(const std::vector<double>& bCoefficients, const std::vector<double>& aCoefficients);
bool isSignal(const Value& value);
bool isFilter(const Value& value);

} // namespace signal
} // namespace stdlib
} // namespace astra

#endif // ASTRA_SIGNAL_MODULE_H