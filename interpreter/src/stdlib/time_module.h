#ifndef ASTRA_TIME_MODULE_H
#define ASTRA_TIME_MODULE_H

#include "../vm/vm.h"
#include "../vm/value.h"
#include <vector>
#include <chrono>
#include <ctime>

namespace astra {
namespace stdlib {
namespace time {

/**
 * Initialize the Time module in the VM
 */
void initialize(VM& vm);

// Time class methods
Value timeConstructor(const std::vector<Value>& args);
Value timeNow(const std::vector<Value>& args);
Value timeFromUnixTimestamp(const std::vector<Value>& args);
Value timeFromString(const std::vector<Value>& args);
Value timeFromYMDHMS(const std::vector<Value>& args);
Value timeGetYear(const std::vector<Value>& args);
Value timeGetMonth(const std::vector<Value>& args);
Value timeGetDay(const std::vector<Value>& args);
Value timeGetHour(const std::vector<Value>& args);
Value timeGetMinute(const std::vector<Value>& args);
Value timeGetSecond(const std::vector<Value>& args);
Value timeGetMillisecond(const std::vector<Value>& args);
Value timeGetMicrosecond(const std::vector<Value>& args);
Value timeGetNanosecond(const std::vector<Value>& args);
Value timeGetUnixTimestamp(const std::vector<Value>& args);
Value timeGetDayOfWeek(const std::vector<Value>& args);
Value timeGetDayOfYear(const std::vector<Value>& args);
Value timeGetWeekOfYear(const std::vector<Value>& args);
Value timeFormat(const std::vector<Value>& args);
Value timeToString(const std::vector<Value>& args);
Value timeAdd(const std::vector<Value>& args);
Value timeSubtract(const std::vector<Value>& args);
Value timeDifference(const std::vector<Value>& args);
Value timeEquals(const std::vector<Value>& args);
Value timeLessThan(const std::vector<Value>& args);
Value timeGreaterThan(const std::vector<Value>& args);
Value timeLessThanOrEqual(const std::vector<Value>& args);
Value timeGreaterThanOrEqual(const std::vector<Value>& args);

// Duration class methods
Value durationConstructor(const std::vector<Value>& args);
Value durationFromNanoseconds(const std::vector<Value>& args);
Value durationFromMicroseconds(const std::vector<Value>& args);
Value durationFromMilliseconds(const std::vector<Value>& args);
Value durationFromSeconds(const std::vector<Value>& args);
Value durationFromMinutes(const std::vector<Value>& args);
Value durationFromHours(const std::vector<Value>& args);
Value durationFromDays(const std::vector<Value>& args);
Value durationFromWeeks(const std::vector<Value>& args);
Value durationGetNanoseconds(const std::vector<Value>& args);
Value durationGetMicroseconds(const std::vector<Value>& args);
Value durationGetMilliseconds(const std::vector<Value>& args);
Value durationGetSeconds(const std::vector<Value>& args);
Value durationGetMinutes(const std::vector<Value>& args);
Value durationGetHours(const std::vector<Value>& args);
Value durationGetDays(const std::vector<Value>& args);
Value durationGetWeeks(const std::vector<Value>& args);
Value durationAdd(const std::vector<Value>& args);
Value durationSubtract(const std::vector<Value>& args);
Value durationMultiply(const std::vector<Value>& args);
Value durationDivide(const std::vector<Value>& args);
Value durationToString(const std::vector<Value>& args);
Value durationEquals(const std::vector<Value>& args);
Value durationLessThan(const std::vector<Value>& args);
Value durationGreaterThan(const std::vector<Value>& args);
Value durationLessThanOrEqual(const std::vector<Value>& args);
Value durationGreaterThanOrEqual(const std::vector<Value>& args);

// Timer class methods
Value timerConstructor(const std::vector<Value>& args);
Value timerStart(const std::vector<Value>& args);
Value timerStop(const std::vector<Value>& args);
Value timerReset(const std::vector<Value>& args);
Value timerIsRunning(const std::vector<Value>& args);
Value timerGetElapsed(const std::vector<Value>& args);
Value timerGetElapsedNanoseconds(const std::vector<Value>& args);
Value timerGetElapsedMicroseconds(const std::vector<Value>& args);
Value timerGetElapsedMilliseconds(const std::vector<Value>& args);
Value timerGetElapsedSeconds(const std::vector<Value>& args);
Value timerToString(const std::vector<Value>& args);

// Time utility functions
Value sleep(const std::vector<Value>& args);
Value nanoseconds(const std::vector<Value>& args);
Value microseconds(const std::vector<Value>& args);
Value milliseconds(const std::vector<Value>& args);
Value seconds(const std::vector<Value>& args);
Value minutes(const std::vector<Value>& args);
Value hours(const std::vector<Value>& args);
Value days(const std::vector<Value>& args);
Value weeks(const std::vector<Value>& args);
Value now(const std::vector<Value>& args);
Value clock(const std::vector<Value>& args);
Value formatTime(const std::vector<Value>& args);
Value parseTime(const std::vector<Value>& args);
Value isLeapYear(const std::vector<Value>& args);
Value getDaysInMonth(const std::vector<Value>& args);
Value getTimeZoneOffset(const std::vector<Value>& args);
Value convertTimeZone(const std::vector<Value>& args);
Value getUTCTime(const std::vector<Value>& args);
Value getLocalTime(const std::vector<Value>& args);

// Helper functions
ObjectPtr createTime(const std::chrono::system_clock::time_point& timePoint);
ObjectPtr createDuration(const std::chrono::nanoseconds& duration);
ObjectPtr createTimer();
bool isTime(const Value& value);
bool isDuration(const Value& value);
bool isTimer(const Value& value);

} // namespace time
} // namespace stdlib
} // namespace astra

#endif // ASTRA_TIME_MODULE_H