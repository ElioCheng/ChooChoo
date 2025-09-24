#ifndef MARKLIN_TRAIN_KINEMATICS_H
#define MARKLIN_TRAIN_KINEMATICS_H

#include "types.h"
#include "marklin/common/constants.h"

// ############################################################################
// # Fixed-Point Arithmetic Constants and Types
// ############################################################################

// All calculations use 64-bit integers for precision and overflow safety

// Time: measured in ticks
typedef i64 kinematic_time_t; // Time in ticks
typedef i64 kinematic_distance_t; // Distance in millimeters
typedef i64 kinematic_velocity_t; // Velocity in fixed-point mm/tick (scaled by VELOCITY_SCALE_FACTOR)
typedef i64 kinematic_accel_t; // Acceleration in fixed-point mm/tick² (scaled by ACCEL_SCALE_FACTOR)

// Fixed-point scaling for velocity and acceleration precision
// To change precision, modify DIGITS and update SCALE_FACTOR to 10^DIGITS
#define KINEMATIC_VELOCITY_SCALE_DIGITS 6 // Number of decimal places for velocity precision
#define KINEMATIC_VELOCITY_SCALE_FACTOR 100000000 // 10^8 for 8 decimal places
#define KINEMATIC_ACCEL_SCALE_DIGITS 6 // Number of decimal places for acceleration precision
#define KINEMATIC_ACCEL_SCALE_FACTOR 100000000 // 10^8 for 8 decimal places
#define KINEMATIC_TIME_SCALE 10 // 10ms per tick

// Speed level constants
#define KINEMATIC_MAX_SPEED_LEVEL 14
#define KINEMATIC_TOTAL_SPEED_LEVELS 28 // 2 velocities per level 1-13 + level 0 & 14
#define KINEMATIC_MAX_TRAINS ALL_POSSIBLE_TRAINS_COUNT

#define TRAIN_LENGTH_MM 200
#define SENSOR_POLLING_INTERVAL_MS 60 // Sensor polling interval

// EWMA constants
#define KINEMATIC_EWMA_ALPHA_SHIFT 8 // Alpha = 1/256 for EWMA
#define KINEMATIC_EWMA_ALPHA_DENOM (1 << KINEMATIC_EWMA_ALPHA_SHIFT)

// ############################################################################
// # Fixed-Point Math Utility Functions
// ############################################################################

static inline i64 kinematic_safe_multiply(i64 a, i64 b)
{
	if (a == 0 || b == 0)
		return 0;

	// if a * b would overflow, then |a| > max_i64 / |b|
	const i64 max_val = 0x7FFFFFFFFFFFFFFFLL;
	if ((a > 0 && b > 0 && a > max_val / b) || (a < 0 && b < 0 && a < max_val / b) ||
	    (a > 0 && b < 0 && b < -max_val / a) || (a < 0 && b > 0 && a < -max_val / b)) {
		// Would overflow, return max safe value
		return (a > 0) == (b > 0) ? max_val : -max_val;
	}

	return a * b;
}

static inline i64 kinematic_safe_divide_scaled(i64 a, i64 c, i64 b)
{
	if (b == 0)
		return 0;

	// (a * c) / b = a * (c / b) when c % b == 0
	if (c % b == 0) {
		return kinematic_safe_multiply(a, c / b);
	}

	i64 product = kinematic_safe_multiply(a, c);
	return product / b;
}

// Create fixed-point velocity value from scaled integer
static inline kinematic_velocity_t kinematic_velocity_from_scaled(i64 scaled_value)
{
	return scaled_value;
}

// Calculate velocity from distance and time with fixed-point precision
static inline kinematic_velocity_t kinematic_velocity(kinematic_distance_t distance, kinematic_time_t time)
{
	if (time == 0)
		return 0;

	// Scale distance before division to maintain precision
	// velocity = (distance * SCALE_FACTOR) / time
	return kinematic_safe_divide_scaled(distance, KINEMATIC_VELOCITY_SCALE_FACTOR, time);
}

static inline kinematic_accel_t kinematic_acceleration(kinematic_velocity_t v1, kinematic_velocity_t v2,
						       kinematic_time_t time)
{
	if (time == 0)
		return 0;
	// v1 and v2 are already scaled by VELOCITY_SCALE_FACTOR
	// (v2 - v1) is scaled by VELOCITY_SCALE_FACTOR
	// We need to scale by ACCEL_SCALE_FACTOR/VELOCITY_SCALE_FACTOR to get proper acceleration scaling
	// Since both scale factors are the same (10^6), we just divide by time
	return (v2 - v1) / time;
}

static inline kinematic_distance_t kinematic_distance_from_velocity(kinematic_velocity_t velocity,
								    kinematic_time_t time)
{
	// velocity is scaled by SCALE_FACTOR, so divide result by SCALE_FACTOR
	// distance = (velocity * time) / SCALE_FACTOR
	return kinematic_safe_divide_scaled(velocity, time, KINEMATIC_VELOCITY_SCALE_FACTOR);
}

// Calculate distance from acceleration: v1*t + 0.5*a*t²
static inline kinematic_distance_t kinematic_distance_from_acceleration(kinematic_velocity_t v1,
									kinematic_accel_t accel, kinematic_time_t time)
{
	// v1 is scaled by VELOCITY_SCALE_FACTOR, so divide by SCALE_FACTOR after multiplication
	kinematic_distance_t linear_part = kinematic_safe_divide_scaled(v1, time, KINEMATIC_VELOCITY_SCALE_FACTOR);

	// accel is scaled by ACCEL_SCALE_FACTOR, and we have t²
	// distance = 0.5 * a * t² = (a * t²) / (2 * ACCEL_SCALE_FACTOR)
	kinematic_distance_t time_squared = kinematic_safe_multiply(time, time);
	kinematic_distance_t accel_distance = kinematic_safe_multiply(accel, time_squared);
	kinematic_distance_t accel_part = accel_distance / (2 * KINEMATIC_ACCEL_SCALE_FACTOR);

	return linear_part + accel_part;
}

// Calculate time to reach distance with constant velocity
static inline kinematic_time_t kinematic_time_for_distance(kinematic_distance_t distance, kinematic_velocity_t velocity)
{
	if (velocity == 0)
		return 0;
	// velocity is scaled, so multiply distance by SCALE_FACTOR before division
	// time = (distance * SCALE_FACTOR) / velocity
	return kinematic_safe_divide_scaled(distance, KINEMATIC_VELOCITY_SCALE_FACTOR, velocity);
}

// Average velocity during acceleration: (v1 + v2) / 2
static inline kinematic_velocity_t kinematic_average_velocity(kinematic_velocity_t v1, kinematic_velocity_t v2)
{
	return (v1 + v2) / 2;
}

// ############################################################################
// # EWMA (Exponentially Weighted Moving Average) Functions
// ############################################################################

// Update EWMA estimate: current = current * (1 - α) + new_sample * α
static inline i64 kinematic_ewma_update(i64 current_estimate, i64 new_sample)
{
	i64 weighted_current = (current_estimate * (KINEMATIC_EWMA_ALPHA_DENOM - 1)) >> KINEMATIC_EWMA_ALPHA_SHIFT;
	i64 weighted_sample = new_sample >> KINEMATIC_EWMA_ALPHA_SHIFT;
	return weighted_current + weighted_sample;
}

// ############################################################################
// # Conversion Utilities
// ############################################################################

// Convert milliseconds to ticks (10ms granularity)
static inline kinematic_time_t kinematic_ms_to_ticks(u32 milliseconds)
{
	return milliseconds / KINEMATIC_TIME_SCALE;
}

// Convert ticks to milliseconds
static inline u32 kinematic_ticks_to_ms(kinematic_time_t ticks)
{
	return (u32)(ticks * KINEMATIC_TIME_SCALE);
}

// Convert scaled mm/tick to mm/s
static inline i64 kinematic_velocity_to_mm_per_second(kinematic_velocity_t velocity_mm_per_tick)
{
	// velocity_mm_per_tick is scaled, so divide by scale factor first, then convert to mm/s
	// mm/s = (velocity / SCALE_FACTOR) * (1000ms/s / 10ms/tick) = (velocity / SCALE_FACTOR) * 100
	return kinematic_safe_divide_scaled(velocity_mm_per_tick, (1000 / KINEMATIC_TIME_SCALE),
					    KINEMATIC_VELOCITY_SCALE_FACTOR);
}

// Convert mm/s to scaled mm/tick
static inline kinematic_velocity_t kinematic_velocity_from_mm_per_second(i64 velocity_mm_per_second)
{
	// mm/tick = mm/s / (1000ms/s / 10ms/tick) = mm/s / 100, then scale by SCALE_FACTOR
	return (velocity_mm_per_second * KINEMATIC_VELOCITY_SCALE_FACTOR) / (1000 / KINEMATIC_TIME_SCALE);
}

// Get integer part of scaled velocity (mm/tick)
static inline i64 kinematic_velocity_integer_part(kinematic_velocity_t velocity)
{
	return velocity / KINEMATIC_VELOCITY_SCALE_FACTOR;
}

// Get fractional part of scaled velocity (scaled by SCALE_FACTOR)
static inline i64 kinematic_velocity_fractional_part(kinematic_velocity_t velocity)
{
	return velocity % KINEMATIC_VELOCITY_SCALE_FACTOR;
}

// Format velocity for display: returns mm/tick with configurable decimal places
// Note: This is for logging/debugging only - avoid in time-critical code
// Returns both integer and fractional parts for use with logging functions
static inline void kinematic_velocity_split(kinematic_velocity_t velocity, i64 *integer_part, i64 *fractional_part)
{
	*integer_part = kinematic_velocity_integer_part(velocity);
	*fractional_part = kinematic_velocity_fractional_part(velocity);

	// Handle negative numbers
	if (velocity < 0) {
		*integer_part = -*integer_part;
		*fractional_part = -*fractional_part;
	}
}

// Create fixed-point acceleration value from scaled integer
static inline kinematic_accel_t kinematic_accel_from_scaled(i64 scaled_value)
{
	return scaled_value;
}

// Get integer part of scaled acceleration (mm/tick²)
static inline i64 kinematic_accel_integer_part(kinematic_accel_t accel)
{
	return accel / KINEMATIC_ACCEL_SCALE_FACTOR;
}

// Get fractional part of scaled acceleration (scaled by SCALE_FACTOR)
static inline i64 kinematic_accel_fractional_part(kinematic_accel_t accel)
{
	return accel % KINEMATIC_ACCEL_SCALE_FACTOR;
}

// Format acceleration for display: returns mm/tick² with configurable decimal places
// Note: This is for logging/debugging only - avoid in time-critical code
// Returns both integer and fractional parts for use with logging functions
static inline void kinematic_accel_split(kinematic_accel_t accel, i64 *integer_part, i64 *fractional_part)
{
	*integer_part = kinematic_accel_integer_part(accel);
	*fractional_part = kinematic_accel_fractional_part(accel);

	// Handle negative numbers
	if (accel < 0) {
		*integer_part = -*integer_part;
		*fractional_part = -*fractional_part;
	}
}

// ############################################################################
// # Speed Level Mapping
// ############################################################################

// Map speed level and direction to velocity table index
// Speed 0: index 0
// Speed 1-13: indices 1-26 (2 per speed: low->high, high->low)
// Speed 14: index 27
static inline u8 kinematic_speed_to_index(u8 speed_level, bool from_higher_speed)
{
	if (speed_level == 0)
		return 0;
	if (speed_level == 14)
		return 27;
	if (speed_level >= 1 && speed_level <= 13) {
		return (speed_level - 1) * 2 + (from_higher_speed ? 2 : 1);
	}
	return 0; // Invalid speed level
}

// Get speed level from velocity table index
static inline u8 kinematic_index_to_speed(u8 index, bool *from_higher_speed)
{
	if (index == 0) {
		if (from_higher_speed)
			*from_higher_speed = false;
		return 0;
	}
	if (index == 27) {
		if (from_higher_speed)
			*from_higher_speed = false;
		return 14;
	}
	if (index >= 1 && index <= 26) {
		u8 speed = ((index - 1) / 2) + 1;
		if (from_higher_speed)
			*from_higher_speed = ((index - 1) % 2) == 1;
		return speed;
	}
	if (from_higher_speed)
		*from_higher_speed = false;
	return 0;
}

// ############################################################################
// # Track Resistance Utilities
// ############################################################################

// Track resistance coefficient constants
#define RESISTANCE_SCALE_FACTOR 1000 // Fixed-point scale: 1000 = 1.0 resistance
#define RESISTANCE_DEFAULT 1000 // Default resistance coefficient = 1.0

// Apply resistance coefficient to distance calculation
// distance: raw distance in mm
// resistance_coefficient: fixed-point resistance (scale 1000)
// Returns: effective distance accounting for resistance
static inline kinematic_distance_t kinematic_apply_resistance_to_distance(kinematic_distance_t distance,
									  u32 resistance_coefficient)
{
	if (resistance_coefficient == 0) {
		return distance; // Avoid division by zero
	}

	// effective_distance = raw_distance * resistance_coefficient / SCALE_FACTOR
	// Use safe multiplication to avoid overflow
	return kinematic_safe_divide_scaled(distance, resistance_coefficient, RESISTANCE_SCALE_FACTOR);
}

// Convert fixed-point resistance coefficient to scaled value
static inline u32 kinematic_resistance_from_fixed(u32 fixed_value)
{
	return fixed_value;
}

// Create resistance coefficient from integer and fractional parts
// int_part: integer part (e.g., 1 for 1.x)
// frac_part: fractional part in thousandths (e.g., 500 for 0.5)
static inline u32 kinematic_resistance_from_parts(u16 int_part, u16 frac_part)
{
	if (frac_part >= RESISTANCE_SCALE_FACTOR) {
		frac_part = RESISTANCE_SCALE_FACTOR - 1; // Cap fractional part
	}
	return (u32)int_part * RESISTANCE_SCALE_FACTOR + frac_part;
}

#endif /* MARKLIN_TRAIN_KINEMATICS_H */
