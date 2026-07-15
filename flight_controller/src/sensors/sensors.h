#ifndef SENSORS_H
#define SENSORS_H

/**
 * @brief Initialize all sensors (e.g. the MS5611 barometer).
 *
 * Meant to be called once at startup, before the sensors thread starts.
 */
void sensors_init(void);

/**
 * @brief Read the altitude source (real sensor, or the UART in simulation mode)
 *        and publish the latest value.
 *
 * Meant to be called periodically by the sensors thread.
 */
void sensors_altitude_process(void);

/**
 * @brief Get the latest altitude value published by sensors_altitude_process().
 *
 * Meant to be called by the control thread.
 *
 * @return int latest altitude value
 */
int sensors_read_altitude(void);

/**
 * @brief Read the VL53L1X time-of-flight sensor and publish the latest distance.
 *
 * Meant to be called periodically by the sensors thread.
 */
void sensors_distance_process(void);

/**
 * @brief Get the latest distance value (millimeters) published by
 *        sensors_distance_process().
 *
 * Meant to be called by the control thread.
 *
 * @return int latest distance value in millimeters
 */
int sensors_read_distance(void);

#endif // SENSORS_H
