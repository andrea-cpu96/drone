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
 * @return float latest altitude value in meters
 */
float sensors_read_altitude(void);

/**
 * @brief Read the BMI088 IMU (accelerometer + gyroscope) and publish the latest
 *        values. Does nothing in simulation mode, where there is no IMU source.
 *
 * Meant to be called periodically by the sensors thread.
 */
void sensors_imu_process(void);

#endif  // SENSORS_H
