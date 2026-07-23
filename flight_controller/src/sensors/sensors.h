#ifndef SENSORS_H
#define SENSORS_H

void sensors_init(void);
void sensors_altitude_process(void);
float sensors_read_altitude(void);
void sensors_imu_process(void);

#endif  // SENSORS_H
