#ifndef IMU_BUS_H
#define IMU_BUS_H

#include <stdint.h>

void imu_bus_init(void);
int imu_bus_write(unsigned char slave_addr,
                  unsigned char reg_addr,
                  unsigned char length,
                  unsigned char const *data);
int imu_bus_read(unsigned char slave_addr,
                 unsigned char reg_addr,
                 unsigned char length,
                 unsigned char *data);
void imu_bus_delay_ms(unsigned long ms);
void imu_bus_get_ms(unsigned long *time_ms);

#endif
