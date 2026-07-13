#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#include <zephyr/device.h>

enum i2c_transfer_direction {
	I2C_TRANSFER_WRITE = 0,
	I2C_TRANSFER_READ  = 1,
};

enum status_code {
	STATUS_OK           = 0x00,
	STATUS_ERR_OVERFLOW = 0x01,
	STATUS_ERR_TIMEOUT  = 0x02,
};

struct i2c_master_packet {
	/* Address to slave device */
	uint16_t address;
	/* Length of data array */
	uint16_t data_length;
	/* Data array containing all data to be transferred */
	uint8_t *data;
};

void i2c_master_init(void);
enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet);
enum status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet);
enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet);

#endif /* I2C_H */
