#include "i2c.h"

#include <errno.h>
#include <stdio.h>

#include <zephyr/drivers/i2c.h>

const struct device *const i2c_driver =
    DEVICE_DT_GET(DT_ALIAS(i2c_interface));

void i2c_master_init(void)
{
    if (!device_is_ready(i2c_driver))
    {
        printf("I2C device not ready\n");
    }
}

enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet)
{
    int ret = i2c_write(i2c_driver, packet->data, packet->data_length,
                        packet->address);
    if (ret == 0)
    {
        return STATUS_OK;
    }
    if (ret == -EIO)
    {
        return STATUS_ERR_OVERFLOW;
    }
    return STATUS_ERR_TIMEOUT;
}

enum status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet)
{
    struct i2c_msg msg = {
        .buf = packet->data,
        .len = packet->data_length,
        .flags = I2C_MSG_WRITE,
    };

    int ret = i2c_transfer(i2c_driver, &msg, 1, packet->address);
    if (ret == 0)
    {
        return STATUS_OK;
    }
    if (ret == -EIO)
    {
        return STATUS_ERR_OVERFLOW;
    }
    return STATUS_ERR_TIMEOUT;
}

enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet)
{
    int ret = i2c_read(i2c_driver, packet->data, packet->data_length,
                       packet->address);
    if (ret == 0)
    {
        return STATUS_OK;
    }
    if (ret == -EIO)
    {
        return STATUS_ERR_OVERFLOW;
    }
    return STATUS_ERR_TIMEOUT;
}
