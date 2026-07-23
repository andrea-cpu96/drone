#include "i2c.h"

#include "log.h"

#include <errno.h>

#include <zephyr/drivers/i2c.h>

const struct device *const i2c_driver =
    DEVICE_DT_GET(DT_ALIAS(i2c_interface));

/**
 * @brief Verify the I2C bus controller and the given devices are ready.
 *
 * @param devices
 * @param count
 * @return bool
 */
bool i2c_master_init(const struct device *const *devices, size_t count)
{
    if (!device_is_ready(i2c_driver))
    {
        log_print("I2C device not ready\n");
        return false;
    }

    for (size_t i = 0; i < count; i++)
    {
        if (!device_is_ready(devices[i]))
        {
            log_print("I2C device not ready\n");
            return false;
        }
    }

    log_print("I2C master ready\n");
    return true;
}

/**
 * @brief Write an I2C packet and wait for the transfer to complete.
 *
 * @param packet
 * @return enum status_code
 */
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

/**
 * @brief Write an I2C packet without issuing a stop condition.
 *
 * @param packet
 * @return enum status_code
 */
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

/**
 * @brief Read an I2C packet and wait for the transfer to complete.
 *
 * @param packet
 * @return enum status_code
 */
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
