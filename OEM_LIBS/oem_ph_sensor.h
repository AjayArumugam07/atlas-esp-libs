/*
 * ezo_ph_sensor.h
 *
 *  Created on: July 1, 2021
 *      Author: Karthick Siva. 
 */

#ifndef PH_SENSOR_H
#define PH_SENSOR_H

#include <esp_err.h>
#include "i2cdev.h"
#define PH_ADDR_BASE 0x65

#ifdef __cplusplus
extern "C" {
#endif

typedef i2c_dev_t ph_sensor_t;

/**
 * @brief Setup pH I2C communication
 * @param dev I2C device descriptor
 * @param port I2C port
 * @param addr I2C address
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return ESP_OK to indicate success
 */
esp_err_t ph_init(ph_sensor_t *dev, i2c_port_t port, uint8_t addr, int8_t sda_gpio, int8_t scl_gpio);

/**
 * @brief Wake up pH sensor
 * @param dev I2C device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t activate_ph(ph_sensor_t *dev);

/**
 * @brief Hibernate pH sensor
 * @param dev I2C device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t hibernate_ph(ph_sensor_t *dev);

/**
 * @brief Calibrate pH sensor
 * @param dev I2C device descriptor
 * @param temperature This value is required for temperature compensation
 * @return ESP_OK to indicate success
 */
esp_err_t calibrate_ph(ph_sensor_t *dev, float temperature);

/**
 * @brief Clear Calibration settings
 * @param dev I2C device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t clear_calibration_ph(ph_sensor_t *dev);

/**
 * @brief Read pH with temperature compensation
 * @param dev I2C device descriptor
 * @param temperature This value is required for temperature compensation
 * @param ph pointer to ph variable
 * @return ESP_OK to indicate success
 */
esp_err_t read_ph_with_temperature(ph_sensor_t *dev, float temperature, float *ph);

/**
 * @brief Read pH without temperature compensation
 * @param dev I2C device descriptor
 * @param ph pointer to ph variable
 * @return ESP_OK to indicate success
 */
esp_err_t read_ph(ph_sensor_t *dev, float *ph);

esp_err_t get_firmware_ph(ph_sensor_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* PH_SENSOR_H */


