/*
 * ezo_ph_sensor.c
 *
 *  Created on: July 1, 2021
 *      Author: Karthick Siva. 
 */

#include "oem_ph_sensor.h"
#include <esp_log.h>
#include <esp_err.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


/* MACRO for checkng argument paramters */
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
/* I2C Protocol Speed Parameter (10-100 kHz for OEM Device) */
#define I2C_FREQ_HZ 10000
/* MACRO for calibration function*/
#define STABILIZATION_ACCURACY 0.002
/* MACRO for calibration function */
#define STABILIZATION_COUNT_MAX 10
/* Debugging Tag for PH sensor */
static const char *TAG = "Atlas PH Sensor";

esp_err_t ph_init(ph_sensor_t *dev, i2c_port_t port, uint8_t addr, int8_t sda_gpio, int8_t scl_gpio) {
	// Check Arguments
    CHECK_ARG(dev);
    if (addr < PH_ADDR_BASE || addr > PH_ADDR_BASE + 7) {
        ESP_LOGE(TAG, "Invalid device address: 0x%02x", addr);
        return ESP_ERR_INVALID_ARG;
    }

    // Setup I2C settings
    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;

    return i2c_dev_create_mutex(dev);
}

esp_err_t get_firmware_ph(ph_sensor_t *dev) { 
    char data = 0x00; 
    char reg = 0x01; 
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &reg, sizeof(reg)));
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &data, sizeof(data)));
    I2C_DEV_GIVE_MUTEX(dev);
    printf("PH Firmware Version: %d\n", data);
    return ESP_OK; 
}

esp_err_t activate_ph(ph_sensor_t *dev) {
	// Write 0x01 to register 0x06 for activating ph sensor // 
	char data = 0x01; 
	char reg = 0x06; 
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &reg, sizeof(reg), &data, sizeof(data)));
    I2C_DEV_GIVE_MUTEX(dev);
	vTaskDelay(pdMS_TO_TICKS(1000));
	return ESP_OK; 
}

esp_err_t hibernate_ph(ph_sensor_t *dev) {
	//Write 0x00 to register 0x06 for placing ph sensor in hibernation mode // 
	char data = 0x00; 
	char reg = 0x06; 
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &reg, sizeof(reg), &data, sizeof(data)));
    I2C_DEV_GIVE_MUTEX(dev);
	vTaskDelay(pdMS_TO_TICKS(1000));
	return ESP_OK; 
}

esp_err_t calibrate_ph(ph_sensor_t *dev, float temperature){
	uint8_t count = 0;

	float ph = 0;
	float ph_min = 0;
	float ph_max = 0;

	// Keep restarting until 10 consecutive ph values are within stabilization accuracy range
	count = 0; 
	while(count < STABILIZATION_COUNT_MAX){
		esp_err_t err = read_ph_with_temperature(dev, temperature, &ph);	// read ph with temperature
		ESP_LOGI(TAG, "ph: %f", ph);
		if (err == ESP_OK) {	// Proceed if ph sensor responds with success code
			if(count == 0) {	// If first reading, then calculate stabilization range
				ph_min = ph * (1 - STABILIZATION_ACCURACY);
				ph_max = ph * (1 + STABILIZATION_ACCURACY);
				ESP_LOGI(TAG, "min ph: %f, max ph: %f", ph_min, ph_max);
				count++;
			} else {
				if(ph >= ph_min && ph <= ph_max){	// increment count if ph is within range
					count++;
				} else {
					count = 0;	// reset count to zero if ph is not within range
				}
			}
		} else {
			ESP_LOGI(TAG, "response code: %d", err);
		}
	}

	// Identify and create calibration command
	char calib_point = 0; 
	unsigned char msb = 0x00; 
	unsigned char high_byte = 0x00; 
	unsigned char low_byte = 0x00; 
	unsigned char lsb = 0x00; 
	 if(ph >= 2.5 && ph < 5.5) {
		//4.0 solution // 
		low_byte = 0x0F; 
		lsb = 0xA0;
		calib_point = 2; 
		ESP_LOGI(TAG, "4.0 solution identified");
	} else if (ph >= 5.5 && ph <= 8.5) {
		//7.0 solution //
		low_byte = 0x1B; 
		lsb = 0x58;  
		calib_point = 3; 
		ESP_LOGI(TAG, "7.0 solution identified");
	} else if (ph > 8.5 && ph <= 11.5) {
		//10.0 solution //
		low_byte = 0x27; 
		lsb = 0x10; 
		calib_point = 4; 
		ESP_LOGI(TAG, "10.0 solution identified");
	} else {
		ESP_LOGE(TAG, "calibration solution not identified, ph is lower than 2.5 or greater than 11.5");
		return ESP_FAIL;
	}

	// Send Calibration Command to EZO Sensor by sending each byte from msb to lsb
	char msb_reg = 0x08;
	char high_reg = 0x09; 
	char low_reg = 0x0A; 
	char lsb_reg = 0x0B; 
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &msb_reg, sizeof(msb_reg), &msb, sizeof(msb)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &high_reg, sizeof(high_reg), &high_byte, sizeof(high_byte)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &low_reg, sizeof(low_reg), &low_byte, sizeof(low_byte)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &lsb_reg, sizeof(lsb_reg), &lsb, sizeof(lsb)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Calibration request Register//
	char calib_req_reg = 0x0C; 
	I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &calib_req_reg, sizeof(calib_req_reg), &calib_point, sizeof(calib_point)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Calibration Confirmation register//
	char calib_confirm_reg = 0x0D; 
	char output = -1; 
	I2C_DEV_TAKE_MUTEX(dev);
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &calib_confirm_reg, sizeof(calib_confirm_reg)));
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &output, sizeof(output)));
    I2C_DEV_GIVE_MUTEX(dev);

	switch (calib_point) {
		//if 4.0 solution //
		case 2: 
			// Make sure calibration confirmation register confirmed calibration setting// 
			if (output == 1 || output == 3 || output == 5 || output == 7) {
				ESP_LOGI(TAG, "4.0 ph calibration set");
			} else {
				ESP_LOGE(TAG, "4.0 ph calibration unable to be set");
				return ESP_FAIL; 
			}
			break;
		//if 7.0 solution //
		case 3: 
			// Make sure calibration confirmation register confirmed calibration setting// 
			if (output == 2 || output == 3 || output == 6 || output == 7) {
				ESP_LOGI(TAG, "7.0 ph calibration set");
			} else {
				ESP_LOGE(TAG, "7.0 ph calibration unable to be set");
				return ESP_FAIL; 
			}
			break;
		//if 10.0 solution//
		case 4: 
			// Make sure calibration confirmation register confirmed calibration setting// 
			if (output == 4 || output == 5 || output == 6 || output == 7) {
				ESP_LOGI(TAG, "10.0 ph calibration set");
			} else {
				ESP_LOGE(TAG, "10.0 ph calibration unable to be set");
				return ESP_FAIL; 
			}
			break;
		default: 
			ESP_LOGE(TAG, "Unable to confirm calibration.");
			return ESP_FAIL; 
	}
	return ESP_OK;
}

esp_err_t clear_calibration_ph(ph_sensor_t *dev) {
	//Calibration request Register: Transmit 1 to clear old calibration settings//
	char calib_req_reg = 0x0C; 
	char calib_point = 1; 
	I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &calib_req_reg, sizeof(calib_req_reg), &calib_point, sizeof(calib_point)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Calibration Confirmation register//
	char calib_confirm_reg = 0x0D; 
	char output = -1; 
	I2C_DEV_TAKE_MUTEX(dev);
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &calib_confirm_reg, sizeof(calib_confirm_reg)));
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &output, sizeof(output)));
    I2C_DEV_GIVE_MUTEX(dev);

	// Make sure calibration confirmation register confirmed calibration clear setting// 
	if (output == 0) {
		ESP_LOGI(TAG, "Calibration data cleared");
	} else {
		ESP_LOGE(TAG, "Calibration data not cleared");
		return ESP_FAIL; 
	}
	return ESP_OK; 
}

esp_err_t read_ph_with_temperature(ph_sensor_t *dev, float temperature, float *ph) {
	//Check if temperature is in valid range
	float temp = temperature;
	if (temp <= 10.0 || temp >= 35.0) {
		//Set to Default value//
		temp = 25.0;
	}
	//Round float temp to 2 decimal places first//
	float nearest = roundf(temp * 100) / 100;
	unsigned int temp_compensation = (unsigned int) (nearest * 100); 
	// Get each byte using bitwise operations for temperature value //
	unsigned char msb = (temp_compensation>>24) & 0xFF;  
	unsigned char high_byte = (temp_compensation>>16) & 0xFF; 
	unsigned char low_byte = (temp_compensation>>8) & 0xFF; 
	unsigned char lsb = temp_compensation & 0xFF; 
	// Write each temperature byte to ph registers //
	char msb_reg = 0x0E;
	char high_reg = 0x0F; 
	char low_reg = 0x10; 
	char lsb_reg = 0x11; 
	I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &msb_reg, sizeof(msb_reg), &msb, sizeof(msb)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &high_reg, sizeof(high_reg), &high_byte, sizeof(high_byte)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &low_reg, sizeof(low_reg), &low_byte, sizeof(low_byte)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &lsb_reg, sizeof(lsb_reg), &lsb, sizeof(lsb)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Temperature Compensation Confirmation// 
	int count = 0; 
	unsigned char bytes [4]; 
	float check_temp = 0.0f; 
	// Make sure temperature compensation is set //
	while (check_temp != nearest) {
		// if temp is not updated after 3 readings then return //
		if (count == 3) {
			ESP_LOGE(TAG, "Unable to set temperature compensation point.");
			break; 
		} 
		// Get each byte from temperature confirmation register and place in bytes array// 
		msb_reg = 0x12; 
		high_reg = 0x13;
		low_reg = 0x14;
		lsb_reg = 0x15; 
		I2C_DEV_TAKE_MUTEX(dev);
		I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &msb_reg, sizeof(msb_reg)));
    	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &msb, sizeof(msb)));
		bytes[0] = msb; 
		I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &high_reg, sizeof(high_reg)));
    	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &high_byte, sizeof(high_byte)));
		bytes[1] = high_byte; 
		I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &low_reg, sizeof(low_reg)));
    	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &low_byte, sizeof(low_byte)));
		bytes[2] = low_byte; 
		I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &lsb_reg, sizeof(lsb_reg)));
    	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &lsb, sizeof(lsb)));
		bytes[3] = lsb; 
    	I2C_DEV_GIVE_MUTEX(dev);
		// Use bitwise shifting to combine bytes into an int value and then cast to float to get true temp value// 
		unsigned int check = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
		check_temp = ((float) check) / 100; 
		count++;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	//ESP_LOGI(TAG, "Temp Point Set: %.2f", check_temp);

	//Commands to check for new ph data//
	char new_reading_reg = 0x07; 
	char new_reading = 0; 
	count = 0; 
	// Keep checking unitl new ph data availible //
	while (new_reading == 0) {
		// If no data available after 3 checks then return // 
		if (count == 5) {
			ESP_LOGE(TAG, "Unable to get new ph reading.");
			return ESP_FAIL; 
		} 
		// Read if new data available (will return 1 for new reading) //
		I2C_DEV_TAKE_MUTEX(dev);
		I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &new_reading_reg, sizeof(new_reading_reg)));
    	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &new_reading, sizeof(new_reading)));
		I2C_DEV_GIVE_MUTEX(dev);
		vTaskDelay(pdMS_TO_TICKS(1000));
		if (new_reading == 1) {
			//reset back to 0 for next use//
			char reset = 0; 
			I2C_DEV_TAKE_MUTEX(dev);
    		I2C_DEV_CHECK(dev, i2c_dev_write(dev, &new_reading_reg, sizeof(new_reading_reg), &reset, sizeof(reset)));
			I2C_DEV_GIVE_MUTEX(dev);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		count++; 
	}

	//get each byte of ph value from register// 
	msb_reg = 0x16; 
	high_reg = 0x17;
	low_reg = 0x18;
	lsb_reg = 0x19; 
	I2C_DEV_TAKE_MUTEX(dev);
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &msb_reg, sizeof(msb_reg)));
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &msb, sizeof(msb)));
	bytes[0] = msb; 
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &high_reg, sizeof(high_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &high_byte, sizeof(high_byte)));
	bytes[1] = high_byte; 
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &low_reg, sizeof(low_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &low_byte, sizeof(low_byte)));
	bytes[2] = low_byte;  
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &lsb_reg, sizeof(lsb_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &lsb, sizeof(lsb)));
	bytes[3] = lsb; 
    I2C_DEV_GIVE_MUTEX(dev);
	// Use bitwise shifting and casting to get ph value and place into ph paramter // 
	int val = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
	*ph = ((float) val) / 1000; 
	vTaskDelay(pdMS_TO_TICKS(1000));
    return ESP_OK;
}

esp_err_t read_ph(ph_sensor_t *dev, float *ph) {
	//Commands to check for new ph data//
	char new_reading_reg = 0x07; 
	char new_reading = 0; 
	int count = 0; 
	// Keep checking unitl new ph data availible //
	while (new_reading == 0) {
		// If no data available after 3 checks then return // 
		if (count == 5) {
			ESP_LOGE(TAG, "Unable to get new ph reading.");
			return ESP_FAIL; 
		} 
		// Read if new data available (will return 1 for new reading) //
		I2C_DEV_TAKE_MUTEX(dev);
		I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &new_reading_reg, sizeof(new_reading_reg)));
    	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &new_reading, sizeof(new_reading)));
		I2C_DEV_GIVE_MUTEX(dev);
		vTaskDelay(pdMS_TO_TICKS(1000));
		if (new_reading == 1) {
			//reset back to 0 for next use//
			char reset = 0; 
			I2C_DEV_TAKE_MUTEX(dev);
    		I2C_DEV_CHECK(dev, i2c_dev_write(dev, &new_reading_reg, sizeof(new_reading_reg), &reset, sizeof(reset)));
			I2C_DEV_GIVE_MUTEX(dev);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		count++; 
	}

	//get each byte of ph value from register// 
	unsigned char bytes[4];
	unsigned char msb = 0x00;  
	unsigned char high_byte = 0x00; 
	unsigned char low_byte = 0x00; 
	unsigned char lsb = 0x00; 
	char msb_reg = 0x16; 
	char high_reg = 0x17;
	char low_reg = 0x18;
	char lsb_reg = 0x19; 
	I2C_DEV_TAKE_MUTEX(dev);
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &msb_reg, sizeof(msb_reg)));
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &msb, sizeof(msb)));
	bytes[0] = msb; 
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &high_reg, sizeof(high_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &high_byte, sizeof(high_byte)));
	bytes[1] = high_byte; 
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &low_reg, sizeof(low_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &low_byte, sizeof(low_byte)));
	bytes[2] = low_byte;  
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &lsb_reg, sizeof(lsb_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &lsb, sizeof(lsb)));
	bytes[3] = lsb; 
    I2C_DEV_GIVE_MUTEX(dev);
	// Use bitwise shifting and casting to get ph value and place into ph paramter // 
	int val = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
	*ph = ((float) val) / 1000; 
	vTaskDelay(pdMS_TO_TICKS(1000));

    return ESP_OK;
}

