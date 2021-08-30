/*
 * ezo_ec_sensor.c
 *
 *  Created on: July 1, 2020
 *      Author: Karthick Siva. 
 */

#include "oem_ec_sensor.h"
#include <esp_log.h>
#include <esp_err.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* MACRO for checkng argument paramters */
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
/* I2C Protocol Speed Paramter (10-100 kHz for OEM Device) */
#define I2C_FREQ_HZ 10000
/* MACRO for calibration function*/
#define STABILIZATION_ACCURACY 0.002
/* MACRO for calibration function */
#define STABILIZATION_COUNT_MAX 10
/* MACRO for dry calibration function */
#define DRY_CALIBRATION_READING_COUNT 20
/* Debugging Tag for EC sensor */
static const char *TAG = "Atlas EC Sensor";

esp_err_t ec_init(ec_sensor_t *dev, i2c_port_t port, uint8_t addr, int8_t sda_gpio, int8_t scl_gpio) {
	// Check Arguments
    CHECK_ARG(dev);
    if (addr < EC_ADDR_BASE || addr > EC_ADDR_BASE + 7) {
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

esp_err_t activate_ec(ec_sensor_t *dev) {
	// Write 0x01 to register 0x06 for activating ec sensor // 
	char data = 0x01; 
	char reg = 0x06; 
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &reg, sizeof(reg), &data, sizeof(data)));
    I2C_DEV_GIVE_MUTEX(dev);
	return ESP_OK; 
}

esp_err_t hibernate_ec(ec_sensor_t *dev) {
	//Write 0x00 to register 0x06 for placing ec sensor in hibernation mode // 
	char data = 0x00; 
	char reg = 0x06; 
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &reg, sizeof(reg), &data, sizeof(data)));
    I2C_DEV_GIVE_MUTEX(dev);
	vTaskDelay(pdMS_TO_TICKS(1000));
	return ESP_OK; 
}

esp_err_t probe_type(ec_sensor_t *dev, float probe_val) {
	//Write the probe value to register //
	unsigned int probe = (unsigned int) probe_val * 100; 
	// Get each byte using bitwise operations for temperature value //
	unsigned char msb = (probe>>8) & 0xFF; 
	unsigned char lsb = probe & 0xFF; 
	char msb_reg = 0x08;
	char lsb_reg = 0x09; 
	I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &msb_reg, sizeof(msb_reg), &msb, sizeof(msb)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &lsb_reg, sizeof(lsb_reg), &lsb, sizeof(lsb)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	return ESP_OK; 

}

esp_err_t calibrate_ec(ec_sensor_t *dev){
	uint8_t count = 0;

	float ec = 0;
	float ec_min = 0;
	float ec_max = 0;

	// Keep restarting until 10 consecutive ec values are within stabilization accuracy range
	while(count < STABILIZATION_COUNT_MAX){
		esp_err_t err = read_ec(dev, &ec);	// read ec with temperature
		ESP_LOGI(TAG, "ec: %f", ec);
		if (err == ESP_OK) {	// Proceed if ec sensor responds with success code
			if(count == 0) {	// If first reading, then calculate stabilization range
				ec_min = ec * (1 - STABILIZATION_ACCURACY);
				ec_max = ec * (1 + STABILIZATION_ACCURACY);
				ESP_LOGI(TAG, "min ec: %f, max ec: %f", ec_min, ec_max);
				count++;
			} else {
				if(ec >= ec_min && ec <= ec_max){	// increment count if ec is within range
					count++;
				} else {
					count = 0;	// reset count to zero if ec is not within range
				}
			}
		} else {
			ESP_LOGI(TAG, "response code: %d", err);
		}
	}

	// Identify and create calibration command
	char calib_point = 3; 
	unsigned char msb = 0x00; 
	unsigned char high_byte = 0x00; 
	unsigned char low_byte = 0x00; 
	unsigned char lsb = 0x00; 
	if(ec >= 5 && ec < 20) {
		// 12.88 solution //
		low_byte = 0x05; 
		lsb = 0x08;
		ESP_LOGI(TAG, "12.88 millisiemens solution identified");
	} else {
		ESP_LOGE(TAG, "calibration solution not identified, ec is lower than 7 millisiemens or greater than 90 millisiemens");
		return ESP_FAIL;
	}

	// Send Calibration Command to EZO Sensor by sending each byte from msb to lsb
	char msb_reg = 0x0A;
	char high_reg = 0x0B; 
	char low_reg = 0x0C; 
	char lsb_reg = 0x0D; 
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &msb_reg, sizeof(msb_reg), &msb, sizeof(msb)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &high_reg, sizeof(high_reg), &high_byte, sizeof(high_byte)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &low_reg, sizeof(low_reg), &low_byte, sizeof(low_byte)));
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, &lsb_reg, sizeof(lsb_reg), &lsb, sizeof(lsb)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Calibration request Register//
	char calib_req_reg = 0x0E; 
	I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &calib_req_reg, sizeof(calib_req_reg), &calib_point, sizeof(calib_point)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Calibration Confirmation register//
	char calib_confirm_reg = 0x0F; 
	char output = -1; 
	I2C_DEV_TAKE_MUTEX(dev);
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &calib_confirm_reg, sizeof(calib_confirm_reg)));
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &output, sizeof(output)));
    I2C_DEV_GIVE_MUTEX(dev);

	switch (calib_point) {
		//if 12.88 solution //
		case 3: 
			// Make sure calibration confirmation register confirmed calibration setting//
			if (output == 2 || output == 3) {
				ESP_LOGI(TAG, "Single Point 12.88 millisiemen calibration set");
			} else {
				ESP_LOGE(TAG, "Single Point 12.88 millisiemen calibration uanble to be set");
				return ESP_FAIL; 
			}
			break;
		default: 
			ESP_LOGE(TAG, "Unable to confirm calibration.");
			return ESP_FAIL; 
	}
	return ESP_OK;
}

esp_err_t calibrate_ec_dry(ec_sensor_t *dev) {
	// Get dry readings // 
	float ec = 0;
	for (int i = 0; i < DRY_CALIBRATION_READING_COUNT; i++) {
		read_ec(dev, &ec);
	}
	// Create Calibration Command
	char calib_point = 2; 
	char calib_req_reg = 0x0E; 
	I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &calib_req_reg, sizeof(calib_req_reg), &calib_point, sizeof(calib_point)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Calibration Confirmation register//
	char calib_confirm_reg = 0x0F; 
	char output = -1; 
	I2C_DEV_TAKE_MUTEX(dev);
	I2C_DEV_CHECK(dev, i2c_dev_write(dev, NULL, 0, &calib_confirm_reg, sizeof(calib_confirm_reg)));
    I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &output, sizeof(output)));
    I2C_DEV_GIVE_MUTEX(dev);

	// Make sure calibration confirmation register confirmed calibration dry setting//
	if (output == 1 || output == 3) {
		ESP_LOGI(TAG, "Dry calibration set");
	} else {
		ESP_LOGE(TAG, "Dry calibration uanble to be set");
		return ESP_FAIL; 
	}
	return ESP_OK;
}

esp_err_t clear_calibration_ec(ec_sensor_t *dev) {
	//Calibration request Register: Transmit 1 to clear old calibration settings//
	char calib_req_reg = 0x0E; 
	char calib_point = 1; 
	I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &calib_req_reg, sizeof(calib_req_reg), &calib_point, sizeof(calib_point)));
    I2C_DEV_GIVE_MUTEX(dev);
    vTaskDelay(pdMS_TO_TICKS(1000));	// Processing Delay

	//Calibration Confirmation register//
	char calib_confirm_reg = 0x0F; 
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

esp_err_t read_ec_with_temperature(ec_sensor_t *dev, float temperature, float *ec) {
	//First check if temperature is in valid range// 
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
	// Write each temperature byte to ec registers //
	char msb_reg = 0x10;
	char high_reg = 0x11; 
	char low_reg = 0x12; 
	char lsb_reg = 0x13; 
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
		msb_reg = 0x14; 
		high_reg = 0x15;
		low_reg = 0x16;
		lsb_reg = 0x17; 
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
		int check = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
		check_temp = ((float) check) / 100; 
		count++;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	//ESP_LOGI(TAG, "Temp Point Set: %.2f", check_temp);

	//Commands to check for new ec data//
	char new_reading_reg = 0x07; 
	char new_reading = 0; 
	count = 0; 
	// Keep checking unitl new ec data availible //
	while (new_reading == 0) {
		// If no data available after 3 checks then return // 
		if (count == 5) {
			ESP_LOGE(TAG, "Unable to get new ec reading.");
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

	//get each byte of ec value from register// 
	msb_reg = 0x18; 
	high_reg = 0x19;
	low_reg = 0x1A;
	lsb_reg = 0x1B; 
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
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &lsb_reg, sizeof(lsb_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &lsb, sizeof(lsb)));
	bytes[3] = lsb; 
    I2C_DEV_GIVE_MUTEX(dev);
	// Use bitwise shifting and casting to get ec value and place into ec paramter // 
	int val = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
	*ec = ((float) val) / 100; 

    return ESP_OK;
}

esp_err_t read_ec(ec_sensor_t *dev, float *ec) {
	//Commands to check for new ec data//
	char new_reading_reg = 0x07; 
	char new_reading = 0; 
	int count = 0; 
	// Keep checking unitl new ec data availible //
	while (new_reading == 0) {
		// If no data available after 3 checks then return // 
		if (count == 5) {
			ESP_LOGE(TAG, "Unable to get new ec reading.");
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

	//get each byte of ec value from register// 
	unsigned char msb = 0x00;  
	unsigned char high_byte = 0x00; 
	unsigned char low_byte = 0x00; 
	unsigned char lsb = 0x00; 
	unsigned char bytes[4];
	char msb_reg = 0x18; 
	char high_reg = 0x19;
	char low_reg = 0x1A;
	char lsb_reg = 0x1B; 
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
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &lsb_reg, sizeof(lsb_reg)));
	I2C_DEV_CHECK(dev, i2c_dev_read(dev, NULL, 0, &lsb, sizeof(lsb)));
	bytes[3] = lsb; 
    I2C_DEV_GIVE_MUTEX(dev);
	// Use bitwise shifting and casting to get ec value and place into ec paramter // 
	int val = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
	*ec = ((float) val) / 100; 

    return ESP_OK;
}







