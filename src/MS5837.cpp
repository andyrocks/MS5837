#include "MS5837.h"
#include <math.h>
#include <driver/i2c.h>
#include <freertos/task.h>
#include "esp_check.h"
#include "esp_err.h"

#define TAG "MS5837"

const uint8_t MS5837_ADDR = 0x76;
const uint8_t MS5837_RESET = 0x1E;
const uint8_t MS5837_ADC_READ = 0x00;
const uint8_t MS5837_PROM_READ = 0xA0;
const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar =  1.0f;
const float MS5837::g = 9.80665;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;
const uint8_t MS5837::MS5837_UNRECOGNISED = 255;

const uint8_t MS5837_02BA01 = 0x00; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_02BA21 = 0x15; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_30BA26 = 0x1A; // Sensor version: From MS5837_30BA datasheet Version PROM Word 0

MS5837::MS5837(i2c_port_t i2cport) : port(i2cport) {
    fluidDensity = 1029;
};

bool MS5837::init() {
    // Reset the MS5837, per datasheet
	esp_err_t err = i2c_register_write(MS5837_RESET);

	ESP_RETURN_ON_ERROR(err, TAG, "Couldn't reset device");

	// Wait for reset to complete
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// Read calibration values and CRC
	for (uint8_t i = 0 ; i < 7 ; i++) {
		// _i2cPort->beginTransmission(MS5837_ADDR);
		// _i2cPort->write(MS5837_PROM_READ+i*2);
		// _i2cPort->endTransmission();

		// _i2cPort->requestFrom(MS5837_ADDR, (uint8_t)2);
		// C[i] = (_i2cPort->read() << 8) | _i2cPort->read();
		uint8_t buf[2];

		err = i2c_register_read(MS5837_PROM_READ + (i * 2), buf, sizeof(buf));
		ESP_RETURN_ON_ERROR(err, TAG, "Error reading calibration CRC");

		C[i] = (buf[0] << 8) | buf[1];
	}

	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	if (crcCalculated != crcRead) {
		return false; // CRC fail
	}

	uint8_t version = (C[0] >> 5) & 0x7F; // Extract the sensor version from PROM Word 0

	// Set _model according to the sensor version
	if (version == MS5837_02BA01 || version == MS5837_02BA21) {
		model = MS5837_02BA;
	} else if (version == MS5837_30BA26) {
		model = MS5837_30BA;
	}

	// The sensor has passed the CRC check, so we should return true even if
	// the sensor version is unrecognised.
	// (The MS5637 has the same address as the MS5837 and will also pass the CRC check)
	// (but will hopefully be unrecognised.)
	return true;
};

esp_err_t MS5837::i2c_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(port, MS5837_ADDR, &reg_addr, 1, data, len, 100 / portTICK_PERIOD_MS);
};

esp_err_t MS5837::i2c_register_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(port, MS5837_ADDR, write_buf, sizeof(write_buf), 100 / portTICK_PERIOD_MS);
};

esp_err_t MS5837::i2c_register_write(uint8_t reg_addr) {
    return i2c_master_write_to_device(port, MS5837_ADDR, &reg_addr, sizeof(reg_addr), 100 / portTICK_PERIOD_MS);;
};


uint8_t MS5837::getModel() {
	return model;
};

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
};

void MS5837::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation
	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	// Terms called
	dT = D2_temp - uint32_t(C[5]) * 256l;
	if (model == MS5837_02BA) {
		SENS = int64_t(C[1]) * 65536l+(int64_t(C[3]) * dT) / 128l;
		OFF = int64_t(C[2]) * 131072l+(int64_t(C[4]) * dT) / 64l;
		P = (D1_pres * SENS / (2097152l) - OFF) / (32768l);
	} else {
		SENS = int64_t(C[1]) * 32768l + (int64_t(C[3]) * dT) / 256l;
		OFF = int64_t(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;
		P = (D1_pres * SENS / (2097152l) - OFF) / (8192l);
	}

	// Temp conversion
	temp = 2000l + int64_t(dT) * C[6] / 8388608LL;

	//Second order compensation
	if (model == MS5837_02BA) {
		if((temp / 100) < 20){         //Low temp
			Ti = (11 * int64_t(dT) * int64_t(dT)) / (34359738368LL);
			OFFi = (31 * (temp - 2000)*(temp - 2000)) / 8;
			SENSi = (63 * (temp - 2000)*(temp - 2000)) / 32;
		}
	} else {
		if((temp / 100) < 20){         //Low temp
			Ti = (3 * int64_t(dT) * int64_t(dT)) / (8589934592LL);
			OFFi = (3 * (temp - 2000)*(temp - 2000)) / 2;
			SENSi = (5 * (temp - 2000)*(temp - 2000)) / 8;
			if((temp / 100) < -15){    //Very low temp
				OFFi = OFFi + 7 * (temp + 1500l) * (temp + 1500l);
				SENSi = SENSi + 4 * (temp + 1500l) * (temp + 1500l);
			}
		}
		else if((temp / 100) >= 20){    //High temp
			Ti = 2 * (dT * dT ) / (137438953472LL);
			OFFi = (1 * (temp - 2000) * (temp - 2000)) / 16;
			SENSi = 0;
		}
	}

	OFF2 = OFF - OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS - SENSi;

	temp = (temp - Ti);

	if (model == MS5837_02BA) {
		P = (((D1_pres * SENS2) / 2097152l - OFF2) / 32768l);
	} else {
		P = (((D1_pres * SENS2) / 2097152l - OFF2) / 8192l);
	}
}


float MS5837::pressure(float conversion) {
	if (model == MS5837_02BA) {
		return P * conversion / 100.0f;
	} else {
		return P * conversion / 10.0f;
	}
}

float MS5837::temperature() {
	return temp / 100.0f;
}

// The pressure sensor measures absolute pressure, so it will measure the atmospheric pressure + water pressure
// We subtract the atmospheric pressure to calculate the depth with only the water pressure
// The average atmospheric pressure of 101300 pascal is used for the calcuation, but atmospheric pressure varies
// If the atmospheric pressure is not 101300 at the time of reading, the depth reported will be offset
// In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and
// that value should subtracted for subsequent depth calculations.
float MS5837::depth() {
	return (pressure(MS5837::Pa) - 101300) / (fluidDensity * MS5837::g);
}

float MS5837::altitude() {
	return (1 - pow((pressure() / 1013.25), 0.190284)) * 145366.45 * 0.3048;
}

bool MS5837::read(MS5837State* state) {
	bool result = false;

	if (state == NULL) {
		return false;
	}

	read();

	state->depth = depth();
	state->temperature = temperature();


	return result;
}

void MS5837::read() {
	// Request D1 conversion
	// _i2cPort->beginTransmission(MS5837_ADDR);
	// _i2cPort->write(MS5837_CONVERT_D1_8192);
	// _i2cPort->endTransmission();

	// vTaskDelay(20 / portTICK_PERIOD_MS); // Max conversion time per datasheet

	// _i2cPort->beginTransmission(MS5837_ADDR);
	// _i2cPort->write(MS5837_ADC_READ);
	// _i2cPort->endTransmission();

	// _i2cPort->requestFrom(MS5837_ADDR, (uint8_t)3);
	// D1_pres = 0;
	// D1_pres = _i2cPort->read();
	// D1_pres = (D1_pres << 8) | _i2cPort->read();
	// D1_pres = (D1_pres << 8) | _i2cPort->read();

	// // Request D2 conversion
	// _i2cPort->beginTransmission(MS5837_ADDR);
	// _i2cPort->write(MS5837_CONVERT_D2_8192);
	// _i2cPort->endTransmission();

	// vTaskDelay(20 / portTICK_PERIOD_MS); // Max conversion time per datasheet

	// _i2cPort->beginTransmission(MS5837_ADDR);
	// _i2cPort->write(MS5837_ADC_READ);
	// _i2cPort->endTransmission();

	// _i2cPort->requestFrom(MS5837_ADDR, (uint8_t)3);
	// D2_temp = 0;
	// D2_temp = _i2cPort->read();
	// D2_temp = (D2_temp << 8) | _i2cPort->read();
	// D2_temp = (D2_temp << 8) | _i2cPort->read();

	calculate();
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for (uint8_t i = 0 ; i < 16; i++) {
		if (i % 2 == 1) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}

		for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}


