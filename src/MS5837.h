#pragma once

#include <stdint.h>
#include <driver/i2c.h>

class MS5837 {
    private:
        i2c_port_t port;

        uint16_t C[8];
        uint32_t D1_pres, D2_temp;
        int32_t TEMP;
        int32_t P;
        uint8_t _model;

        float fluidDensity;

        esp_err_t i2c_register_write_byte(uint8_t reg_addr, uint8_t data);
        esp_err_t i2c_register_read(uint8_t reg_addr, uint8_t *data, size_t len);

        /** Performs calculations per the sensor data sheet for conversion and
         *  second order compensation.
         */
        void calculate();

        uint8_t crc4(uint16_t n_prom[]);
    public:
    	static const float Pa;
        static const float bar;
        static const float mbar;

        static const uint8_t MS5837_30BA;
        static const uint8_t MS5837_02BA;
        static const uint8_t MS5837_UNRECOGNISED;

        MS5837(i2c_port_t port = I2C_NUM_0);

        bool init();

        //bool init(TwoWire &wirePort = Wire);
        //bool begin(TwoWire &wirePort = Wire); // Calls init()

        /** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
         * and MS5837::MS5837_02BA.
         */
        void setModel(uint8_t model);
        uint8_t getModel();

        /** Provide the density of the working fluid in kg/m^3. Default is for
         * seawater. Should be 997 for freshwater.
         */
        void setFluidDensity(float density);

        /** The read from I2C takes up to 40 ms, so use sparingly is possible.
         */
        void read();

        /** Pressure returned in mbar or mbar*conversion rate.
         */
        float pressure(float conversion = 1.0f);

        /** Temperature returned in deg C.
         */
        float temperature();

        /** Depth returned in meters (valid for operation in incompressible
         *  liquids only. Uses density that is set for fresh or seawater.
         */
        float depth();

        /** Altitude returned in meters (valid for operation in air only).
         */
        float altitude();
};
