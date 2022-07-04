#pragma once

#include <stdint.h>

class MS5837 {
    private:
        uint8_t sda;
        uint8_t scl;
    public:
    	static const float Pa;
        static const float bar;
        static const float mbar;

        static const uint8_t MS5837_30BA;
        static const uint8_t MS5837_02BA;
        static const uint8_t MS5837_UNRECOGNISED;

        MS5837(uint8_t pinSDA, uint8_t pinSCL);

        uint8_t getSDA();
        uint8_t getSCL();

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
