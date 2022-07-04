#include "MS5837.h"

extern "C" { void app_main(void); }

void loop(void *pvParameter)
{ 
    MS5837 sensor;
};


void app_main() {
    int sda = 0;
    int scl = 0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,         // select GPIO specific to your project
        .scl_io_num = scl,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .clk_flags = 0
    };

    // TODO error handling
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    xTaskCreate(&loop, "main", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}