#include "MS5837.h"
#include "esp_check.h"
#include "freertos/task.h"

#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */

extern "C" { void app_main(void); }

void loop(void *pvParameter) { 
    MS5837 sensor;

    bool result = sensor.init();
    
    while(1) vTaskDelay(1);
};


void app_main() {
    int sda = 18; 
    int scl = 19;


    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,         // select GPIO specific to your project
        .scl_io_num = scl,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE
        //.master.clk_speed = I2C_MASTER_FREQ_HZ //,
        //.clk_flags = 0
    };

    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_NUM_0, &conf);

    // TODO error handling
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    xTaskCreate(&loop, "main", 2048, NULL, 5, NULL);
}