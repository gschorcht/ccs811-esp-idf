#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "esp8266_wrapper.h"

#include "ccs811.c"

#define I2C_BUS 0

static ccs811_sensor_t* sensor;

void user_task(void *pvParameters)
{
    uint16_t tvoc;
    uint16_t eco2;

    while (1)
    {
        // get environmental data from another sensor and set them
        // ccs811_set_environmental_data (sensor, 25.3, 47.8);

        // get the results and do something with them
        if (ccs811_get_results (sensor, &tvoc, &eco2, 0, 0))
            printf("%.3f CCS811 Sensor periodic: TVOC %d ppb, eCO2 %d ppm\n",
                   (double)sdk_system_get_time()*1e-3, tvoc, eco2);

        // passive waiting until 1 second is over
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

int main (int argc, char* argv[])
{
    if ((i2c_bus[I2C_BUS] = open("/dev/i2c-9", O_RDWR)) < 0)
    {
        perror("Could not open i2c interface");
        exit(1);
    }
    
    sensor = ccs811_init_sensor (I2C_BUS, CCS811_I2C_ADDRESS_2);
 
    if (sensor)
    {
        // start periodic measurement with one measurement per second
        ccs811_set_mode (sensor, ccs811_mode_1s);
        
        user_task (0);
    }
}
