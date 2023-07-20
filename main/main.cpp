#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"

extern "C" {
	void app_main(void);
}

extern void task_initI2C(void*);
extern void task_mpu(void*);


void app_main(void)
{
    xTaskCreate(&task_initI2C, "I2C_init", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&task_mpu, "mpu_task", 8192, NULL, 5, NULL);
}

