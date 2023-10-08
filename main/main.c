#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


// -------- User Defined Library --------
#include "DHT22.h"


// -------- GPIO PIN DECLARATION --------
#define DHT22_PIN GPIO_NUM_32


/*----------------------------------------------------

Function:
Initialization Setup

----------------------------------------------------*/
void init_setup()
{
  // initialize the NVS (Non-Volatile Storage)
  nvs_flash_init();
  vTaskDelay(2000 / portTICK_PERIOD_MS);
}


/*----------------------------------------------------

Task:
DHT Temperature and Humidity Reader
from DHT22.h

----------------------------------------------------*/
void DHT_task(void *pvParameter)
{
  setDHTgpio(DHT22_PIN); // Set GPIO for DHT22 Sensor

  while (1)
  {
    printf("DHT Sensor Readings\n");
    int ret = readDHT(); // Read Temperatuew and Humidity Value

    errorHandler(ret); // checks for the response after reading from DHT22

    printf("Humidity %.2f %%\n", getHumidity());           // get the humidity value
    printf("Temperature %.2f degC\n\n", getTemperature()); // get the temperature value

    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay 2s
  }
}


/*----------------------------------------------------

Main App Function

----------------------------------------------------*/
void app_main()
{
  // First, Call Initialization Setup
  init_setup();

  /*----------------------------------------------------

  TASK CALLING

  ----------------------------------------------------*/
  xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
}