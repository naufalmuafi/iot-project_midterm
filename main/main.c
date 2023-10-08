#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// -------- User Defined Library --------
#include "DHT22.h"


// -------- GPIO PIN DECLARATION --------
#define DHT22_PIN GPIO_NUM_32
#define POT_PIN GPIO_NUM_15


static esp_adc_cal_characteristics_t adc2_chars;


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

    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay selama 2s
  }
}


/*----------------------------------------------------

Task:
Potensiometer Read

----------------------------------------------------*/
void potentiometer_task(void *pvParameter)
{  
  // Konfigurasi ADC untuk mengukur nilai analog
  esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_DEFAULT, 0, &adc2_chars);

  adc2_config_width(ADC_WIDTH_BIT_DEFAULT);                       // Resolusi 12-bit
  adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_DB_0); // Tegangan referensi penuh (0 dB)

  while (1)
  {    
    // convert to mV
    uint32_t mV = esp_adc_cal_raw_to_voltage(adc2_get_raw(ADC2_CHANNEL_3), &adc2_chars);

    // Tampilkan nilai di terminal serial
    printf("Nilai Potensiometer: %d\n", mV);

    vTaskDelay(pdMS_TO_TICKS(100)); // delay selama 1s
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
  xTaskCreate(potentiometer_task, "potentiometer_task", 2048, NULL, 5, NULL);
}