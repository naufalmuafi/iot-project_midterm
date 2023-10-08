// -------- Including Necessary Library --------
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


// -------- USER DEFINED DECLARATION --------

// DHT22 Declaration
#define DHT22_PIN GPIO_NUM_32

// ADC Declaration
#define CHANNEL_ADC adc1_chars
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTENUASI ADC_ATTEN_DB_11

// Potentiometer Declaration
#define POT_PIN ADC1_CHANNEL_5

// LDR Declaration
#define LDR_PIN ADC1_CHANNEL_6

static esp_adc_cal_characteristics_t CHANNEL_ADC;


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

    vTaskDelay(2000 / portTICK_PERIOD_MS); // delay for 2s
  }
}


/*----------------------------------------------------

Task:
Potensiometer Read

----------------------------------------------------*/
void potentiometer_task(void *pvParameter)
{  
  // -------- ADC CONFIGURATION --------

  // Calibrate the ADC
  esp_adc_cal_characterize(ADC_UNIT, ADC_ATTENUASI, ADC_WIDTH_BIT_DEFAULT, 0, &CHANNEL_ADC);

  adc1_config_width(ADC_WIDTH_BIT_DEFAULT); // Config ADC bit width
  adc1_config_channel_atten(POT_PIN, ADC_ATTENUASI); // config potentio pin with attenuation parameter

  while (1)
  {
    int adc_value = adc1_get_raw(POT_PIN); // capture RAW ADC Value
    // convert to mV    
    uint32_t mV = esp_adc_cal_raw_to_voltage(adc1_get_raw(POT_PIN), &CHANNEL_ADC);

    // display value on serial terminal
    printf("ADC Value: %d\n", adc_value);
    printf("Nilai Potentiometer: %ld mV\n\n", mV);

    vTaskDelay(pdMS_TO_TICKS(500)); // delay for 0.5s
  }
}

/*----------------------------------------------------

Task:
LDR/PHotoresistor Read

----------------------------------------------------*/
void ldr_task(void *pvParameter)
{
  // -------- ADC CONFIGURATION --------

  // Calibrate the ADC
  esp_adc_cal_characterize(ADC_UNIT, ADC_ATTENUASI, ADC_WIDTH_BIT_DEFAULT, 0, &CHANNEL_ADC);

  adc1_config_width(ADC_WIDTH_BIT_DEFAULT);          // Config ADC bit width
  adc1_config_channel_atten(POT_PIN, ADC_ATTENUASI); // config potentio pin with attenuation parameter

  while (1)
  {
    int adc_value = adc1_get_raw(LDR_PIN); // capture RAW ADC Value from LDR Sensor

    // display value on serial terminal
    printf("LDR ADC Value: %d\n\n", adc_value);    

    vTaskDelay(pdMS_TO_TICKS(500)); // delay for 0.5s
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
  xTaskCreate(ldr_task, "ldr_task", 2048, NULL, 5, NULL);
}