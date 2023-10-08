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
#include "ssd1306.h"


// -------- USER DEFINED DECLARATION --------

// DHT22 Declaration
#define DHT22_PIN GPIO_NUM_32

// ADC Declaration
#define CHANNEL_ADC adc1_chars
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTENUASI ADC_ATTEN_DB_11

// Potentiometer Declaration
#define POT_PIN ADC1_CHANNEL_5 //GPIO 33

// LDR Declaration
#define LDR_PIN ADC1_CHANNEL_6 // GPIO 34

// Object Declaration
static esp_adc_cal_characteristics_t CHANNEL_ADC;
SSD1306_t dev;

/*----------------------------------------------------

Function:
Initialization Setup

----------------------------------------------------*/
void init_setup()
{
  // initialize the NVS (Non-Volatile Storage)
  nvs_flash_init();
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // -------- OLED Configuration --------  
  i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

  ssd1306_init(&dev, 128, 64);
  ssd1306_clear_screen(&dev, false);
  ssd1306_contrast(&dev, 0xff);
}


/*----------------------------------------------------

Function:
OLED Handler
data type 0 = float ; data type 1 = int ; data type 2 = string

----------------------------------------------------*/
void set_OLED(const char *label, void *value, int data_type, int page)
{
  char display_text[64]; // Assume the maximum length of text is 64 characters

  if (data_type == 0)
  {
    // Tipe data float
    float float_value = *((float *)value);
    snprintf(display_text, sizeof(display_text), "%s: %.2f", label, float_value);
  }
  else if (data_type == 1)
  {
    // Tipe data integer
    int int_value = *((int *)value);
    snprintf(display_text, sizeof(display_text), "%s: %d", label, int_value);
  }
  else if (data_type == 2)
  {
    // Tipe data lainnya (misalnya, string)
    char *string_value = (char *)value;
    snprintf(display_text, sizeof(display_text), "%s: %s", label, string_value);
  }
  else
  {
    // Tipe data tidak valid
    snprintf(display_text, sizeof(display_text), "Invalid");
  }

  int length = strlen(display_text);
  ssd1306_display_text(&dev, page, display_text, length, false);
}

void set_OLED1(int length, float input, int page)
{
  char text[length];
  sprintf(text, "%.2f", input);
  ssd1306_display_text(&dev, page, text, length, false);
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