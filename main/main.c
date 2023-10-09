// -------- Including Necessary Library --------
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/dac.h"

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

// Motor Servo Declaration
#define SERVO_PIN GPIO_NUM_25
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_13_BIT

// -------- OBJECT DECLARATION --------
// ADC Declaration
static esp_adc_cal_characteristics_t CHANNEL_ADC;

// OLED Declaration
SSD1306_t dev;

// Queue Declaration: Send Data Potentiometer
QueueHandle_t pot_queue;
int           pot_receive;

// Queue Declaration: Send Data Photoresistor
QueueHandle_t ldr_queue;
int           ldr_receive;

// Mutex Declaration: Create 1 Mutex for Critical Session
SemaphoreHandle_t mutex;


/*----------------------------------------------------

Function:
Mapping Value/Normalization

----------------------------------------------------*/
long map(long x, long i_min, long i_max, long o_min, long o_max)
{
  return (x - i_min)*(o_max - o_min) / (i_max - i_min) + o_min;
}


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

  ssd1306_init(&dev, 128, 32);
  ssd1306_clear_screen(&dev, false);
  ssd1306_contrast(&dev, 0xff);
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
    char  humidity[15],
          temperature[15];
    
    printf("DHT Sensor Readings\n");
    int ret = readDHT(); // Read Temperatuew and Humidity Value

    errorHandler(ret); // checks for the response after reading from DHT22

    float hum = getHumidity();      // get the humidity value
    float temp = getTemperature();  // get the temperature value

    printf("Humidity %.2f %%\n", hum);
    printf("Temperature %.2f degC\n\n", temp);

    // display to OLED
    sprintf(humidity, "hum: %.2f %%", hum);
    sprintf(temperature, "temp: %.2f C", temp);
    ssd1306_display_text(&dev, 0, humidity, 8, false);
    ssd1306_display_text(&dev, 2, temperature, 12, false);

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

    xQueueSend(pot_queue, &adc_value, (TickType_t)0);
    vTaskDelay(10 / portTICK_PERIOD_MS); // delay for 10 ms
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

    xQueueSend(ldr_queue, &adc_value, (TickType_t)0);

    vTaskDelay(10 / portTICK_PERIOD_MS); // delay for 10 ms
  }
}

/*----------------------------------------------------

Task:
Motor Servo Motion

----------------------------------------------------*/
void servo_task(void *pvParameter)
{
  // Configure GPIO for servo control
  gpio_config_t io_conf = {
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << SERVO_PIN),
      .intr_type = GPIO_INTR_DISABLE,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&io_conf);

  // Configure LEDC for PWM control
  ledc_timer_config_t timer_conf = {
      .duty_resolution = LEDC_DUTY_RESOLUTION,
      .freq_hz = 50,
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
      .channel = LEDC_CHANNEL,
      .duty = 0,
      .gpio_num = SERVO_PIN,
      .speed_mode = LEDC_MODE,
      .timer_sel = LEDC_TIMER,
      .hpoint = 0,
  };
  ledc_channel_config(&channel_conf);

  while(1)
  {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
      xQueueReceive(pot_queue, &pot_receive, (TickType_t)5);
      int val = map(pot_receive, 0, 4095, 200, 1000);

      ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, val);
      ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

      xSemaphoreGive(mutex);
    }
    vTaskDelay(10/portTICK_PERIOD_MS); // delay for 10 ms
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

  OBJECT METHOD DECLARATION

  ----------------------------------------------------*/
  pot_queue = xQueueCreate(5, sizeof(int));
  ldr_queue = xQueueCreate(5, sizeof(int));

  mutex = xSemaphoreCreateMutex();

  /*----------------------------------------------------

  TASK CALLING

  ----------------------------------------------------*/
  xTaskCreate(&DHT_task, "DHT_task", 2048*2, NULL, 5, NULL);
  xTaskCreate(&potentiometer_task, "potentiometer_task", 2048*2, NULL, 4, NULL);
  xTaskCreate(ldr_task, "ldr_task", 2048*2, NULL, 4, NULL);
  xTaskCreate(servo_task, "servo task", 2048*2, NULL, 4, NULL);  
}