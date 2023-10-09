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
#include "driver/uart.h"
#include "esp_log.h"

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

// UART Communication Declaration
#define ECHO_TEST_TXD 1
#define ECHO_TEST_RXD 3
#define UART_DELAY_MS 100
#define BUF_SIZE 1024

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

// Queue Declaration: Send Data Humidity from DHT22 Sensor
QueueHandle_t hum_queue;
float         hum_receive;

// Queue Declaration: Send Data Temperature from DHT22 Sensor
QueueHandle_t tmp_queue;
float         tmp_receive;

// Mutex Declaration: Create 1 Mutex for Critical Session
SemaphoreHandle_t mutex;


// -------- VARIABLE DECLARATION --------

// OLED Variable
int oled_state;


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
    int ret = readDHT(); // Read Temperatuew and Humidity Value

    errorHandler(ret); // checks for the response after reading from DHT22

    float hum = getHumidity();     // get the humidity value
    float tmp = getTemperature();  // get the temperature value

    // send adc_value to Queue
    xQueueSend(hum_queue, &hum, (TickType_t)0);
    xQueueSend(tmp_queue, &tmp, (TickType_t)0);
    vTaskDelay(10 / portTICK_PERIOD_MS); // delay for 10 ms
  }
}


/*----------------------------------------------------

Task:
Potensiometer Read

----------------------------------------------------*/
void potentio_task(void *pvParameter)
{  
  // -------- ADC CONFIGURATION --------

  // Calibrate the ADC
  esp_adc_cal_characterize(ADC_UNIT, ADC_ATTENUASI, ADC_WIDTH_BIT_DEFAULT, 0, &CHANNEL_ADC);

  adc1_config_width(ADC_WIDTH_BIT_DEFAULT); // Config ADC bit width
  adc1_config_channel_atten(POT_PIN, ADC_ATTENUASI); // config potentio pin with attenuation parameter

  while (1)
  {
    int adc_value = adc1_get_raw(POT_PIN); // capture RAW ADC Value    

    // send adc_value to Queue
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

    // send adc_value to Queue
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
  // Configuring GPIO to control the servo
  gpio_config_t io_conf = {
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << SERVO_PIN),
      .intr_type = GPIO_INTR_DISABLE,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&io_conf);

  // Configuring LEDC to control PWM
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
    // Take Mutex to save receiving data from potentio
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
      // Receive potentio value from queue
      xQueueReceive(pot_queue, &pot_receive, (TickType_t)5);

      // normalization adc value to rotation value
      int val = map(pot_receive, 0, 4095, 200, 1000);

      // set servo motion
      ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, val);
      ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

      // Give semaphore back
      xSemaphoreGive(mutex);
    }
    vTaskDelay(10/portTICK_PERIOD_MS); // delay for 10 ms
  }
}

/*----------------------------------------------------

Task:
Display to OLED

Arg:
state 1 = display humidity ; state 2 = display temperature
state 3 = display LDR      ; state 4 = display Servo Degree

----------------------------------------------------*/
void oled_task(void *pvParameter)
{
  // Create OLED Object
  SSD1306_t dev;
  i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
  ssd1306_init(&dev, 128, 32);
  ssd1306_clear_screen(&dev, false);
  ssd1306_contrast(&dev, 0xff);

  const int uart_num = UART_NUM_0;
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };

  uart_param_config(uart_num, &uart_config);
  uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);

  char cli_data[8];

  while(1)
  {
    // read CLI Command data from UART
    if (uart_read_bytes(UART_NUM_0, cli_data, sizeof(cli_data), pdMS_TO_TICKS(100)) > 0)
    {
      oled_state = atoi(cli_data); // convert raw data to int
      ssd1306_clear_screen(&dev, false);
    }

    if (oled_state == 1)
    {
      char  head[10],
            tail[10];

      xQueueReceive(hum_queue, &hum_receive, (TickType_t)5);
      
      sprintf(head, "Humidity");
      sprintf(tail, "%.2f %%", hum_receive);

      ssd1306_display_text(&dev, 0, head, 10, false);
      ssd1306_display_text(&dev, 2, tail, 10, false);
    }
    else if (oled_state == 2)
    {
      char  head[12],
            tail[10];

      xQueueReceive(tmp_queue, &tmp_receive, (TickType_t)5);

      sprintf(head, "Temperature");
      sprintf(tail, "%.2f C", tmp_receive);

      ssd1306_display_text(&dev, 0, head, 12, false);
      ssd1306_display_text(&dev, 2, tail, 10, false);
    }
    else if (oled_state == 3)
    {
      char  head[5],
            tail[5];

      xQueueReceive(ldr_queue, &ldr_receive, (TickType_t)5);

      sprintf(head, "LDR");
      sprintf(tail, "%d", ldr_receive);

      ssd1306_display_text(&dev, 0, head, 5, false);
      ssd1306_display_text(&dev, 2, tail, 5, false);
    }
    else if (oled_state == 4)
    {
      char  head[10],
            tail[10];

      // Take Mutex to save receiving data from potentio
      if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
      {
        // Receive potentio value from queue
        xQueueReceive(pot_queue, &pot_receive, (TickType_t)5);

        // normalization adc value to rotation value
        int val = map(pot_receive, 0, 4095, 200, 1000);

        sprintf(head, "Servo");
        sprintf(tail, "%d degree", val);

        ssd1306_display_text(&dev, 0, head, 10, false);
        ssd1306_display_text(&dev, 2, tail, 10, false);

        // Give semaphore back
        xSemaphoreGive(mutex);
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // delay for 10 ms
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

  // Create Method for Queue
  pot_queue = xQueueCreate(5, sizeof(int));
  ldr_queue = xQueueCreate(5, sizeof(int));
  hum_queue = xQueueCreate(5, sizeof(int));
  tmp_queue = xQueueCreate(5, sizeof(int));

  // Create Method for Mutex
  mutex = xSemaphoreCreateMutex();

  /*----------------------------------------------------

  TASK CALLING

  ----------------------------------------------------*/
  xTaskCreate(&oled_task,     "oled_task",      2048 * 2, NULL, 5, NULL);
  xTaskCreate(&DHT_task,      "DHT_task",       2048 * 2, NULL, 4, NULL);
  xTaskCreate(&potentio_task, "potentio_task",  2048 * 2, NULL, 4, NULL);
  xTaskCreate(ldr_task,       "ldr_task",       2048 * 2, NULL, 4, NULL);
  xTaskCreate(servo_task,     "servo task",     2048 * 2, NULL, 4, NULL);  
}