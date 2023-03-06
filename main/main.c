/**
 * @file main.c
 * @author Nguyen Huy Hai ( @20203898 ) 
 * @brief Main file of WeAir project
 * @version 0.1
 * @date 2023-03-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/*------------------------------------ INCLUDE LIBRARY ------------------------------------ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_attr.h"
#include "esp_spi_flash.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>

// #include "driver/adc.h"
// #include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"

// #include "lwip/err.h"
// #include "lwip/sys.h"

// #include "sntp_sync.h"
//#include "bme280.h"
#include "sdcard.h"
//#include "button.h"
//#include "pms7003.h"
#include "DS3231Time.h"
// #include "datamanager.h"
// #include "DeviceManager.h"


/*------------------------------------ DEFINE ------------------------------------ */

__attribute__((unused)) static const char *TAG = "Main";


#define PERIOD_GET_DATA_FROM_SENSOR                 (TickType_t)(5000 / portTICK_RATE_MS)
#define PERIOD_SAVE_DATA_SENSOR_TO_SDCARD           (TickType_t)(2500 / portTICK_RATE_MS)
#define PERIOD_SAVE_DATA_AFTER_WIFI_RECONNECT       (TickType_t)(1000 / portTICK_RATE_MS)

#define NO_WAIT                                     (TickType_t)(0)
#define WAIT_10_TICK                                (TickType_t)(10 / portTICK_RATE_MS)

#define QUEUE_SIZE              10U
#define NAME_FILE_QUEUE_SIZE    5U




// esp_mqtt_client_handle_t mqttClient_handle = NULL;

// TaskHandle_t getDataFromSensorTask_handle = NULL;
// TaskHandle_t saveDataSensorToSDcardTask_handle = NULL;
// TaskHandle_t saveDataSensorAfterReconnectWiFiTask_handle = NULL;
// TaskHandle_t mqttPublishMessageTask_handle = NULL;

// SemaphoreHandle_t getDataSensor_semaphore = NULL;
// SemaphoreHandle_t writeDataToSDcard_semaphore = NULL;
// SemaphoreHandle_t sentDataToMQTT_semaphore = NULL;
// SemaphoreHandle_t writeDataToSDcardNoWifi_semaphore = NULL;

// QueueHandle_t dataSensorSentToSD_queue;
// QueueHandle_t dataSensorSentToMQTT_queue;
// QueueHandle_t moduleError_queue;
// QueueHandle_t nameFileSaveDataNoWiFi_queue;

//static struct statusDevice_st statusDevice = { 0 };

static char nameFileSaveData[21];
static char dataString[20] = "Hello SD card";
static int  counter = 0;
// char mqttTopic[32];
// uint8_t MAC_address[6];

// const char *formatDataSensorString = "{\n\t\"station_id\":\"%x%x%x%x\",\n\t\"Time\":%lld,\n\t\"Temperature\":%.2f,\n\t\"Humidity\":%.2f,\n\t\"Pressure\":%.2f,\n\t\"PM1\":%d,\n\t\"PM2p5\":%d,\n\t\"PM10\":%d\n}";

//------------------------------------------------------------------

//i2c_dev_t ds3231_device;


void writeDataToSDCard_task(void *parameter)
{
    for( ; ; )
    {
        printf("write to sd card\n");
        counter++;
        sprintf(nameFileSaveData,"Data%d",counter);
        static esp_err_t errorCode_t;
        errorCode_t = sdcard_write_data_to_file(nameFileSaveData,"%s",dataString);
        if (errorCode_t == ESP_OK){
            printf("Write data to SD completed.\n");
        }

        vTaskDelay(PERIOD_SAVE_DATA_SENSOR_TO_SDCARD);
    }
}




/*------------------------------------ MAIN_APP ------------------------------------*/

void app_main(void)
{
    // Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(200)); 

// Initialize SD card
#if(CONFIG_USING_SDCARD)
    // Initialize SPI Bus

    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t  mount_config_t   = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t            spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t                host_t           = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t       slot_config      = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK_WITHOUT_ABORT(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));
#endif  //CONFIG_USING_SDCARD


// Initialize BME280 Sensor
// #if(CONFIG_USING_BME280)
//     ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
//     ESP_LOGI(__func__, "Initialize BME280 sensor(I2C/Wire%d).", CONFIG_BME_I2C_PORT);

//     ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_init(&bme280_device, &bme280_params, BME280_ADDRESS,
//                                     CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL));

// #endif  //CONFIG_USING_BME280


// Initialize RTC module
// #if(CONFIG_USING_RTC)
//     ESP_LOGI(__func__, "Initialize DS3231 module(I2C/Wire%d).", CONFIG_RTC_I2C_PORT);

//     memset(&ds3231_device, 0, sizeof(i2c_dev_t));

//     ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_initialize(&ds3231_device, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
//     //ds3231_convertTimeToString(&ds3231_device, nameFileSaveData, 10);
// #endif  //CONFIG_USING_RTC

    xTaskCreate(writeDataToSDCard_task, "writeDataToSDCard_task", 1024 * 16, NULL, 4, NULL);
}

