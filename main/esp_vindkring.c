/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

// #include <light_driver.h>
#include <light_driver.c>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_vindkring.h"
// #include "esp_sleep.h"
// #include <stdlib.h>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "bmx280.h"
#include <string.h>
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"


/**
 * @note Make sure set idf.py menuconfig in zigbee component as zigbee end device!
*/
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif


SemaphoreHandle_t i2c_semaphore = NULL;

char modelid[] = {14, 'M', 'O', 'D', 'E', 'L', '_', 'Z', 'I', 'G', 'B', 'E', 'E', '_', 'V'};
char manufname[] = {5, 'D', 'U', 'R', 'K', 'A'};

uint16_t undefined_value = 0;
int16_t temp = 2360, hum = 5000, pres = 5000;
int16_t temp_max = 5000;
int16_t temp_min = 0;

uint16_t pm2 = 0;
uint16_t pm1 = 0;
uint16_t pm10 = 0;

uint16_t pm2_attr_id = 0x0001U;
uint16_t pm1_attr_id = 0x0002U;
uint16_t pm10_attr_id = 0x0003U;


float temp_f = 0, pres_f = 0, hum_f = 0;

static const char *TAG = "ESP_ZB_ON_OFF_LIGHT";
/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void attr_cb(uint8_t status, uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id, void *new_value)
{
    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        uint8_t value = *(uint8_t *)new_value;
        if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            /* implemented light on/off control */
            ESP_LOGI(TAG, "on/off light set to %hd", value);
            uint8_t random_number = rand() % 255 + 1;
            uint8_t random_number2 = rand() % 255 + 1;
            uint8_t random_number3 = rand() % 255 + 1;
            
            // temp += 1;
            light_driver_set_color_RGB((bool)value, random_number, random_number2, random_number3);
        }
    } else {
        /* Implement some actions if needed when other cluster changed */
        ESP_LOGI(TAG, "cluster:0x%x, attribute:0x%x changed ", cluster_id, attr_id);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %d, status: %d", sig_type, err_status);
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee end-device config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    //esp_zb_set_network_channel(25);
    /* set the on-off light device config */
    uint8_t test_attr, test_attr2;
    
    //------------------------------------------ Attribute ------------------------------------------------
    //***********************BASIC CLUSTER***************************
    test_attr = 0;
    test_attr2 = 4;
    /* basic cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr2);
    esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr2);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);
    
    /* identify cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);


    //***********************SCENES CLUSTER***************************
    /* scenes cluster create with standard cluster + customized */
    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(NULL);
    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);
    
    /* groups cluster */
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &test_attr);

    //***********************ON OFF CLUSTER***************************
    /* on-off cluster create with standard cluster config*/
    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    // esp_zb_color_control_cluster_create
    // esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID)


    //***********************TEMPRETURE CLUSTER***************************

    esp_zb_attribute_list_t *esp_zb_tempreture_sensor_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_tempreture_sensor_attr_list, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_tempreture_sensor_attr_list, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &temp_max);
    esp_zb_temperature_meas_cluster_add_attr(esp_zb_tempreture_sensor_attr_list, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &temp_min);

    //***********************HUMIDITY CLUSTER***************************
    esp_zb_attribute_list_t *esp_zb_humidity_sensor_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_sensor_attr_list, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &hum);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_sensor_attr_list, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_sensor_attr_list, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    //***********************PRESSURE CLUSTER***************************
    esp_zb_attribute_list_t *esp_zb_pressure_sensor_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_sensor_attr_list, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pres);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_sensor_attr_list, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, &undefined_value);
    esp_zb_pressure_meas_cluster_add_attr(esp_zb_pressure_sensor_attr_list, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, &undefined_value);

    //***********************PM2.5 CLUSTER***************************
    const uint8_t attr_type = ESP_ZB_ZCL_ATTR_TYPE_U16;
    const uint8_t attr_access = ESP_ZB_ZCL_ATTR_MANUF_SPEC | ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING | ESP_ZB_ZCL_ATTR_ACCESS_SINGLETON;

    esp_zb_attribute_list_t *custom_co2_attributes_list = esp_zb_zcl_attr_list_create(CO2_CUSTOM_CLUSTER);
    esp_zb_custom_cluster_add_custom_attr(custom_co2_attributes_list, pm2_attr_id, attr_type, ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &undefined_value);
    esp_zb_custom_cluster_add_custom_attr(custom_co2_attributes_list, pm1_attr_id, attr_type, ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &undefined_value);
    esp_zb_custom_cluster_add_custom_attr(custom_co2_attributes_list, pm10_attr_id, attr_type, ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &undefined_value);


    //------------------------------------------ Cluster ------------------------------------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();



    /* create cluster lists for this endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list, esp_zb_scenes_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    

    // esp_zb_device_add_report_attr_cb(report_attr_cb);
    // esp_zb_add_read_attr_resp_cb(HA_ESP_LIGHT_ENDPOINT, read_attr_cb);

    /* add created endpoint (cluster_list) to endpoint list */
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, HA_ESP_LIGHT_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID);



    /* create cluster lists for this endpoint */
    esp_zb_cluster_list_t *esp_zb_temp_cluster_list = esp_zb_zcl_cluster_list_create();
    // esp_zb_cluster_list_add_basic_cluster(esp_zb_temp_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_temp_cluster_list, esp_zb_tempreture_sensor_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_temp_cluster_list, esp_zb_humidity_sensor_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_temp_cluster_list, esp_zb_pressure_sensor_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    

    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_temp_cluster_list, HA_ESP_TEMP_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID);



    // endpoint for co2
    esp_zb_cluster_list_t *esp_zb_co2_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_custom_cluster(esp_zb_co2_cluster_list, custom_co2_attributes_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_co2_cluster_list, HA_ESP_PM_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);


    esp_zb_device_register(esp_zb_ep_list);
    // esp_zb_device_add_set_attr_value_cb(attr_cb);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}


static void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, uint8_t clusterRole, void *value)
{

    // ESP_LOGI(TAG, "value: %d", (int)(*value));

    esp_zb_zcl_status_t state = esp_zb_zcl_set_attribute_val(
        endpoint, 
        clusterID, 
        clusterRole, 
        attributeID, 
        value, 
        false
    );

    /* Check for error */
    if(state != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Setting attribute failed! State: %d", state);
        // return ESP_FAIL;
    }


    // memcpy because set attribute returning invalid value and idk why!
    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, clusterRole, attributeID);
    memcpy(value_r->data_p, value, 2);


    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint = endpoint,
            .src_endpoint = endpoint,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .cluster_role = clusterRole
    };

    state = esp_zb_zcl_report_attr_cmd_req(&cmd);

    /* Check for error */
    if(state != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGE(TAG, "Sending total active power attribute report command failed!");
    } else {
        ESP_LOGI(TAG, "Succesfully sent report");
    }
}

void temp_report_main(void *arg) {
    ESP_LOGI(TAG, "temp report loop start");
    vTaskDelay(30000 / portTICK_PERIOD_MS);

    while (1)
    {
        ESP_LOGI(TAG, "temp report loop iteration");
        reportAttribute(
            HA_ESP_TEMP_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            &temp
        );

        reportAttribute(
            HA_ESP_TEMP_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            &hum
        );

        reportAttribute(
            HA_ESP_TEMP_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
            ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            &pres
        );


        reportAttribute(
            HA_ESP_PM_ENDPOINT,
            CO2_CUSTOM_CLUSTER,
            pm2_attr_id,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            &pm2
        );

        reportAttribute(
            HA_ESP_PM_ENDPOINT,
            CO2_CUSTOM_CLUSTER,
            pm1_attr_id,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            &pm1
        );

        reportAttribute(
            HA_ESP_PM_ENDPOINT,
            CO2_CUSTOM_CLUSTER,
            pm10_attr_id,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            &pm10
        );

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

esp_err_t i2c_master_init()
{
	// Don't initialize twice
    if (i2c_semaphore != NULL)
	{
		return ESP_FAIL;
	}
        
    i2c_semaphore = xSemaphoreCreateMutex();
    if (i2c_semaphore == NULL)
	{
		return ESP_FAIL;
	}
        
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_NUM_10,
		.scl_io_num = GPIO_NUM_11,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};

	esp_err_t ret;

	ret = i2c_param_config(I2C_NUM_0, &i2c_config);
	if(ret != ESP_OK)
	{
		return ret;
	}
        
	ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	if(ret != ESP_OK)
	{
		return ret;
	}
        
    return ESP_OK;
}

void bme280_communication_main(void *arg) {
    bmx280_t* bmx280 = bmx280_create(I2C_NUM_0);

    if (!bmx280) { 
        ESP_LOGE("BMX280", "Could not create bmx280 driver.");
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    while (1)
    {
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        do {
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        } while(bmx280_isSampling(bmx280));

        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp_f, &pres_f, &hum_f));
        ESP_LOGI("BMX280", "Read Values: temp = %.2f, pres = %.1f, hum = %.1f", temp_f, pres_f/100, hum_f);

        temp = (uint16_t)(temp_f * 100);
        hum = (uint16_t)(hum_f * 100);
        pres = (uint16_t)(pres_f/100);
    }
}

void vindkring_communication_main(void *arg) {

    ESP_LOGI("UART", "Setupping uart communication...");

    static const int RX_BUF_SIZE = 1024;
    
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(
        UART_NUM_1, 
        GPIO_NUM_3,  // TX, but we dont use it
        GPIO_NUM_2,  // RX
        UART_PIN_NO_CHANGE, 
        UART_PIN_NO_CHANGE
    );

    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_DEBUG);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            

            bool headerValid = data[0] == 0x16 && data[1] == 0x11 && data[2] == 0x0B;

            if (!headerValid) {
                ESP_LOGI("UART", "Received message with invalid header.");
            }

            uint8_t checksum = 0;

            for (uint8_t i = 0; i < 20; i++) {
                checksum += data[i];
            }

            if (checksum != 0) {
                ESP_LOGI("UART", "Received message with invalid checksum. Expected: 0. Actual: %d", checksum);
            }
 
            uint16_t t_pm25 = (data[5] << 8) | data[6];
            ESP_LOGI("UART", "pm2 value: %d", t_pm25);


            uint16_t t_pm2 = data[5]*256+data[6], 
            t_pm1 = data[9]*256+data[10], 
            t_pm10 = data[13]*256+data[14];

            ESP_LOGI("UART", "pm2: %d, pm1: %d, pm10: %d", t_pm2, t_pm1, t_pm10);


            if(checksum == 0 && headerValid) {
                ESP_LOGI("UART", "Header is valid and checksum is 0, so setting values to memory...");
                pm2 = t_pm2, pm1 = t_pm1, pm10 = t_pm10;
            }
        }
    }
}

void app_main(void)
{

    // esp_log_set_level_master(ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(i2c_master_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* load Zigbee light_bulb platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    /* hardware related and device init */
    light_driver_init(LIGHT_DEFAULT_OFF);

    xTaskCreate(bme280_communication_main, "bmx280_task",  4096, NULL, 2, NULL);
    xTaskCreate(vindkring_communication_main, "vindkring_communication",  4096, NULL, 2, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    xTaskCreate(temp_report_main, "Tempreture_report", 4096, NULL, 5, NULL);
}