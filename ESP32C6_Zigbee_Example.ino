// Copyright 2023 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates simple Zigbee light bulb.
 * 
 * The example demonstrates how to use ESP Zigbee stack to create a end device light bulb.
 * The light bulb is a Zigbee end device, which is controlled by a Zigbee coordinator.
 * 
 * Proper Zigbee mode must be selected in Tools->Zigbee mode 
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 * 
 * Please check the README.md for instructions and more detailed description.
 */

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "driver/temperature_sensor.h"
temperature_sensor_handle_t tempsensor;

#include "esp_zigbee_core.h"
#include "esp_zigbee_endpoint.h"
#include "esp_zigbee_cluster.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "driver/gpio.h"  // gpio_read_level

#define LED_PIN RGB_BUILTIN
#define BTN_PIN GPIO_NUM_9


bool light_state = 0;
uint8_t light_level = 254;

/* Default End Device config */
#define ESP_ZB_ZED_CONFIG()                                     \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                   \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
        .nwk_cfg = {                                            \
            .zed_cfg = {                                        \
                .ed_timeout = ED_AGING_TIMEOUT,                 \
                .keep_alive = ED_KEEP_ALIVE,                    \
            },                                                  \
        },                                                      \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = RADIO_MODE_NATIVE,                        \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,      \
    }

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000    /* 3000 millisecond */
#define LIGHT_ENDPOINT                  1     /* esp light bulb device endpoint, used to process light controlling commands */
#define MANUFACTURER                    "DIY"
#define MODELNAME                       "LedLight"
#define APP_PROFILE_ID                  0x0104 /* 0x0104 == Home Automation (HA) */
#define POWER_SOURCE                    3 /* 0x03 == battery */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask use in the example */


static void set_zcl_string(char *buffer, const char *value)
{
    buffer[0] = (char) strlen(value);
    memcpy(buffer + 1, value, buffer[0]);
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        log_i("Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            log_i("Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /* commissioning failed */
            log_w("Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            log_i("Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            log_i("Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        log_i("ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
            break;

        case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
            ret = zb_read_attribute_handler((esp_zb_zcl_cmd_read_attr_resp_message_t*) message);
            break;

        default:
            log_w("Receive Zigbee action(0x%x) callback", callback_id);
            break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // list of endpoints of this device/node
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();


    /////////////////////////////////////////////////////
    //   Create endpoint for LED light
    /////////////////////////////////////////////////////
  
    // clusters of the light endpoint
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // initalize the cluster cfg with defaults for a light
    esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();

    // Basic cluster
    // 

    // indicate use of battery instead of default value
    light_cfg.basic_cfg.power_source = POWER_SOURCE;
    esp_zb_attribute_list_t *cluster_basic = esp_zb_basic_cluster_create(&light_cfg.basic_cfg);

    char buffer[16];
    set_zcl_string(buffer, MANUFACTURER);
    esp_zb_basic_cluster_add_attr(cluster_basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, buffer);
    set_zcl_string(buffer, MODELNAME);
    esp_zb_basic_cluster_add_attr(cluster_basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, buffer);

    esp_zb_cluster_list_add_basic_cluster(cluster_list, cluster_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Identify Cluster
    //
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&light_cfg.identify_cfg);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Groups Cluster
    //
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_groups_cluster_create(&light_cfg.groups_cfg);
    esp_zb_cluster_list_add_groups_cluster(cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Scenes Cluster
    //
    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(&light_cfg.scenes_cfg);
    esp_zb_cluster_list_add_groups_cluster(cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // On/Off Cluster
    //
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&light_cfg.on_off_cfg);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Level Cluster
    //
    esp_zb_attribute_list_t *esp_zb_level_cluster = esp_zb_level_cluster_create(&light_cfg.level_cfg);
    esp_zb_level_cluster_add_attr(esp_zb_level_cluster, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, &light_level);
    esp_zb_cluster_list_add_level_cluster(cluster_list, esp_zb_level_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Device Temp Cluster
    //
    // esp_zb_attribute_list_t *device_temp_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_DEVICE_TEMP_CONFIG);
    // int16_t int_temp = 55;
    // esp_zb_cluster_add_attr(device_temp_cluster, (uint16_t)0x0000, &int_temp);

    // Time cluster
    //
    esp_zb_attribute_list_t *time_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME);
    esp_zb_cluster_list_add_time_cluster(cluster_list, time_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);


    // add clusters to endpoint
    esp_zb_ep_list_add_ep(ep_list, cluster_list, LIGHT_ENDPOINT, APP_PROFILE_ID, ESP_ZB_HA_LEVEL_CONTROLLABLE_OUTPUT_DEVICE_ID);


    /////////////////////////////////////////////////////
    //   Create endpoint for button
    /////////////////////////////////////////////////////

    // esp_zb_cluster_list_t *btn_cluster_list = esp_zb_zcl_cluster_list_create();

    // esp_zb_on_off_switch_cfg_t onoffswitch_config = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    // onoffswitch_config.basic_cfg.power_source = POWER_SRC_BAT;

    // // not sure if could reuse basic cluster from endpoint-1
    // esp_zb_attribute_list_t *btn_cluster_basic = esp_zb_basic_cluster_create(&onoffswitch_config.basic_cfg);

    // esp_zb_cluster_list_add_basic_cluster(btn_cluster_list, btn_cluster_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // // on/off switch cluster
    // //
    // esp_zb_on_off_switch_cluster_cfg_t cluster_onoffswitch_config = {
    //   .switch_type = ESP_ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
    //   .switch_action = ESP_ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_ACTIONS_TYPE1
    // };
    // esp_zb_attribute_list_t *onoff_switch_cluster = esp_zb_on_off_switch_cfg_cluster_create(&cluster_onoffswitch_config);
    // esp_zb_cluster_list_add_on_off_switch_config_cluster(btn_cluster_list, onoff_switch_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // // add clusters of btn endpoint to endpoint
    // esp_zb_ep_list_add_ep(ep_list, btn_cluster_list, SWITCH_ENDPOINT, APP_PROFILE_ID, ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID);


    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    //Erase NVRAM before creating connection to new Coordinator
    esp_zb_nvram_erase_at_start(true); //Comment out this line to erase NVRAM data if you are conneting to new Coordinator

    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_main_loop_iteration();
}

/* Handle the light attribute */

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    if(!message){
      log_e("Empty message");
    }
    if(message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS){
      log_e("Received message: error status(%d)", message->info.status);
    }

    log_i("Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

    if (message->info.dst_endpoint != LIGHT_ENDPOINT) {
      return ESP_ERR_INVALID_ARG;
    }

    switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY: {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
              uint16_t identifyTime = * (uint16_t*) message->attribute.data.value;
              log_i("Identify for %ds", identifyTime);
            }
            break;
        }
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF: {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                log_i("Light sets to %s", light_state ? "On" : "Off");
                neopixelWrite(LED_PIN,255*light_state,255*light_state,255*light_state); // Toggle light
            }
            break;
        }
        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL: {
          
          break;
        }
        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL: {
            if (message->attribute.id ==  ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 255;
                log_i("Level set to %d", light_level);
                neopixelWrite(LED_PIN, light_level * light_state, light_level * light_state, light_level * light_state);
            }
            break;
        }
    }

    return ret;
}


static esp_err_t zb_read_attribute_handler(esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    log_i("Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->variables->attribute.id, message->variables->attribute.data.size);

    return ESP_OK;
}


static void send_btn() {
    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.src_endpoint = LIGHT_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
    log_i("Send 'on_off off' command");
    esp_zb_zcl_on_off_cmd_req(&cmd_req);
}


static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR btn_isr()
{
    int state = digitalRead(BTN_PIN);
    xQueueSendFromISR(gpio_evt_queue, &state, NULL);
}


static void btn_task(void *pvParameters) {
  int last_state = 1;
  int press_stamp = 0;
  int release_stamp = 0;
  int fast_stamp = 0;
  int fast_count = 0;

  for (;;) {
    int btn_state;
    xQueueReceive(gpio_evt_queue, &btn_state, portMAX_DELAY);
    //int btn_state = gpio_get_level(BTN_PIN);
    if (btn_state == 0 && last_state == 1) {
      // press event
      press_stamp = millis();
      if (fast_stamp == 0) {
        // beginning of fast press
        fast_stamp = press_stamp;
      } else if (press_stamp - fast_stamp > 1000) {
        // too slow for fast press
        fast_stamp = 0;
        fast_count = 0;
      }
    }
    else if (btn_state == 1 && last_state == 0) {
      release_stamp = millis();

      if (release_stamp - press_stamp < 500) {
        log_i("Short press");
        send_btn();
      }
      else {
        log_i("Long press");
        esp_zb_factory_reset();
      }
    }

    last_state = btn_state;
  }
}


/********************* Arduino functions **************************/
void setup() {
    temperature_sensor_config_t temp_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    temperature_sensor_install(&temp_config, &tempsensor);
    temperature_sensor_enable(tempsensor);

    // Init Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Init RMT and leave light OFF
    neopixelWrite(LED_PIN,0,0,0);

    // Start Zigbee task
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_PIN), btn_isr, CHANGE);
    gpio_evt_queue = xQueueCreate(10, sizeof(int));

    // Start btn task
    xTaskCreate(btn_task, "Btn Task", 2048, NULL, 5, NULL);
}

void loop() {
  float temp = 0;
  temperature_sensor_get_celsius(tempsensor, &temp);
  
  //log_i("Temp %f", temp);
  //int16_t itemp = temp;
  //esp_zb_zcl_set_attribute_val(1, ZB_ZCL_CLUSTER_ID_DEVICE_TEMP_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_ATTR_DEVICE_TEMP_CONFIG_CURRENT_TEMP_ID, &itemp, false);)
  delay(5000);

}
