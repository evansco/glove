#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include <float.h>

#define GLOVE_TAG "GLOVE"
#define SPP_TAG "SPP_INITIATOR_DEMO"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_INITIATOR"

#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA 26
#define I2C_MASTER_SCL 25
#define I2C_FREQ_HZ 19200
#define ACK_CHECK_EN 0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define CAMERA_ADDR 0x58

#define CAMERA_INIT_CMD_LEN 2
#define CAMERA_INIT_NUM_CMDS 6

// IMU Stuff
#define CTRL_REG1_G 0x10
#define CTRL_REG2_G 0x11
#define CTRL_REG3_G 0x12
#define CTRL_REG4 0x1e
#define ORIENT_CFG_G 0x13
#define IMU_AG_ADDR 0x6B // Address of gyro and accel
#define IMU_GYRO_START 0x18
#define IMU_ACCEL_START 0x28

// ADC
#define DEFAULT_VREF 3300 // 3.3V
#define NO_OF_SAMPLES 64
#define ADC_UNIT ADC_UNIT_1
#define FLEX1_CHANNEL ADC_CHANNEL_6 // ADC1: GPIO34, ADC2: GPIO14
#define FLEX2_CHANNEL ADC_CHANNEL_7 // ADC1: GPIO35, ADC2: GPIO27
#define FLEX1_ATTEN ADC_ATTEN_DB_0 // None
#define FLEX2_ATTEN ADC_ATTEN_DB_6 // 1/2
#define FLEX1_BENT_THRESHOLD 600
#define FLEX1_UNBENT_THRESHOLD 400
#define FLEX2_BENT_THRESHOLD 2000
#define FLEX2_UNBENT_THRESHOLD 1500

#define SCALE_X (640.f / 1024.f)
#define SCALE_Y (480.f / 768.f)

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;

static esp_bd_addr_t peer_bd_addr;
static const char remote_device_name[] = "HC-06";
static const esp_bd_addr_t remote_device_addr = {0x98, 0xd3, 0x31, 0xfb, 0x3c, 0x13};
static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 0x30;
static const uint8_t inq_num_rsps = 0;

uint8_t camera_init_vals[CAMERA_INIT_NUM_CMDS][CAMERA_INIT_CMD_LEN] = {
    {0x30, 0x01},
    {0x30, 0x08},
    {0x06, 0x90},
    {0x08, 0xC0},
    {0x1A, 0x40},
    {0x33, 0x33}
};

TaskHandle_t ct_handle;

#define THRESHOLD 100

typedef struct {
    int16_t gyro[3];
    int16_t accel[3];
} imu_t;

uint8_t imu_who_am_i() {
    uint8_t whoami;
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IMU_AG_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xF, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IMU_AG_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &whoami, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        ESP_LOGW(GLOVE_TAG, "FAIL1");
    ESP_LOGI(GLOVE_TAG, "Who am I: %d\n", whoami);
    return whoami;
}

int imu_write_byte(uint8_t addr, uint8_t subaddr, uint8_t data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, subaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

int init_imu() {
    int ret;
    int reg = (0x7 << 5) | 0x3;
    ret = imu_write_byte(IMU_AG_ADDR, CTRL_REG1_G, reg);
    if (ret != ESP_OK) {
        ESP_LOGW(GLOVE_TAG, "Failed gyro init\n");
        return ret;
    }

    //reg = 0b00000010;
    reg = 0;
    ret = imu_write_byte(IMU_AG_ADDR, CTRL_REG2_G, reg);
    if (ret != ESP_OK) {
        ESP_LOGW(GLOVE_TAG, "Failed gyro init\n");
        return ret;
    }

    //reg = 0b01000001;
    reg = 0;
    ret = imu_write_byte(IMU_AG_ADDR, CTRL_REG3_G, reg);
    if (ret != ESP_OK) {
        ESP_LOGW(GLOVE_TAG, "Failed gyro init\n");
        return ret;
    }

    reg = 7 << 3;
    ret = imu_write_byte(IMU_AG_ADDR, CTRL_REG4, reg);
    if (ret != ESP_OK) {
        ESP_LOGW(GLOVE_TAG, "Failed gyro init\n");
        return ret;
    }

    reg = 0;
    ret = imu_write_byte(IMU_AG_ADDR, ORIENT_CFG_G, reg);
    if (ret != ESP_OK) {
        ESP_LOGW(GLOVE_TAG, "Failed gyro init\n");
        return ret;
    }

    imu_who_am_i();
    vTaskDelay(100 / portTICK_RATE_MS);

    return ret;
    
}

uint8_t imu_sensor_avail() {
    int ret;
    uint8_t status;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IMU_AG_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x17, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IMU_AG_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &status, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        ESP_LOGW(GLOVE_TAG, "STATFAIL2");
    return status & 11;
}

int imu_read_sensor(uint8_t saddr, int16_t* buffer) {
    int ret;
    uint8_t last_byte;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IMU_AG_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, saddr | 0x80, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IMU_AG_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, (uint8_t*) buffer, 5, ACK_VAL);
    i2c_master_read_byte(cmd, &last_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    buffer[2] |= last_byte << 8;
    if (ret != ESP_OK)
        ESP_LOGW(GLOVE_TAG, "FAIL2");
    return ret;
}

imu_t* read_imu() {
    int ret = ESP_OK;
    imu_t* imu = (imu_t*) malloc(sizeof(imu_t));
    uint8_t avail = imu_sensor_avail(); 
    if (avail & 1) {
        ret = imu_read_sensor(IMU_ACCEL_START, imu->accel);
        if (ret != ESP_OK) {
            free(imu);
            ESP_LOGW(GLOVE_TAG, "Failed to read accel data\n");
            return NULL;
        }
    } else {
        ESP_LOGW(GLOVE_TAG, "Accel data unavailable\n");
    }


    if (avail & 2) {
        ret = imu_read_sensor(IMU_GYRO_START, imu->gyro);
        if (ret != ESP_OK) {
            free(imu);
            ESP_LOGW(GLOVE_TAG, "Failed to read gyro data\n");
            return NULL;
        }
    } else {
        ESP_LOGW(GLOVE_TAG, "Gyro data unavailable\n");
    }

    return imu;
}

bool is_valid(int x, int y) {
    static int x_prev = 0x3FF;
    static int y_prev = 0x3FF;
    static int dx_prev = 0x3FF;
    static int dy_prev = 0x3FF;
    static float theta_prev = FLT_MAX;
    
    if (y == 0x3FF) {
        return false;
    }

    if (y_prev == 0x3FF) {
        x_prev = x;
        y_prev = y;
        return false;
    }

    int dx = abs(x - x_prev);
    int dy = abs(y - y_prev);
    float theta = atan2(dy, dx);

    if (theta_prev == FLT_MAX) {
        x_prev = x;
        y_prev = y;
        dx_prev = dx;
        dy_prev = dy;
        theta_prev = theta;
        return false;
    }

    if (abs(dx - dx_prev) < 150 
            && abs(dy - dy_prev) < 150
            && fabsf(theta - theta_prev) < 1.0) {
        x_prev = x;
        y_prev = y;
        dx_prev = dx;
        dy_prev = dy;
        theta_prev = theta;
        return true;
    } else {
        y_prev = 0x3FF;
        theta_prev = FLT_MAX;
        return false;
    }
}

#define MEDIAN_FILTER_SIZE 3

typedef struct {
    int x;
    int y;
    int dist;
} point_t;

point_t* get_median(point_t** points, int size) {
    // Copy the array of point pointers
    point_t* arr[size];
    for (int i = 0; i < size; ++i) {
        arr[i] = points[i];
    }

    // Insertion sort
    int i, j; 
    point_t* key;
    for (i = 1; i < size; i++) { 
        key = arr[i]; 
        j = i - 1; 
  
        /* Move elements of arr[0..i-1], that are 
          greater than key, to one position ahead 
          of their current position */
        while (j >= 0 && arr[j]->dist > key->dist) { 
            arr[j + 1] = arr[j]; 
            j = j - 1; 
        } 
        arr[j + 1] = key; 
    }

    return arr[(size - 1) / 2];
}

point_t* median_filter(int x, int y) {
    static point_t* points[MEDIAN_FILTER_SIZE];
    static int size = 0;
    static int x_prev = 0x3FF;
    static int y_prev = 0x3FF;

    // Check if point is valid, if not return NULL
    if (x == 0x3FF || y == 0x3FF) {
        x_prev = 0x3FF;
        y_prev = 0x3FF;
        return NULL;
    }

    // Calculate the distance to the new point
    point_t* new_point = (point_t*)malloc(sizeof(point_t));
    if (size == 0) {
        x_prev = x;
        y_prev = y;
    }
    int dx = x - x_prev;
    int dy = y - y_prev;
    new_point->dist = (dx * dx) + (dy * dy);
    new_point->x = x;
    new_point->y = y;

    // If size < 3, just push in the new point
    if (size < MEDIAN_FILTER_SIZE) {
        points[size++] = new_point;
        if (size < 3) {
            return new_point;
        } else {
            return get_median(points, size);
        }
    }
    
    free(points[0]);
    for (int i = 0; i < MEDIAN_FILTER_SIZE - 1; ++i) {
        points[i] = points[i + 1];
    }
    points[MEDIAN_FILTER_SIZE - 1] = new_point;

    return get_median(points, size);
}

/*
 * I2C initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              0, 0, 0);
}

/*
 * Camera initialization
 */
static esp_err_t camera_init()
{
    int ret;
    for (unsigned i = 0; i < CAMERA_INIT_NUM_CMDS; ++i) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, CAMERA_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write(cmd, camera_init_vals[i], CAMERA_INIT_CMD_LEN, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    vTaskDelay(100 / portTICK_RATE_MS);
    return ret;
}

/*
 * Read from camera
 * Expects that buf is an array of size 16
 */
static esp_err_t camera_read(uint8_t* buf) {
    // Write 0x36 to camera
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CAMERA_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x36, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // Read 16 bytes from camera
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, CAMERA_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, buf, 15, ACK_VAL);
    i2c_master_read_byte(cmd, &buf[15], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * Get position from camera data
 * Expects that data is an array of size 16,
 *   and pos is an array of size 2
 */
static esp_err_t camera_data_proc(uint8_t* data, int* pos)
{
    pos[0] = data[1] + ((data[3] & 0x30) << 4);
    pos[1] = data[2] + ((data[3] & 0xC0) << 2);
    return ESP_OK;
}

uint8_t get_bent(adc_channel_t channel, uint32_t adc_reading) {
    static uint8_t bent_draw = 0;
    static uint8_t bent_erase = 0;
    
    if (channel == FLEX1_CHANNEL) {
        ESP_LOGI(GLOVE_TAG, "Draw ADC Reading: %d", adc_reading);
        if (bent_draw) {
            if (adc_reading <= FLEX1_UNBENT_THRESHOLD) {
                bent_draw = 0;
            }
        } else {
            if (adc_reading >= FLEX1_BENT_THRESHOLD) {
                bent_draw = 1;
            }
        }
        return bent_draw;
    } else {
        ESP_LOGI(GLOVE_TAG, "Erase ADC Reading: %d", adc_reading);
        if (bent_erase) {
            if (adc_reading <= FLEX2_UNBENT_THRESHOLD) {
                bent_erase = 0;
            }
        } else {
            if (adc_reading >= FLEX2_BENT_THRESHOLD) {
                bent_erase = 1;
            }
        }
        return bent_erase;
    }
}

static void adc_init()
{
    //esp_adc_cal_characteristics_t *adc_chars;
    if (ADC_UNIT == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_12Bit);
        adc1_config_channel_atten(FLEX1_CHANNEL, FLEX1_ATTEN);
        adc1_config_channel_atten(FLEX2_CHANNEL, FLEX2_ATTEN);
    } else {
        adc2_config_channel_atten(FLEX1_CHANNEL, FLEX1_ATTEN);
        adc2_config_channel_atten(FLEX2_CHANNEL, FLEX2_ATTEN);
    }

    //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    /*esp_adc_cal_value_t val_type = *///esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

static uint32_t sample_adc(adc_channel_t channel) {
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (ADC_UNIT == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t) channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t) channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    return adc_reading;
}

uint8_t color_state = 0;
uint8_t update_color_select(int16_t gx) {
    static bool pressed = false;
    static int count = 0;
    static const int PRESS_THRESHOLD = -30000;
    static const int RELEASE_THRESHOLD = 20000;
    if (!pressed && gx < PRESS_THRESHOLD) {
        //pressed = true;
        count++;
        //color_state = !color_state;
        if (count >= 1) {
            pressed = true;
            color_state = !color_state;
        }
    } else if (pressed && gx > RELEASE_THRESHOLD) {
        pressed = false;
        count = 0;
    }
    ESP_LOGI(GLOVE_TAG, "Color select: %d", color_state);
    return color_state << 2;
}

uint8_t get_color_select() {
    return color_state << 2;
}

uint8_t get_reset(int16_t gy) {
    static bool pressed = false;
    static int count = 0;
    static const int PRESS_THRESHOLD = -30000;
    static const int RELEASE_THRESHOLD = 20000;
    if (!pressed && gy < PRESS_THRESHOLD) {
        //pressed = true;
        count++;
        if (count >= 1) {
            pressed = true;
            return 1 << 3;
        }
        ESP_LOGI(GLOVE_TAG, "Reset detected");
        //return 1 << 3;
    } else if (pressed && gy > RELEASE_THRESHOLD) {
        pressed = false;
        count = 0;
    }
    return 0 << 3;
}

uint8_t get_mode_toggle(int16_t gx) {
    static bool pressed = false;
    static int count = 0;
    static const int PRESS_THRESHOLD = -30000;
    static const int RELEASE_THRESHOLD = 20000;
    if (!pressed && gx < PRESS_THRESHOLD) {
        //pressed = true;
        count++;
        if (count >= 1) {
            pressed = true;
            return 1 << 4;
        }
        ESP_LOGI(GLOVE_TAG, "Mode toggle detected");
        //return 1 << 4;
    } else if (pressed && gx > RELEASE_THRESHOLD) {
        pressed = false;
        count = 0;
    }
    return 0 << 4;
}

void measure_task(void* spp_handle)
{
    uint8_t buf[4];
    int pos[2];
    for (;;) {
        // Reset the buffer
        memset(buf, 0, 4);

        // Read from the camera
        ESP_LOGI(GLOVE_TAG, "Reading from camera...\n");
        ESP_ERROR_CHECK(camera_read(buf));
        for (unsigned i = 0; i < 16; ++i) {
            //ESP_LOGI(GLOVE_TAG, "%x", buf[i]);
        }
        camera_data_proc(buf, pos);
        //ESP_LOGI(GLOVE_TAG, "X: %d\tY: %d\n", pos[0], pos[1]);
        
        // Get state of flex sensors
        uint8_t bent_draw = get_bent(FLEX1_CHANNEL, sample_adc(FLEX1_CHANNEL));
        uint8_t bent_erase = get_bent(FLEX2_CHANNEL, sample_adc(FLEX2_CHANNEL));
        buf[3] = 0;
        /*if (bent_draw && bent_erase) {
            buf[3] = 0;
        } else {
            buf[3] = bent_draw | (bent_erase << 1);
        }*/
        if (bent_draw ^ bent_erase) {
            buf[3] = (!bent_draw) | (!bent_erase << 1);
        }
        
        imu_t* imu = read_imu();
        if (!imu) {
            ESP_LOGW(GLOVE_TAG, "Spaghett");
            continue;
        }
        ESP_LOGI(GLOVE_TAG, "accel: (%d, %d, %d)\n", imu->accel[0], imu->accel[1], imu->accel[2]);
        ESP_LOGI(GLOVE_TAG, "gyro: (%d, %d, %d)\n", imu->gyro[0], imu->gyro[1], imu->gyro[2]);
        //continue;
        

        bool valid = true;
        if (!is_valid(pos[0], pos[1])) {
            ESP_LOGW(GLOVE_TAG, "Point (%d, %d) detected as outlier. Thrown away.", pos[0], pos[1]);
            //free(imu);
            //continue;
            valid = false;
        }
        ESP_LOGI(GLOVE_TAG, "X: %d\tY: %d\n", pos[0], pos[1]);

        // Scale the aspect ratio of the camera
        pos[0] *= SCALE_X;
        pos[1] = (768 - pos[1]) * SCALE_Y;
        ESP_LOGI(GLOVE_TAG, "Scaled X: %d\tScaled Y: %d\n", pos[0], pos[1]);

        // Draw, erase, and color select
        if (!bent_draw && !bent_erase) {
            buf[3] |= get_color_select() | get_mode_toggle(imu->gyro[0]);
        } else {
            buf[3] |= update_color_select(imu->gyro[0]) | get_reset(imu->gyro[1]);
        }
        // TODO: Remove
        //buf[3] = 1 | color_select;
        // x-position
        buf[2] = (pos[0] & 0x3F) << 2;
        buf[1] = (pos[0] & 0x3C0) >> 6;
        // y-position
        buf[1] |= (pos[1] & 0xF) << 4;
        buf[0] = (pos[1] & 0x3F0) >> 4;
        // x- and y-movement = true
        //buf[0] |= 0xC0;
        if (valid) {
            buf[0] |= 0xC0;
        }

        if (bent_draw) ESP_LOGI(GLOVE_TAG, "Draw got bent\n");
        else ESP_LOGI(GLOVE_TAG, "Draw did not get bent\n");
        if (bent_erase) ESP_LOGI(GLOVE_TAG, "Erase got bent\n");
        else ESP_LOGI(GLOVE_TAG, "Erase did not get bent\n");

        free(imu);

        esp_spp_write((uint32_t)spp_handle, 4, buf);
        vTaskDelay(5 / portTICK_RATE_MS);
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);

        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
        }
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        // TODO: Start data collection task
        xTaskCreate(measure_task, "measure_task", 1024 * 2, (void*)param->write.handle, 10, &ct_handle);
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        // TODO: Kill data collection task
        vTaskDelete(ct_handle);
        // Restart discovery
        esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");
        esp_log_buffer_char("Received data",param->data_ind.data, param->data_ind.len);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong=%d", param->cong.cong);
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len , param->write.cong);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
        esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        if (!memcmp(param->disc_res.bda, remote_device_addr, ESP_BD_ADDR_LEN)) {
            ESP_LOGI(GLOVE_TAG, "Found match\n");
            memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
            esp_spp_start_discovery(peer_bd_addr);
            esp_bt_gap_cancel_discovery();
        }
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    default:
        break;
    }
}

void app_main()
{
    ESP_LOGI(GLOVE_TAG, "Starting...\n");
    ESP_LOGI(GLOVE_TAG, "Initializing master I2C...\n");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(GLOVE_TAG, "Initializing camera...\n");
    ESP_ERROR_CHECK(camera_init());
    ESP_LOGI(GLOVE_TAG, "Initializing ADC %d\n", ADC_UNIT);
    adc_init();
    ESP_LOGI(GLOVE_TAG, "Initializing IMU...\n");
    init_imu();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}
