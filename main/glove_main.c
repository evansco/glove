#include <stdio.h>
#include <stdint.h>
#include "driver/i2c.h"

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

uint8_t camera_init_vals[CAMERA_INIT_NUM_CMDS][CAMERA_INIT_CMD_LEN] = {
    {0x30, 0x01},
    {0x30, 0x08},
    {0x06, 0x90},
    {0x08, 0xC0},
    {0x1A, 0x40},
    {0x33, 0x33}
};

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

void camera_task()
{
    printf("Starting...\n");
    printf("Initializing master I2C...\n");
    ESP_ERROR_CHECK(i2c_master_init());
    printf("Initializing camera...\n");
    ESP_ERROR_CHECK(camera_init());

    uint8_t buf[16];
    int pos[2];
    for (;;) {
        printf("Reading from camera...\n");
        ESP_ERROR_CHECK(camera_read(buf));
        for (unsigned i = 0; i < 16; ++i) {
            printf("%x ", buf[i]);
        }
        printf("\n");
        camera_data_proc(buf, pos);
        printf("X: %d\tY: %d\n", pos[0], pos[1]);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void app_main()
{
    printf("Starting...\n");
    xTaskCreate(camera_task, "camera_task", 1024 * 2, NULL, 10, NULL);
}
