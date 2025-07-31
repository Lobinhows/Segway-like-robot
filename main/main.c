#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "stdio.h"
#include "MPU_Lib.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "driver/uart.h"

#define MPU_ADDRESS 0x69
#define BUF_SIZE 256


//Creates a task for each LED color change
void task_red(){
    static int duty = 0;
    for(;;){
        duty --;
        if (duty<0){
            duty = 511;
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, duty);
        // Update duty to apply the new value
        ledc_update_duty(LEDC_LOW_SPEED_MODE, 0);
        // printf("Vermelho = %d\n", duty);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void task_green(){
    static int duty = 0;
    for(;;){
        duty --;
        if (duty<0){
            duty = 511;
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, duty);
        // Update duty to apply the new value
        ledc_update_duty(LEDC_LOW_SPEED_MODE, 1);
        // printf("Verde = %d\n", duty);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_blue(){
    static int duty = 0;
    for(;;){
        duty --;
        if (duty<0){
            duty = 511;
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 2, duty);
        // Update duty to apply the new value
        ledc_update_duty(LEDC_LOW_SPEED_MODE, 2);
        // printf("Azul = %d\n", duty);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}


i2c_master_bus_handle_t i2c_handle;
i2c_master_dev_handle_t dev_handle;
uint8_t accel_sensitivity = R8_1000;
uint8_t gyro_sensitivity = R2_250;

void task_accel(){
    MPU_Initialize(&i2c_handle,
                    &dev_handle,
                    MPU_ADDRESS,
                    0x01,    //Low pass filter (both gyro and accel)
                    accel_sensitivity,
                    gyro_sensitivity);

    for(;;){
        uint8_t signal;
        int aa;
        uart_get_buffered_data_len(UART_NUM_0, (size_t*)&aa); 
        if (aa > 0){
            uart_read_bytes(UART_NUM_0,&signal,sizeof(uint8_t),100);
        }

        // scanf("%hhu", &signal);
        // fflush(stdin);
        if (signal == 23){
            // fflush(stdin);
            struct SensorData readings = read_Data(&dev_handle);
            double a = (readings.AccelX/4096.0f)*9.80665;
            uart_write_bytes(UART_NUM_0,&a,sizeof(double));
            a = (readings.AccelY/4096.0f)*9.80665;
            uart_write_bytes(UART_NUM_0,&a,sizeof(double));
            a = (readings.AccelZ/4096.0f)*9.80665;
            uart_write_bytes(UART_NUM_0,&a,sizeof(double));
            a = (readings.GyroY/131);
            uart_write_bytes(UART_NUM_0,&a,sizeof(double));

            // uint8_t *bytes = (uint8_t*)&a;
            // for (int i = 0; i<sizeof(double);i++){   
            //     printf("%hhu", bytes[i]);
            // }
            // printf("\n");
                // printf("AccelX: %f\n",(readings.AccelX/4096.0f)*9.80665);
            // printf("AccelY: %f\n",(readings.AccelY/4096.0f)*9.80665);
            // printf("AccelZ: %f\n",(readings.AccelZ/4096.0f)*9.80665);
            // printf("Temp: %f\n",(readings.Temp/340.0f)+36.53);
            // printf("GyroX: %f\n",(readings.GyroX*1.0f));
            // printf("GyroY: %f\n",(readings.GyroY*1.0f));
            // printf("GyroZ: %f\n",(readings.GyroZ*1.0f));
            signal = NULL;    
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }  
}

void app_main() {
    // Configure PWM frequency and resolution
    ledc_timer_config_t ledc_timer = {
        .freq_hz = 4000,
        .duty_resolution = LEDC_TIMER_9_BIT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = 0,   
        .clk_cfg = LEDC_AUTO_CLK        
    };

    ledc_timer_config(&ledc_timer);

    //Configuring a channel for each LED
    ledc_channel_config_t led_red = {
        .channel = 0,
        .duty = 0,
        .gpio_num = 12,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = 0    
    };

    ledc_channel_config_t led_green = {
        .channel = 1,
        .duty = 0,
        .gpio_num = 13,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = 0    
    };

    ledc_channel_config_t led_blue = {
        .channel = 2,
        .duty = 0,
        .gpio_num = 14,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = 0    
    };
    
    ledc_channel_config(&led_red);
    ledc_channel_config(&led_green);
    ledc_channel_config(&led_blue);

    
    //Configuring the I2C bus
    i2c_master_bus_config_t accel_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    //Initializing I2C bus
    i2c_new_master_bus(&accel_config,&i2c_handle);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    // 3. Inicialização da UART
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE, 0, 0, NULL, 0);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Initializing each Task
    xTaskCreate(task_red,"Task_Red",2048,NULL,1,NULL);
    xTaskCreate(task_green,"Task_Green",2048,NULL,1,NULL);
    xTaskCreate(task_blue,"Task_Blue",2048,NULL,1,NULL);
    xTaskCreate(task_accel,"Task_Accel",2048,NULL,1,NULL);
}