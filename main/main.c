#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "Fusion.h"

#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
QueueHandle_t xQueuePos;

typedef struct {
    int id;
    int dados;
} mpu_t;

void mpu6050_task(void *p) {
    // configuracao do I2C
    // i2c_init(i2c_default, 400 * 1000);
    // gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA_GPIO);
    // gpio_pull_up(I2C_SCL_GPIO);

    imu_c mpu;
    mpu.i2c = i2c_default;
    mpu.pin_scl = I2C_SCL_GPIO;
    mpu.pin_sda = I2C_SDA_GPIO;
    mpu.acc_scale = 0;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mpu6050_init(mpu);

    int16_t acceleration[3], gyro[3], temp;
    float acc_antigo = 0.0f;
    int last_yaw = 0.0f, last_roll = 0.0f;
    mpu_t info;
    float yaw_gyro = 0.0f;

    while(1) {
        // leitura da MPU, sem fusao de dados
        mpu6050_read_temp(mpu, &temp);
        mpu6050_read_gyro(mpu, gyro);
        mpu6050_read_acc(mpu, acceleration);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        }; 

        // printf("GYRO X: %0.0f| Y: %0.0f| Z: %0.0f \n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);

        // printf("ACCEL X: %0.0f| Y: %0.0f| Z: %0.0f \n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        if (yaw_gyro == 0.0f){
            yaw_gyro -= gyroscope.axis.z * SAMPLE_PERIOD;
            last_yaw = yaw_gyro;
        } else {
            yaw_gyro += gyroscope.axis.z * SAMPLE_PERIOD;
        }

        // printf("YAW: %0.0f \n", euler.angle.yaw);
        
        if (last_yaw != yaw_gyro){
            info.id = 0;
            info.dados = -yaw_gyro;
            last_yaw = yaw_gyro;
            xQueueSend(xQueuePos, &info, 0);
        }

        if (euler.angle.roll != last_roll){
            info.id = 1;
            info.dados = euler.angle.roll;
            last_roll = euler.angle.roll;
            xQueueSend(xQueuePos, &info, 0);
        }

        double acc = pow(accelerometer.axis.x, 2) + pow(accelerometer.axis.y, 2) + pow(accelerometer.axis.z, 2);
        acc = sqrt(acc);

        if ((acc - acc_antigo) > 0.65 && acc_antigo != 0.0f){
            info.id = 2;
            info.dados = 0;
            xQueueSend(xQueuePos, &info, 150);
        }

        acc_antigo = acc;

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void uart_task(void *p) {
    mpu_t data;

    while (1) {
        if (xQueueReceive(xQueuePos, &data, 100)){
            uint8_t bytes[4];
            bytes[0] = (uint8_t)(data.id);
            bytes[1] = (data.dados >> 8) & 0xFF;
            bytes[2] = data.dados & 0xFF;
            bytes[3] = 0xFF;

            uart_write_blocking(uart_default, bytes, 4);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


int main() {
    stdio_init_all();

    uart_init(uart_default, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", 4095, NULL, 1, NULL);
    xQueuePos = xQueueCreate(64, sizeof(mpu_t));

    vTaskStartScheduler();

    while (true)
        ;
}
