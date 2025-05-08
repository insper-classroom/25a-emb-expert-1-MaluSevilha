#ifndef _MPU6050_H
#define _MPU6050_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define MPU6050_I2C_DEFAULT 0x68
#define MPUREG_WHOAMI 0x75     
#define MPUREG_SMPLRT_DIV 0x19 
#define MPUREG_CONFIG 0x1A     
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_FIFO_EN 0x23
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_INT_STATUS 0x3A
#define MPUREG_ACCEL_XOUT_H 0x3B 
#define MPUREG_ACCEL_XOUT_L 0x3C 
#define MPUREG_ACCEL_YOUT_H 0x3D 
#define MPUREG_ACCEL_YOUT_L 0x3E 
#define MPUREG_ACCEL_ZOUT_H 0x3F 
#define MPUREG_ACCEL_ZOUT_L 0x40 
#define MPUREG_TEMP_OUT_H 0x41   
#define MPUREG_TEMP_OUT_L 0x42   
#define MPUREG_GYRO_XOUT_H 0x43  
#define MPUREG_GYRO_XOUT_L 0x44  
#define MPUREG_GYRO_YOUT_H 0x45  
#define MPUREG_GYRO_YOUT_L 0x46  
#define MPUREG_GYRO_ZOUT_H 0x47  
#define MPUREG_GYRO_ZOUT_L 0x48  
#define MPUREG_USER_CTRL 0x6A    
#define MPUREG_PWR_MGMT_1 0x6B   
#define MPUREG_PWR_MGMT_2 0x6C   
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_PRODUCT_ID 0x0C

// no arquivo .h
typedef struct imu6050 {
    // configuração do I2C
    i2c_inst_t *i2c;
    int pin_sda;
    int pin_scl;

    // configuração do range do acelerômetro
    int acc_scale;
} imu_c;

// Função que configura o struct de configuração do componente
void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale); 

// Configura pinos e periférico I2C
int mpu6050_init(imu_c config);

// Reinicia o dispositivo para o estado original
int mpu6050_reset(imu_c config);

// Faz a leitura do acelerômetro
int mpu6050_read_acc(imu_c config, int16_t accel[3]);

// Faz a leitura do giroscópio
int mpu6050_read_gyro(imu_c config, int16_t gyro[3]);

// Faz a leitura da temperatura
int mpu6050_read_temp(imu_c config, int16_t *temp);

#endif