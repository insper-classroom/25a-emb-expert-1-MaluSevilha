#include "mpu6050.h"

// Função que configura o struct de configuração do componente
void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale){

} 

// Configura pinos e periférico I2C
int mpu6050_init(imu_c config){

}

// Reinicia o dispositivo para o estado original
int mpu6050_reset(imu_c config){

}

// Faz a leitura do acelerômetro
int mpu6050_read_acc(imu_c config, int16_t accel[3]){

}

// Faz a leitura do giroscópio
int mpu6050_read_gyro(imu_c config, int16_t gyro[3]){

}

// Faz a leitura da temperatura
int mpu6050_read_temp(imu_c config, int16_t *temp){

}