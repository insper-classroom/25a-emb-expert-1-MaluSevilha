#include "mpu6050.h"

// Função que configura o struct de configuração do componente
void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale){
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
} 

// Configura pinos e periférico I2C
int mpu6050_init(imu_c config){
    // reset device to its default state
    i2c_init(config.i2c, 400 * 1000);
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);

    // Define a acc_scale
    uint8_t buf_write[2];
    buf_write[0] = MPUREG_ACCEL_CONFIG;     // registrador
    buf_write[1] = config.acc_scale << 3;   // valor
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, buf_write, 2, false);
}

// Reinicia o dispositivo para o estado original
int mpu6050_reset(imu_c config){
    uint8_t buf_write[2];
    buf_write[0] = MPUREG_PWR_MGMT_1; // registrador
    buf_write[1] = 1 << 7;            // valor
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, buf_write, 2, false);
}

// Faz a leitura do acelerômetro
int mpu6050_read_acc(imu_c config, int16_t accel[3]){
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

// Faz a leitura do giroscópio
int mpu6050_read_gyro(imu_c config, int16_t gyro[3]){
    uint8_t buffer[6];

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    uint8_t val = 0x43;
    i2c_write_blocking(i2c_default, MPU6050_I2C_DEFAULT, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_DEFAULT, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
}

// Faz a leitura da temperatura
int mpu6050_read_temp(imu_c config, int16_t *temp){
    uint8_t buffer[6];

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    uint8_t val = 0x41;
    i2c_write_blocking(i2c_default, MPU6050_I2C_DEFAULT, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_DEFAULT, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}