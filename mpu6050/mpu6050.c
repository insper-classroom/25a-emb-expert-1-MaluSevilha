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
    uint8_t buffer[8];
    uint8_t reg_address = MPUREG_ACCEL_XOUT_H;
    int accel_x = 0, accel_y = 0, accel_z = 0;

    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        accel_x += buffer[i] << (8+i);
    }

    reg_address = MPUREG_ACCEL_XOUT_L;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        accel_x += buffer[i] << i;
    }

    reg_address = MPUREG_ACCEL_YOUT_H;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        accel_y += buffer[i] << (8+i);
    }

    reg_address = MPUREG_ACCEL_YOUT_L;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        accel_y += buffer[i] << i;
    }

    reg_address = MPUREG_ACCEL_ZOUT_H;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        accel_z += buffer[i] << (8+i);
    }

    reg_address = MPUREG_ACCEL_ZOUT_L;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        accel_z += buffer[i] << i;
    }

    double accel = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z;
    accel = pow(accel, 0.5);

    return (int) accel;
}

// Faz a leitura do giroscópio
int mpu6050_read_gyro(imu_c config, int16_t gyro[3]){
    uint8_t buffer[8];
    uint8_t reg_address = MPUREG_GYRO_XOUT_H;
    int gyro_x = 0, gyro_y = 0, gyro_z = 0;

    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        gyro_x += buffer[i] << (8+i);
    }

    reg_address = MPUREG_GYRO_XOUT_L;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        gyro_x += buffer[i] << i;
    }

    reg_address = MPUREG_GYRO_YOUT_H;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        gyro_y += buffer[i] << (8+i);
    }

    reg_address = MPUREG_GYRO_YOUT_L;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        gyro_y += buffer[i] << i;
    }

    reg_address = MPUREG_GYRO_ZOUT_H;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        gyro_z += buffer[i] << (8+i);
    }

    reg_address = MPUREG_GYRO_ZOUT_L;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        gyro_z += buffer[i] << i;
    }

    double gyro = gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z;
    gyro = pow(gyro, 0.5);

    return (int) gyro;

}

// Faz a leitura da temperatura
int mpu6050_read_temp(imu_c config, int16_t *temp){
    uint8_t buffer[8];
    uint8_t reg_address = MPUREG_TEMP_OUT_H;
    int temp = 0;

    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        temp += buffer[i] << (8+i);
    }

    reg_address = MPUREG_TEMP_OUT_L;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &reg_address, 1, true); // true to keep master control of bus
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 1, false);

    for (int i = 0; i < 8; i++){
        temp += buffer[i] << i;
    }

    return temp;
}