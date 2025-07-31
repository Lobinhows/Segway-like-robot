#ifndef MPU_Lib_H
#define MPU_Lib_H

#include <driver/i2c_master.h>


enum sensor_range {
    R2_250 = 0b00000000,
    R4_500 = 0b00001000,
    R8_1000 = 0b00010000,
    R16_2000 = 0b00011000
};


struct SensorData{
    int16_t AccelX;
    int16_t AccelY;
    int16_t AccelZ;
    int16_t Temp;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;           
};

void MPU_Initialize(i2c_master_bus_handle_t *i2c_handle, i2c_master_dev_handle_t *dev_handle, uint8_t addr, uint8_t filter, enum sensor_range A_Scale ,enum sensor_range G_Scale);
struct SensorData read_Data(i2c_master_dev_handle_t *dev_handle);
void read_Data_Raw(i2c_master_dev_handle_t *dev_handle, uint8_t* data);

#endif