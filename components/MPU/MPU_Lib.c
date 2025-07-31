#include "MPU_Lib.h"


void MPU_Initialize(i2c_master_bus_handle_t *i2c_handle, 
                    i2c_master_dev_handle_t *dev_handle,
                    uint8_t addr, 
                    uint8_t filter, 
                    enum sensor_range A_Scale,
                    enum sensor_range G_Scale)
    {
    i2c_device_config_t MPU_config ={
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = addr,
        .scl_speed_hz = 400000
    };
    
    i2c_master_bus_add_device(*(i2c_handle),&MPU_config,dev_handle);

    uint8_t who_am_i = 0x75;
    i2c_master_transmit_receive(*(dev_handle),&who_am_i,1,&who_am_i,1,-1);
    if (who_am_i == 0x68){
        printf("MPU 6050 encontrado\n");
    }else if (who_am_i == 0x71){
        printf("MPU 9250 encontrado\n");
    }else { 
        printf("Erro\n");
    }
    // uint8_t data[] = {0x6B,0x01,0x1A,filter,0x1B,G_Scale,0x1C,A_Scale}; 
    // i2c_master_transmit(*(dev_handle),data,sizeof(data),-1);

    uint8_t data[] = {0x6B,0x01}; 
    i2c_master_transmit(*(dev_handle),data,sizeof(data),-1);
    data[0] = 0x1A;
    data[1] = filter; 
    i2c_master_transmit(*(dev_handle),data,sizeof(data),-1);
    data[0] = 0x1B;
    data[1] = G_Scale;
    i2c_master_transmit(*(dev_handle),data,sizeof(data),-1);
    data[0] = 0x1C;
    data[1] = A_Scale;
    i2c_master_transmit(*(dev_handle),data,sizeof(data),-1);
}

uint8_t get_accel_sensitivity(i2c_master_dev_handle_t *dev_handle){
    uint8_t init = 0x1B;
    uint8_t data = NULL;
    i2c_master_transmit_receive(*(dev_handle),&init,sizeof(init),data,sizeof(data),-1);
    return data;
}

struct SensorData read_Data(i2c_master_dev_handle_t *dev_handle){
    uint8_t init = 0x3B;
    uint8_t data[14];
  
    // for (i=0;i<=13;i++){
        i2c_master_transmit_receive(*(dev_handle),&init,sizeof(init),data,sizeof(data),-1);
               
    // }


    struct SensorData reads = {
        .AccelX = data[0] << 8 | data[1],
        .AccelY = data[2] << 8 | data[3],
        .AccelZ = data[4] << 8 | data[5],
        .Temp = data[6] << 8 | data[7],
        .GyroX = data[8] << 8 | data[9],
        .GyroY = data[10] << 8 | data[11],
        .GyroZ = data[12] << 8 | data[13]
    };
    
    return reads;
}

void read_Data_Raw(i2c_master_dev_handle_t *dev_handle, uint8_t* data){
    uint8_t init = 0x3B;
    i2c_master_transmit_receive(*(dev_handle),&init,sizeof(init),data,sizeof(uint8_t)*14,-1);
}