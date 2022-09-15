#include "mbed.h"
#include "MPU6050.h"

DigitalOut myled(LED1);
MPU6050 mpu(D4,D5);

int main() {
    while(1){
        int gyro[3];
        mpu.readGyroData(gyro); //gyro[3]に代入
        printf("x:%d y:%d z:%d\r\n",gyro[0],gyro[1],gyro[2]);
        wait(0.1);
    }
    
    /* おまけ 加速度
    while(1){
        int accel[3];
        mpu.readAccelData(acc);
        printf("x:%d y:%d z:%d\r\n",accel[0],accel[1],accel[2]);
    }
    */
    
}
