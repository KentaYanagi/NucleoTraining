#include "mbed.h"
#include "getGPS.h"

Serial pc(SERIAL_TX, SERIAL_RX);
GPS gps(D1, D0);

int main()
{
    pc.printf("\r\n\GPS Start\r\n");
    
    /* 1秒ごとに現在地を取得してターミナル出力 */
    while(1) {
        if(gps.getgps()) //現在地取得
            pc.printf("(%lf, %lf)\r\n", gps.latitude, gps.longitude);//緯度と経度を出力
        
        else
            pc.printf("No data\r\n");//データ取得に失敗した場合
        
        wait(1);
    }

    return 0;
}