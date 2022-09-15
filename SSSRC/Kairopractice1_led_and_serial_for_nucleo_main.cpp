#include "mbed.h"
DigitalOut led(LED3);
Serial pc(USBTX, USBRX);
int main()
{
    while(1) {
        for(int i=1; i<4; i++) {
            led=1;
            pc.printf("LED%d is ON\r\n",3);
            wait(i);
            led=0;
            pc.printf("LED%d is OFF\r\n",3);
            wait(i);
        }
    }
}