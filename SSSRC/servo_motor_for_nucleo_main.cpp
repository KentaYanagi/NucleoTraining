#include "mbed.h"

PwmOut servo(A5);
Serial pc(SERIAL_TX, SERIAL_RX);

float calc(float);
int main()
{
    float a,b;
    servo.period_ms(20);
    wait_ms(100);
    pc.printf("\r\n---servo test---\n\r");
    while(1) {
        pc.printf("degree(-90~90) : ");
        pc.scanf("%f",&a);
        if(-90<=a&&a<=90) {
            b=calc(a);
            pc.printf("%f PWM:%f\r\n",a,b);
            servo.pulsewidth(b);
        } else {
            break;
        }
    }
    return 0;
}
