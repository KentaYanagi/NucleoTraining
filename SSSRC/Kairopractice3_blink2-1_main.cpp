#include "mbed.h"


DigitalOut motor(D13);

int main()
{
    while(1) {
        motor = 1;
        wait(10.0);
        motor = 0;
        wait(1.0);
    }
}