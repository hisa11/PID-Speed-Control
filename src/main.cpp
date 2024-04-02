#include "mbed.h"
#include "PID.hpp"

int suuti = 0;
int gohan = 0;
int sokudo = 0;
int mokuhyou = 0;
BufferedSerial pc(USBTX, USBRX, 115200);
CAN can(PA_11, PA_12, (int)1e6);
uint8_t DATA[8] = {};

// PID controller parameters
const float kp = 0.01;
const float ki = 0.014;
const float kd = 0.07;
const float sample_time = 0.02; // 20ms sample time

// Create PID controller
PID pid_controller(kp, ki, kd, sample_time);

int main()
{
    while (1)
    {
        CANMessage msg;
        if (pc.readable())
        {
            char buf;
            pc.read(&buf, sizeof(buf));
            if (buf == 'w')
            {
                mokuhyou = 5000;
            }
            else if (buf == 's')
            {
                mokuhyou = 1000;
            }
            else if (buf == 'a')
            {
                mokuhyou = 9000;
            }
            else if(buf == 'm'){
                mokuhyou = -9000;
            }
            else if(buf == 'o'){
                mokuhyou =0;
            }
        }


        // Calculate PID output
        float output = pid_controller.calculate(mokuhyou, sokudo);

        int16_t output_int16 = static_cast<int16_t>(output);
        DATA[0] = output_int16 >> 8; // MSB
        DATA[1] = output_int16 & 0xFF; // LSB

        CANMessage msg0(0x200, DATA, 8);
        can.write(msg0);
        if (can.read(msg))
        {
            sokudo = (msg.data[2] << 8) | msg.data[3];
            if(sokudo > 40000){
                sokudo = sokudo - 65536;
            }
            printf("sokudo : %d\n", sokudo);
        }
        ThisThread::sleep_for(20ms);
    }
}
