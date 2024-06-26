#include "mbed.h"
#include "PID.hpp"

int suuti = 0;
int gohan = 0;
int16_t sokudo = 0;
int16_t sokudo1 = 0;
int mokuhyou = 0;
BufferedSerial pc(USBTX, USBRX, 115200);
CAN can(PA_11, PA_12, (int)1e6);
uint8_t DATA[8] = {};

// PID controller parameters
const float kp = 0.4;
const float ki = 0.5;
const float kd = 0.001;
const float sample_time = 0.02; // 20ms sample time

// Create PID controller
PID pid_controller(kp, ki, kd, sample_time);
PID pid_controller1(kp, ki, kd, sample_time);

int main()
{
    while (1)
    {
        CANMessage msg, msg1, msg2;
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
            else if (buf == 'm')
            {
                mokuhyou = -9000;
            }
            else if (buf == 'o')
            {
                mokuhyou = 0;
            }
        }

        // Calculate PID output
        float output = pid_controller.calculate(mokuhyou, sokudo);
        float output1 = pid_controller1.calculate(mokuhyou, sokudo1);

        int16_t output_int16 = static_cast<int16_t>(output);
        DATA[0] = output_int16 >> 8;   // MSB
        DATA[1] = output_int16 & 0xFF; // LSB

        int16_t output1_int16 = static_cast<int16_t>(output1);
        DATA[2] = output1_int16 >> 8;   // MSB
        DATA[3] = output1_int16 & 0xFF; // LSB

        CANMessage msg0(0x200, DATA, 8);
        can.write(msg0);
        if (can.read(msg1) && msg1.id == 0x201)
        { // CAN ID 1のメッセージをチェック
            // msg.dataを適切な変数に格納
            
            sokudo = (msg1.data[2] << 8) | msg1.data[3];
            // dataValueを使用した処理...
        }
        if (can.read(msg2) && msg2.id == 0x202)
        {
            // int8_t h_rank = msg2.data[2];
            // int8_t l_rank = msg2.data[3];
            sokudo1 = (msg2.data[2] << 8) | msg2.data[3];
        }
        printf("sokudo = %d , sokudo1 = %d\n", sokudo,sokudo1);
        ThisThread::sleep_for(20ms);
    }
}
