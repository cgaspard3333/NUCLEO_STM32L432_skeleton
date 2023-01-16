#include "mbed.h"
#include "shell.h"

Ticker ticker;
CAN can(PA_11, PA_12,1000000); // RX, TX, baud rate
DigitalOut led3(LED3);
DigitalOut led2(LED2);

static BufferedSerial pc(USBTX, USBRX, 115200);

SHELL_PARAMETER_INT(ID, "ID_motor", 321)

void send() {
    char command[8];
    command[0] = '\x30';
    command[1] = '\x00';
    command[2] = '\x00';
    command[3] = '\x00';
    command[4] = '\x00';
    command[5] = '\x00';
    command[6] = '\x00';
    command[7] = '\x00';

    CANMessage s_msg = CANMessage(ID, command, 8, CANData, CANStandard);

    if (can.write(s_msg)) {
        shell_print("Message sent:");
        shell_print(s_msg.id);
        shell_print("|");
        shell_print(s_msg.data[0]);
        shell_print(s_msg.data[1]);
        shell_print(s_msg.data[2]);
        shell_print(s_msg.data[3]);
        shell_print(s_msg.data[4]);
        shell_print(s_msg.data[5]);
        shell_print(s_msg.data[6]);
        shell_print(s_msg.data[7]);
        shell_println();

        led3 = !led3;
    } 


}

SHELL_COMMAND(send, "Send")
{
    send();
}

int main() {
    shell_init(&pc);
    CANMessage msg;

    while(1){
        if(can.read(msg)) {
            shell_print("Message received:");
            shell_print(msg.data[0]);
            shell_print(msg.data[1]);
            shell_print(msg.data[2]);
            shell_print(msg.data[3]);
            shell_print(msg.data[4]);
            shell_print(msg.data[5]);
            shell_print(msg.data[6]);
            shell_print(msg.data[7]);
            shell_println();
            led3 = !led3;
        }
         ThisThread::sleep_for(200ms);

    }

}