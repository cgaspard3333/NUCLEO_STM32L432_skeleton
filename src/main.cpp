#include "mbed.h"
#include "shell.h"

using namespace std::chrono;

CAN can(PA_11, PA_12,1000000); // RX, TX, baud rate
DigitalOut led3(LED3);

static BufferedSerial pc(USBTX, USBRX, 921600);

/// @brief Transforms Motor ID + Data into CANMessage (autocomplete data field by zeros)
/// @param ID Motor ID (1~32)
/// @param byte0 Data byte 0
/// @param byte1 Data byte 1
/// @param byte2 Data byte 2
/// @param byte3 Data byte 3
/// @param byte4 Data byte 4
/// @param byte5 Data byte 5
/// @param byte6 Data byte 6
/// @param byte7 Data byte 7
/// @return CANMessage to be send
CANMessage build_can_cmd(int ID, char byte0, char byte1 = 0, char byte2 = 0, char byte3 = 0, char byte4 = 0, char byte5 = 0, char byte6 = 0, char byte7 = 0){

    char raw_data[8] ={byte0,byte1,byte2,byte3,byte4,byte5,byte6,byte7};
    
    CANMessage can_cmd = CANMessage(320+ID, raw_data, 8, CANData, CANStandard); // Motor ID = 0x140 + ID(1~32) | 0x140 = 320

    return can_cmd;

}
/// @brief Read infos from the Motor corresponding to the command sent
/// @param cmd Command send to read infos from the Motor
/// @return Received CANMessage or 0 if no response received in time (>1ms)
CANMessage send_cmd(CANMessage cmd){
    Timer t;
    CANMessage rcv_msg;

    if (can.write(cmd)) {
        t.start();
    } 

    while(1){
        if(can.read(rcv_msg)) {
            return rcv_msg;
        }

        if(duration_cast<microseconds>(t.elapsed_time()).count()>1000 || can.tderror()>127 || can.rderror()>127){
            t.stop();
            can.reset();
            shell_print("Error : Reply not received in time");
            shell_println();
            return -1;
        }
    }
    
}

uint32_t read_data_4byte(CANMessage rcv_msg){
    uint8_t data0 = rcv_msg.data[4];
    uint8_t data1 = rcv_msg.data[5];
    uint8_t data2 = rcv_msg.data[6];
    uint8_t data3 = rcv_msg.data[7];

    uint32_t data = (data3 << 24) | (data2 << 16) | (data1 << 8) | data0;

    return data;
}

/// @brief Read PID parameters of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_pid_params(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x30');
    CANMessage rcv_msg = send_cmd(cmd);

    char params[][19] = {"Current Loop KP :", "Current Loop KI :", "Speed Loop KP :", "Speed Loop KI :", "Position Loop KP :", "Position Loop KI :"};

    for (size_t i = 0; i < 6; i++)
    {
        shell_print(params[i]);
        shell_print(rcv_msg.data[i+2]);
        shell_println();
    }
    
}

/// @brief Read Acceleration command of the position loop of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_acc_cmd(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x42');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Acceleration (°/s²) :");
    shell_print((unsigned int)read_data_4byte(rcv_msg));
    shell_println();

}

/// @brief Read Position command (pulses) of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_pos_pulse(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x60');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Position data (pulses) :");
    shell_print((unsigned int)read_data_4byte(rcv_msg));
    shell_println();    
}

/// @brief Read Position original command (pulses) without zero offset (initial position) of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_original_pos_pulse(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x61');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Position data without zero offset (pulses) :");
    shell_print((unsigned int)read_data_4byte(rcv_msg));
    shell_println();    
}

/// @brief Read Zero offset command (pulses) of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_zero_offset(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x62');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Zero offset command (pulses) :");
    shell_print((unsigned int)read_data_4byte(rcv_msg));
    shell_println();    
}

/// @brief Read the current absolute angle of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_angle(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x92');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Absolute angle position (°) :");

    shell_print((unsigned int)read_data_4byte(rcv_msg));
    shell_println();    
}

/// @brief Read the motor temperature, voltage and error status flags of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_status1(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x9A');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Temperature (°C):");
    shell_print((int)rcv_msg.data[1]);
    shell_println();
    shell_print("Brake release command:");
    shell_print((bool)rcv_msg.data[3]);
    shell_println();

    uint8_t voltage0 = rcv_msg.data[4];
    uint8_t voltage1 = rcv_msg.data[5];
    uint16_t voltage = (voltage1 << 8) | voltage0;

    shell_print("Voltage (V):");
    shell_print((float)voltage/10);
    shell_println();

    uint8_t error0 = rcv_msg.data[6];
    uint8_t error1 = rcv_msg.data[7];
    uint16_t error = (error1 << 8) | error0;

    switch(error)
    {
        case 2: // 0x0002
            shell_print("Error :Motor stall");
            shell_println();  
            break;
        case 4 : // 0x0004
            shell_print("Error :Undervoltage");
            shell_println();  
            break;
        case 8 : // 0x0008
            shell_print("Error :Overvoltage");
            shell_println();  
            break;
        case 16 : // 0x0010
            shell_print("Error :Power overrun");
            shell_println();  
            break;
        case 64 : // 0x0040
            shell_print("Error :Power overrun");
            shell_println();  
            break;
        case 256 : // 0x0100
            shell_print("Error :Overspeed");
            shell_println();  
            break;
        case 4096 : // 0x1000
            shell_print("Error :Overtemperature");
            shell_println();  
            break;
        case 8192 : // 0x2000
            shell_print("Error :Encoder Calibration error");
            shell_println();  
            break;
    }
}

/// @brief Read the motor temperature, speed and position of the output shaft of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_status2(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x9C');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Temperature (°C):");
    shell_print((int)rcv_msg.data[1]);
    shell_println();

    uint8_t current0 = rcv_msg.data[2];
    uint8_t current1 = rcv_msg.data[3];
    int16_t current = (current1 << 8) | current0;

    shell_print("Current (A) (need Driver V3):");
    shell_print((float)current/100);
    shell_println();

    uint8_t speed0 = rcv_msg.data[4];
    uint8_t speed1 = rcv_msg.data[5];
    int16_t speed = (speed1 << 8) | speed0;

    shell_print("Speed of motor output shaft (°/s) (need Driver V3):");
    shell_print(speed);
    shell_println();

    uint8_t pos0 = rcv_msg.data[6];
    uint8_t pos1 = rcv_msg.data[7];
    int16_t pos = (pos1 << 8) | pos0;

    shell_print("Position of motor output shaft relative to zero position (°):");
    shell_print(pos);
    shell_println();

}

/// @brief Read the motor temperature and current of the 3 Phases of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_status3(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x9D');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Temperature (°C):");
    shell_print((int)rcv_msg.data[1]);
    shell_println();

    uint8_t currentA0 = rcv_msg.data[2];
    uint8_t currentA1 = rcv_msg.data[3];
    int16_t currentA = (currentA1 << 8) | currentA0;

    shell_print("Phase A Current (A) :");
    shell_print((float)currentA/100);
    shell_println();

    uint8_t currentB0 = rcv_msg.data[4];
    uint8_t currentB1 = rcv_msg.data[5];
    int16_t currentB = (currentB1 << 8) | currentB0;


    shell_print("Phase B Current (A) :");
    shell_print((float)currentB/100);
    shell_println();

    uint8_t currentC0 = rcv_msg.data[6];
    uint8_t currentC1 = rcv_msg.data[7];
    int16_t currentC = (currentC1 << 8) | currentC0;

    shell_print("Phase C Current (A) :");
    shell_print((float)currentC/100);
    shell_println();

}

SHELL_COMMAND(read_pid, "Read PID parameters")
{
  if (argc > 0) {
    read_pid_params(atoi(argv[0]));
  } else {
    shell_print("Usage: read_pid [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_acc_cmd, "Read acceleration command of position loop")
{
  if (argc > 0) {
    read_acc_cmd(atoi(argv[0]));
  } else {
    shell_print("Usage: read_acc_cmd [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_pos_pulse, "Read encoder position command")
{
  if (argc > 0) {
    read_pos_pulse(atoi(argv[0]));
  } else {
    shell_print("Usage: read_pos_pulse [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_original_pos_pulse, "Read original encoder position command without zero offset (initial position)")
{
  if (argc > 0) {
    read_original_pos_pulse(atoi(argv[0]));
  } else {
    shell_print("Usage: read_original_pos_pulse [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_zero_offset, "Read Zero offset command")
{
  if (argc > 0) {
    read_zero_offset(atoi(argv[0]));
  } else {
    shell_print("Usage: read_zero_offset [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_angle, "Read the current absolute angle")
{
  if (argc > 0) {
    read_angle(atoi(argv[0]));
  } else {
    shell_print("Usage: read_angle [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_status1, "Read the motor temperature, voltage and error status flags")
{
  if (argc > 0) {
    read_status1(atoi(argv[0]));
  } else {
    shell_print("Usage: read_status1 [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_status2, "Read the motor temperature, speed and position of the motor output shaft")
{
  if (argc > 0) {
    read_status2(atoi(argv[0]));
  } else {
    shell_print("Usage: read_status2 [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_status3, "Read the motor temperature and current of the 3 Phases")
{
  if (argc > 0) {
    read_status3(atoi(argv[0]));
  } else {
    shell_print("Usage: read_status3 [Motor ID (1~32)]");
    shell_println();
  }
}


int main() {
    shell_init(&pc);
    while(1){
         ThisThread::sleep_for(200ms);
    }

}