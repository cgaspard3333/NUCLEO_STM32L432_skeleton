#include "mbed.h"
#include "shell.h"

#define ENCODER_MAX 65535

using namespace std::chrono;

CAN can(PA_11, PA_12,1000000); // RX, TX, baud rate
DigitalOut led3(LED3);

struct frame
{ 
  uint8_t head[2];
  uint16_t ts;
  int16_t current;
  int32_t pos;
}__attribute__((packed, aligned(1)));

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

/// @brief Convert 4bits unsigned data from CAN received message (4*uint8_t) to a unique uint32_t
/// @param rcv_msg CANMessage to convert
/// @return data from CANMessage under uint32_t format
uint32_t read_data_4byte(CANMessage rcv_msg){
    uint8_t data0 = rcv_msg.data[4];
    uint8_t data1 = rcv_msg.data[5];
    uint8_t data2 = rcv_msg.data[6];
    uint8_t data3 = rcv_msg.data[7];

    uint32_t data = (data3 << 24) | (data2 << 16) | (data1 << 8) | data0;

    return data;
}

/// @brief Convert 6bits signed data from CAN received message (6*uint8_t + 1uint8_t for the sign) to a unique int64_t
/// @param rcv_msg CANMessage to convert
/// @return data from CANMessage under uint32_t format
int64_t read_data_6byte_signed(CANMessage rcv_msg){
    uint8_t data0 = rcv_msg.data[1];
    uint8_t data1 = rcv_msg.data[2];
    uint8_t data2 = rcv_msg.data[3];
    uint8_t data3 = rcv_msg.data[4];
    uint8_t data4 = rcv_msg.data[5];
    uint8_t data5 = rcv_msg.data[6];
    uint8_t data6 = rcv_msg.data[7];

    uint64_t data = ((uint64_t)data5 << 40) | ((uint64_t)data4 << 32) | (data3 << 24) | (data2 << 16) | (data1 << 8) | data0;

    if(data6 == 1){
        data = (int64_t)data*(-1);
    }
    else{
        data = (int64_t)data;
    }

    return data;
}

/// @brief Takes two values (actual and previous one) of the one-turn  motor encoder to know if the motor output shaft (after reduction) passed to the next part
/// @param a actual encoder value
/// @param b precedent encoder value
/// @return 0 if in the same part of the motor output shaft | 1 if it has passed to the next one | -1 if it has passed on the previous one
int angle_rel2abs(int a, int b){
    int d=b-a;
    if (abs(d)<ENCODER_MAX/2)
    {
      return 0;
    }
    else if (b>a){
      return 1;
    }
    else 
      return -1;

}


/// @brief Read PID parameters of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_pid_params(int ID) {
    uint32_t pid_data;
    char can_code[6] = {'\x30','\x31','\x32','\x33','\x34','\x35'};
    char params[][19] = {"Position Loop KP :", "Position Loop KI :", "Speed Loop KP :", "Speed Loop KI :", "Speed Loop KP :", "Speed Loop KI :"};

    for (size_t i = 0; i < 6; i++)
    {
        CANMessage cmd = build_can_cmd(ID, can_code[i]);
        CANMessage rcv_msg = send_cmd(cmd);
        pid_data = read_data_4byte(rcv_msg);
        shell_print(params[i]);
        shell_print((float)pid_data/16777216,4);  //Q24 (Texas Instrument) format conversion
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
    shell_print((int)read_data_6byte_signed(rcv_msg));
    shell_println();    
}

/// @brief Read Position original command (pulses) without zero offset (initial position) of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_original_pos_pulse(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x61');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Position data without zero offset (pulses) :");
    shell_print((int)read_data_6byte_signed(rcv_msg));
    shell_println();    
}

/// @brief Read Zero offset command (pulses) of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_zero_offset(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x62');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Zero offset command (pulses) :");
    shell_print((int)read_data_6byte_signed(rcv_msg));
    shell_println();    
}

/// @brief Read the current absolute angle of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void read_abs_angle(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x92');
    CANMessage rcv_msg = send_cmd(cmd);

    shell_print("Absolute angle position (°) :");

    shell_print((float)read_data_6byte_signed(rcv_msg)/100/6,3);
    shell_println();
}

/// @brief Read the current angle for one round (0°~360°) BEFORE REDUCTION of the selected Motor
/// @param ID 
void read_angle(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x94');
    CANMessage rcv_msg = send_cmd(cmd);
    uint8_t angle0 = rcv_msg.data[6];
    uint8_t angle1 = rcv_msg.data[7];
    uint16_t angle = (angle1 << 8) | angle0;

    shell_print("Angle position (single round) (°):");
    shell_print((float)angle/100);
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
        // case 0: // 0x0000 No error is also 0x0000 ^^'
        //     shell_print("MotorError :Hardware over-current");
        //     shell_println();  
        //     break;
        case 2: // 0x0002
            shell_print("MotorError :Motor stalled");
            shell_println();  
            break;
        case 4 : // 0x0004
            shell_print("MotorError :Undervoltage");
            shell_println();  
            break;
        case 8 : // 0x0008
            shell_print("MotorError :Over-voltage");
            shell_println();  
            break;
        case 16 : // 0x0010
            shell_print("MotorError :Over-current");
            shell_println();  
            break;
        case 32 : // 0x0020
            shell_print("MotorError :Brake opening failed");
            shell_println();  
            break;
        case 64 : // 0x0040
            shell_print("MotorError :Bus current error");
            shell_println();  
            break;
        case 128 : // 0x0080
            shell_print("MotorError :Battery voltage error");
            shell_println();  
            break;
        case 256 : // 0x0100
            shell_print("MotorError :Overspeed");
            shell_println();  
            break;
        case 512 : // 0x0200
            shell_print("MotorError :Position loop exceeded error");
            shell_println();  
            break;
        case 1024 : // 0x0400
            shell_print("MotorError :VDD Error");
            shell_println();  
            break;
        case 2048 : // 0x0800
            shell_print("MotorError :DSP internal sensor temperature is overheated");
            shell_println();  
            break;
        case 4096 : // 0x1000
            shell_print("MotorError :Motor Over-temperature");
            shell_println();  
            break;
        case 8192 : // 0x2000
            shell_print("MotorError :Encoder Calibration error");
            shell_println();  
            break;
        case 240 : // 0x00F0
            shell_print("CANError :PID parameter write ROM protection, non-safe operation");
            shell_println();  
            break;
        case 241 : // 0x00F1
            shell_print("CANError :Encoder value is written into ROM protection, non-safe operation");
            shell_println();  
            break;
        case 242 : // 0x00F2
            shell_print("CANError :Three-loop switching operation error, non-safe operation");
            shell_println();  
            break;
        case 243 : // 0x00F3
            shell_print("CANError :Motor brake is not open");
            shell_println();  
            break;
        case 244 : // 0x00F4
            shell_print("CANError :Motor write ROM protection, non-safe operation");
            shell_println();  
            break;
    }
}

/// @brief Read the motor temperature, speed and encoder position value (0~65535) of the selected Motor
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

    shell_print("Current (A):");
    shell_print((float)current*33/2048);
    shell_println();

    uint8_t speed0 = rcv_msg.data[4];
    uint8_t speed1 = rcv_msg.data[5];
    int16_t speed = (speed1 << 8) | speed0;

    shell_print("Speed of motor output shaft (°/s):");
    shell_print(speed);
    shell_println();

    uint8_t pos0 = rcv_msg.data[6];
    uint8_t pos1 = rcv_msg.data[7];
    uint16_t pos = (pos1 << 8) | pos0;

    shell_print("Encoder position relative to zero position value (pulses):");
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

/// @brief Shutdown the selected Motor, clear the running state and and the previously received control commands
/// @param ID ID of the choosen Motor (1~32)
void shutdown_motor(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x80');
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\x80'){
        shell_print("Shutdown command sent");
        shell_println();
    }
}

/// @brief Stop the selected Motor (do not clear running state or previously received control commands)
/// @param ID ID of the choosen Motor (1~32)
void stop_motor(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x81');
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\x81'){
        shell_print("Stop command sent");
        shell_println();
    }
}

/// @brief Start/Resume the selected Motor - recover the control mode before the stop
/// @param ID ID of the choosen Motor (1~32)
void start_motor(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x88');
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\x88'){
        shell_print("Start/Resume command sent");
        shell_println();
    }
}

/// @brief Release the brake of the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void release_motor(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x77');
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\x77'){
        shell_print("Release command sent");
        shell_println();
    }
}

/// @brief Brake the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void brake_motor(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x78');
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\x78'){
        shell_print("Brake command sent");
        shell_println();
    }
}

/// @brief Reset the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void reset_motor(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x76');
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\x76'){
        shell_print("Reset command sent");
        shell_println();
    }
}

/// @brief Read the power consumption of the selected Motor (Strange value for now....)
/// @param ID ID of the choosen Motor (1~32)
void read_motor_power(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x71');
    CANMessage rcv_msg = send_cmd(cmd);

    uint8_t power0 = rcv_msg.data[6];
    uint8_t power1 = rcv_msg.data[7];
    uint16_t power = (power1 << 8) | power0;

    shell_print("Power (W):");
    shell_print(power/10);
    shell_println();
}

/// @brief Read the motor mode (Power-on initialization state or Current, Speed or Position Loop)
/// @param ID ID of the choosen Motor (1~32)
void read_motor_mode(int ID) {
    CANMessage cmd = build_can_cmd(ID,'\x70');
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\x70'){
        switch(rcv_msg.data[7])
            {
                case '\x00':
                    shell_print("Current loop mode");
                    shell_println();  
                    break;
                case '\x01':
                    shell_print("Speed loop mode");
                    shell_println();  
                    break;
                case '\x02' :
                    shell_print("Position loop mode");
                    shell_println();  
                    break;
                case '\xFF' :
                    shell_print("Power-on initialization state, not in three-ring mode");
                    shell_println();  
                    break;
            }
    }
}

/// @brief Send Torque closed-loop command (-32A~32A) to the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void torque_cmd(int ID, float torque_raw) {
    int torque = round((torque_raw*2000)/32);
    uint8_t byte0 = torque & 255;
    uint8_t byte1 = (torque >> 8);
    CANMessage cmd = build_can_cmd(ID,'\xA1','\x00','\x00','\x00', byte0, byte1);
    CANMessage rcv_msg = send_cmd(cmd);
    if (rcv_msg.data[0] == '\xA1'){
        shell_print("Torque command sent : ");
        shell_print(torque_raw);
        shell_print("A");
        shell_println();

        shell_print("Temperature (°C):");
        shell_print((int)rcv_msg.data[1]);
        shell_println();

        uint8_t current0 = rcv_msg.data[2];
        uint8_t current1 = rcv_msg.data[3];
        int16_t current = (current1 << 8) | current0;

        shell_print("Current (A):");
        shell_print((float)current*33/2048);
        shell_println();

        uint8_t speed0 = rcv_msg.data[4];
        uint8_t speed1 = rcv_msg.data[5];
        int16_t speed = (speed1 << 8) | speed0;

        shell_print("Speed of motor output shaft (°/s):");
        shell_print(speed);
        shell_println();

        uint8_t pos0 = rcv_msg.data[6];
        uint8_t pos1 = rcv_msg.data[7];
        uint16_t pos = (pos1 << 8) | pos0;

        shell_print("Encoder position relative to zero position value (pulses):");
        shell_print(pos);
        shell_println();
    }

    if (rcv_msg.data[0] == '\xA1' && rcv_msg.data[1] == '\x00' && rcv_msg.data[2] == '\x00' && rcv_msg.data[3] == '\x00' 
        && rcv_msg.data[4] == '\x00' && rcv_msg.data[5] == '\x00' && (rcv_msg.data[6] == '\xF6' || rcv_msg.data[6] == '\xF2')){
        shell_print("MotorError : three-loop operation error, switching to the current-loop safe state");
        shell_println();
    }
}

/// @brief Send Speed closed-loop command to the selected Motor
/// @param ID ID of the choosen Motor (1~32)
void speed_cmd(int ID, int32_t speed_raw) {
    int32_t speed = speed_raw*100*6; // 0.01°/s et ratio 1:6
    uint8_t byte0 = speed & 255;
    uint8_t byte1 = (speed >> 8);
    byte1 = byte1 & 255;
    uint8_t byte2 = (speed >> 16);
    byte2 = byte2 & 255;
    uint8_t byte3 = (speed >> 24);
    byte3 = byte3 & 255;
    CANMessage cmd = build_can_cmd(ID,'\xA2','\x00','\x00', '\x00', byte0, byte1, byte2, byte3);
    CANMessage rcv_msg = send_cmd(cmd);
    
    if (rcv_msg.data[0] == '\xA2'){
        shell_print("Speed command sent : ");
        shell_print((int)speed_raw);
        shell_print("°/s");
        shell_println();

        shell_print("Temperature (°C):");
        shell_print((int)rcv_msg.data[1]);
        shell_println();

        uint8_t current0 = rcv_msg.data[2];
        uint8_t current1 = rcv_msg.data[3];
        int16_t current = (current1 << 8) | current0;

        shell_print("Current (A):");
        shell_print((float)current*33/2048);
        shell_println();

        uint8_t speed0 = rcv_msg.data[4];
        uint8_t speed1 = rcv_msg.data[5];
        int16_t speed = (speed1 << 8) | speed0;

        shell_print("Speed of motor output shaft (°/s):");
        shell_print(speed/6);
        shell_println();

        uint8_t pos0 = rcv_msg.data[6];
        uint8_t pos1 = rcv_msg.data[7];
        uint16_t pos = (pos1 << 8) | pos0;

        shell_print("Encoder position relative to zero position value (pulses):");
        shell_print(pos);
        shell_println();
    }

    if (rcv_msg.data[0] == '\xA2' && rcv_msg.data[1] == '\x00' && rcv_msg.data[2] == '\x00' && rcv_msg.data[3] == '\x00' 
        && rcv_msg.data[4] == '\x00' && rcv_msg.data[5] == '\x00' && (rcv_msg.data[6] == '\xF6' || rcv_msg.data[6] == '\xF2')){
        shell_print("MotorError : three-loop operation error, switching to the current-loop safe state");
        shell_println();
    }
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

SHELL_COMMAND(read_abs_angle, "Read the current absolute angle")
{
  if (argc > 0) {
    read_abs_angle(atoi(argv[0]));
  } else {
    shell_print("Usage: read_abs_angle [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_angle, "Read the current angle for one round (0°~360°) BEFORE REDUCTION")
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

SHELL_COMMAND(shutdown, "Shutdown the selected Motor, clear the running state and and the previously received control commands")
{
  if (argc > 0) {
    shutdown_motor(atoi(argv[0]));
  } else {
    shell_print("Usage: shutdown [Motor ID (1~32)]");
    shell_println();
  }
}


SHELL_COMMAND(stop, "Stop the selected Motor (do not clear running state or previously received control commands)")
{
  if (argc > 0) {
    stop_motor(atoi(argv[0]));
  } else {
    shell_print("Usage: stop [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(release, "Release the brake of the selected Motor")
{
  if (argc > 0) {
    release_motor(atoi(argv[0]));
  } else {
    shell_print("Usage: release [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(brake, "Brake the selected Motor")
{
  if (argc > 0) {
    brake_motor(atoi(argv[0]));
  } else {
    shell_print("Usage: brake [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(start, "Start/Resume the selected Motor - recover the control mode before the stop")
{
  if (argc > 0) {
    start_motor(atoi(argv[0]));
  } else {
    shell_print("Usage: start [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(torque_cmd, "Send Torque closed-loop command (-32A~32A) ")
{
  if (argc > 1) {
    torque_cmd(atoi(argv[0]), atof(argv[1]));
  } else {
    shell_print("Usage: torque_cmd [Motor ID (1~32)] [Torque command (-32A~32A)]");
    shell_println();
  }
}

SHELL_COMMAND(speed_cmd, "Send Speed closed-loop command (°/s)")
{
  if (argc > 1) {
    speed_cmd(atoi(argv[0]), atof(argv[1]));
  } else {
    shell_print("Usage: speed_cmd [Motor ID (1~32)] [Speed command (°/s)]");
    shell_println();
  }
}

SHELL_COMMAND(read_mode, "Read the motor mode (Power-on initialization state or Current, Speed or Position Loop)")
{
  if (argc > 0) {
    read_motor_mode(atoi(argv[0]));
  } else {
    shell_print("Usage: read_mode [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(reset, "Reset motor")
{
  if (argc > 0) {
    reset_motor(atoi(argv[0]));
  } else {
    shell_print("Usage: reset [Motor ID (1~32)]");
    shell_println();
  }
}

SHELL_COMMAND(read_power, "Read the motor power consumption (Strange value for now....)")
{
  if (argc > 0) {
    read_motor_power(atoi(argv[0]));
  } else {
    shell_print("Usage: read_power [Motor ID (1~32)]");
    shell_println();
  }
}


// Commands for Aquisition program

/// @brief Acquisition program
void acquisition() {

    frame f;
    f.head[0]=0xff;
    f.head[1]=0xaa;

    Timer t;
    t.start();

    CANMessage cmd = build_can_cmd(1,'\x9C');
    CANMessage rcv_msg_init = send_cmd(cmd);
    uint8_t pos0 = rcv_msg_init.data[6];
    uint8_t pos1 = rcv_msg_init.data[7];
    uint16_t old_pos = (pos1 << 8) | pos0;

    int tour = 0;
    int t_old = duration_cast<microseconds>(t.elapsed_time()).count();

    for (size_t i = 0; i < 4000; i++)
    {
        
        CANMessage rcv_msg = send_cmd(cmd);

        uint8_t current0 = rcv_msg.data[2];
        uint8_t current1 = rcv_msg.data[3];
        int16_t current = (current1 << 8) | current0;

        uint8_t pos0 = rcv_msg.data[6];
        uint8_t pos1 = rcv_msg.data[7];
        uint16_t pos = (pos1 << 8) | pos0;

        tour += angle_rel2abs(pos, old_pos);
        old_pos = pos;
        int32_t pos_multitour = ((float)pos+tour*65535); // *360/65535/6

        f.current = current;
        f.pos = pos_multitour;
        f.ts = duration_cast<microseconds>(t.elapsed_time()).count()-t_old;

        t_old = duration_cast<microseconds>(t.elapsed_time()).count();
        pc.write(&f,10);
    }
    t.stop();
}


SHELL_COMMAND(exp, "For aquisition program")
{
    acquisition();
}




int main() {
    shell_init(&pc);
    while(1){
         ThisThread::sleep_for(200ms);
    }

}