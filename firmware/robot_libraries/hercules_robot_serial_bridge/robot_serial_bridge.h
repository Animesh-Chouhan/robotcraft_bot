/******************************************************************
 * 
 *  This library create a serial bridge interface between  
 *  the Hercules Dual Motor Controller and the Arduino Mega.
 *  
 *  Created by Ingeniarius, Ltd
 *  
 *  RobotCraft 2017 edition
 *  
 ******************************************************************/
#include "Arduino.h"

// Prototype message protocol
// @,<CMD>,<Param1>,<Param1>;

// Protocol delimiters 
#define PROTOCOL_START    0x40       // @
#define PROTOCOL_END      0x3B       // ; 
#define PROTOCOL_CR       0x0D       // <CR>  Carriage return
#define PROTOCOL_COMMAS   0x2C       // ,
#define PROTOCOL_TOTAL_COMMAS   3    // Number of commas in the message
#define MAX_PROTOCOL_LENGHT 30       // Max protocol message lenght in bytes
#define MIN_PROTOCOL_LENGHT 8        // Min protocol message lenght in bytes

// Protocol delimiters
#define COMMAND_SET_SPEED_PWM 198
#define COMMAND_REQ_ENCODERS 197
#define COMMAND_RESET_ENCODERS 196
#define COMMAND_STOP_MOTORS 195
#define COMMAND_GET_ENCODERS 200
#define COMMAND_SUCCESSFUL 100
#define COMMAND_ERROR 112
#define COMMAND_HEARTBEAT 99

// Debug mode flag
#define DEBUG_MODE 0

// Serial timeout to incoming data
#define TIMEOUT_INCOMING_RESPDATA 50  // in ms

// Default Serial baudrate
#define DEFAULT_SERIAL_BAUDRATE 115200

// Auxiliary function to normalize angle to the -pi, pi domain 
#define NORMALIZE(z) atan2(sin(z), cos(z))


class robot_serial_bridge {
  public:

    /* Init Func */
    robot_serial_bridge(HardwareSerial *ser);                       // Class constructor
    boolean begin(void);                                            // Init class configurations

    /* Serial connectivity verification */
    boolean bridge_heartbeat(void);                                 // Heart Beat function to test serial connectivity

    /* Control Func */
    boolean move_motors_pwm(int pwm1, int pwm2);                    // Move motors function by sending PWM values between -100 and 100
    boolean stop_motors(void);                                      // Stop motors
    boolean get_encoders(long int &enc1, long int &enc2);           // Get absolute encoders values 
    boolean reset_encoders(void);                                   // Reset encoders

    boolean get_encoders_diff(long int &enc1_diff, long int &enc2_diff);
    
  private:

    HardwareSerial *hwSerial;

    long int encoder1_last;
    long int encoder2_last;

    void common_init(void);
    boolean reset(void);

    boolean readResponse(uint16_t timeout_incomingData = 100);
    boolean readResponse(long int &rply1, long int &rply2, uint16_t timeout_incomingData = 100);
    void sendCommand(byte cmd, long int param1, long int param2);

};
