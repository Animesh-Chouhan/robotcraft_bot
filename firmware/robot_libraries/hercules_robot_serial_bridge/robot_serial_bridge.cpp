/******************************************************************

    This library create a serial bridge interface between
    the Hercules Dual Motor Controller and the Arduino Mega.

    Created by Ingeniarius, Ltd

    RobotCraft 2017 edition

 ******************************************************************/


#include "robot_serial_bridge.h"


/************************************************************************

   Function: robot_serial_bridge()
   Objective: Class constructor
   Inputs parameters: HardwareSerial class object to init the desired serial port
   Outputs parameters:
   Return:
   Notes:

 *************************************************************************/
robot_serial_bridge::robot_serial_bridge(HardwareSerial *ser) {
  common_init();  // Set everything to common state, then...
  hwSerial = ser; // ...override hwSerial with value passed.
}


/************************************************************************

   Function:  common_init()
   Objective: Init comm variables
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes: class private func

 *************************************************************************/
void robot_serial_bridge::common_init(void) {
  hwSerial  = NULL;
}

/************************************************************************

   Function:  begin()
   Objective: Init serial configurations
   Inputs parameters:
   Outputs parameters:
   Return: Check sent command, True - successful; False - error
   Notes: The default serial baudrate is defined to 115200

 *************************************************************************/
boolean robot_serial_bridge::begin() {

  hwSerial->begin(DEFAULT_SERIAL_BAUDRATE);

  if (DEBUG_MODE)
    Serial.begin(DEFAULT_SERIAL_BAUDRATE);

  return reset();
}

/************************************************************************

   Function:  reset()
   Objective: Reset routine when class is init
   Inputs parameters:
   Outputs parameters:
   Return:  Check sent command, True - successful; False - error
   Notes:

 *************************************************************************/
boolean robot_serial_bridge::reset() {

  // Flush all serial data before start
  while (hwSerial->available() > 0)
    hwSerial->read();

  return reset_encoders();
}

/************************************************************************

   Function:  bridge_heartbeat()
   Objective: Check serial connection connectivity by sending a ping-pong msg
   Inputs parameters:
   Outputs parameters:
   Return:  Check sent command, True - successful; False - error
   Notes:

 *************************************************************************/
boolean robot_serial_bridge::bridge_heartbeat() {

  sendCommand(COMMAND_HEARTBEAT, 0, 0);
  return readResponse(TIMEOUT_INCOMING_RESPDATA);
}


/************************************************************************

   Function:  stop_motors()
   Objective: Stop motors
   Inputs parameters:
   Outputs parameters:
   Return:  Check sent command, True - successful; False - error
   Notes: "@,195,0,0;"

 *************************************************************************/
boolean robot_serial_bridge::stop_motors() {

  sendCommand(COMMAND_STOP_MOTORS, 0, 0);
  return readResponse(TIMEOUT_INCOMING_RESPDATA);

}

/************************************************************************

   Function:  reset_encoders()
   Objective: Reset encoders counter to zero.
   Inputs parameters:
   Outputs parameters:
   Return:  Check sent command, True - successful; False - error
   Notes: "@,196,0,0;"

 *************************************************************************/
boolean robot_serial_bridge::reset_encoders() {

  sendCommand(COMMAND_RESET_ENCODERS, 0, 0);
  return readResponse(TIMEOUT_INCOMING_RESPDATA);

}

/************************************************************************

   Function: get_encoders()
   Objective: Resquest encoders values
   Inputs parameters:
   Outputs parameters:  encoder1 and encoder2 (absolute counter)
   Return:  Check sent command, True - successful; False - error
   Notes: "@,197,0,0;"

 *************************************************************************/
boolean robot_serial_bridge::get_encoders(long int &enc1, long int &enc2) {

  sendCommand(COMMAND_REQ_ENCODERS, 0, 0);
  return readResponse(enc1, enc2, TIMEOUT_INCOMING_RESPDATA);
}


/************************************************************************

   Function:  move_motors_pwm()
   Objective: Send PWM values to each motor by sending -100 (backwards) to 100 (frontwards)
   Inputs parameters:   pwm1 and pwm2
   Outputs parameters:
   Return:  Check sent command, True - successful; False - error
   Notes: "@,198,0,0;"

 *************************************************************************/
boolean robot_serial_bridge::move_motors_pwm(int pwm1, int pwm2) {

  sendCommand(COMMAND_SET_SPEED_PWM, pwm1, pwm2);
  return readResponse(TIMEOUT_INCOMING_RESPDATA);

}


/************************************************************************

   Function: sendCommand ()
   Objective:
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes: private func

 *************************************************************************/
void robot_serial_bridge::sendCommand(unsigned char cmd, long int param1, long int param2) {

  String stringFinal = String("@," + String(cmd) + "," + String(param1) + "," + String(param2) + ";"); // concatenating strings
  hwSerial->print(stringFinal);
}



/************************************************************************

   Function:  readResponse()
   Objective:
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes: private func

 *************************************************************************/
boolean robot_serial_bridge::readResponse(uint16_t timeout_incomingData) {

  long int null_p1, null_p2;
  return readResponse(null_p1, null_p2, timeout_incomingData);
}

/************************************************************************

   Function:  readResponse();
   Objective:
   Inputs parameters:
   Outputs parameters:
   Return:
   Notes: private func

 *************************************************************************/
boolean robot_serial_bridge::readResponse(long int &rply1, long int &rply2, uint16_t timeout_incomingData) {

  unsigned long timeout_counter = millis();
  byte incomingByte = 0;
  char incomingData[MAX_PROTOCOL_LENGHT] = {'\0'};
  bool healthy_msg = false;

  while (millis() - timeout_counter <= timeout_incomingData) {

    if (hwSerial->available() > 0) {

      while (millis() - timeout_counter <= timeout_incomingData) {   // 1st verification stage

        incomingByte = hwSerial->read();

        if (DEBUG_MODE)
          Serial.println(incomingByte, HEX);

        if (incomingByte == PROTOCOL_START ) { // @ = 0x40
          healthy_msg = true;
          break;
        }

        if (!hwSerial->available() || incomingByte == PROTOCOL_CR) {
          healthy_msg = false;
          break;
        }
      }

      if (incomingByte == PROTOCOL_START && healthy_msg) {  // 2st verification stage

        byte total_received_data = 0;

        while (millis() - timeout_counter <= timeout_incomingData) {

          if (hwSerial->available() > 0 ) {

            incomingByte = hwSerial->read();
            incomingData[total_received_data++] = incomingByte;

            if (incomingByte == PROTOCOL_END) {
              healthy_msg = true;
              break;
            }
          }

        }

        unsigned int commas_counter = 0;
        for (int i = 0; i < total_received_data; i++) {
          if (incomingData[i] == PROTOCOL_START) {        // Verify if there are drop messages and there was started a new one
            healthy_msg = false;
            break;
          }

          if (incomingData[i] == PROTOCOL_COMMAS) {       // Count number of commas in the protocol msg
            commas_counter++;
          }
        }

        if (commas_counter != PROTOCOL_TOTAL_COMMAS) {    // Verify if there are commas (separator) in the protocol msg
          healthy_msg = false;
        }
      }

      if (healthy_msg) {

        if (DEBUG_MODE)
          Serial.println(incomingData);

        char *reply_cmd;
        char *parm1;
        char *parm2;

        reply_cmd = strtok(incomingData, ",");          //let's pars the string at each comma.
        parm1 = strtok(NULL, ",");
        parm2 = strtok(NULL, ",");


        int reply_command = atoi(reply_cmd);
        long int paramter1 = atol(parm1);
        long int paramter2 = atol(parm2);

        switch (reply_command) {

          case COMMAND_SUCCESSFUL:       // move motors with encoder diff velocities

            if (paramter1 == 1 && paramter2 == 1) {
              healthy_msg = true;
            }
            break;

          case COMMAND_ERROR:       // move motors with pwm valueas

            healthy_msg = false;
            break;

          case COMMAND_GET_ENCODERS:       // request encoder values

            rply1 = paramter1;
            rply2 = paramter2;

            break;

          default:

            healthy_msg = false;

            break;
        }

      }
      return healthy_msg;
    }
  }

  return healthy_msg;
}

/************************************************************************

   Function: get_encoders_diff()
   Objective: Resquest encoders difference
   Inputs parameters:
   Outputs parameters:  encoder1 and encoder2 (relative counter)
   Return:  Check sent command, True - successful; False - error
   Notes: "@,197,0,0;"

 *************************************************************************/
boolean robot_serial_bridge::get_encoders_diff(long int &enc1_diff, long int &enc2_diff) {

  long int ec1;
  long int ec2;
  boolean rply_status;

  sendCommand(COMMAND_REQ_ENCODERS, 0, 0);
  rply_status = readResponse(ec1, ec2, TIMEOUT_INCOMING_RESPDATA);

  enc1_diff = ec1 - encoder1_last;
  enc2_diff = ec2 - encoder2_last;


  encoder1_last = ec1;
  encoder2_last = ec2;


  return rply_status;
}