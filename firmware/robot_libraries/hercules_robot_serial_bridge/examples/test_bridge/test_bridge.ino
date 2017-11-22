#include <robot_serial_bridge.h>

// Class Init
robot_serial_bridge robot = robot_serial_bridge(&Serial1); // TX1 e RX1 (Arduino Mega)


struct {
 long int encoder1 = 0;
 long int encoder2 = 0;
 long int diff_encoder1 = 0;
 long int diff_encoder2 = 0;
} robot_data;


void setup() {

  Serial.begin(115200);

  // Init Serial connection with robot
  robot.begin();

  // Check Serial Connectivity
  if (robot.bridge_heartbeat()) {
    Serial.println("Serial bridge ready to rumble!!");
  } else {
    Serial.println("Check your connections");
    delay(1000);
    exit(0);
  }
  delay(1000);

  // Stop motors
  bool command_status = robot.stop_motors();
}


unsigned long lp_time = 0;

void loop() {

  if (millis() - lp_time > 200 ) {                                // Just run the routine every 200 milliseconds  -> 5Hz

    lp_time = millis();                                           // update time

    robot.get_encoders(robot_data.encoder1, robot_data.encoder2); // update encoders values
    robot.get_encoders_diff(robot_data.diff_encoder1, robot_data.diff_encoder2); // update encoders diff values
    robot.move_motors_pwm(5,5);                                   // move motors frontwards
    
    Serial.println("Encoders:");                                  // Print encoders values
    Serial.println(robot_data.encoder1);
    Serial.println(robot_data.encoder2);
    Serial.println("-----------------");

    Serial.println("Encoders (Diff):");
    Serial.println(robot_data.diff_encoder1);
    Serial.println(robot_data.diff_encoder2);
    Serial.println("-----------------");
  } 
}

