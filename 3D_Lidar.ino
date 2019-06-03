#include <ArduinoSTL.h>

#include <SoftwareSerial.h>
#include "TFMini.h"

// https://github.com/maniacbug/StandardCplusplus
#include <StandardCplusplus.h> 
#include <vector>

#include <Servo.h>

// LiDAR Pins
#define RX_PIN 10
#define TX_PIN 11
#define BAUD_RATE 115200

// Stepper Pins
#define L_CLK_PIN 6
#define L_CW_PIN 7
#define R_CLK_PIN 8
#define R_CW_PIN 9

// Servo Pins
#define SERVO_PIN 5
Servo pitchServo;

// Setup software serial port 
SoftwareSerial lidar_serial(RX_PIN, TX_PIN);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

// Data
vector<*vector<uint16_t>> get3DCoords(vector<float> pitch_degrees, float yaw_degree_increments=2.3);
vector<uint16_t> get2DCoords(float yaw_degree_increments=2.3);

// Stepper
void rotateClockwise(float step_delay, unsigned int steps);
void rotateCounterClockwise(float step_delay, unsigned int steps);

// Servo
void setPitch(int pos);

void setup() {
  Serial.begin(BAUD_RATE);
  
  // LiDAR Setup
  while (!Serial);// wait for serial port to connect. Needed for native USB port only
  mySerial.begin(TFMINI_BAUDRATE);
  tfmini.begin(&lidar_serial);    

  // Stepper Setup
  pinMode(R_CLK_PIN, OUTPUT);
  pinMode(R_CW_PIN, OUTPUT);
  pinMode(L_CLK_PIN, OUTPUT);
  pinMode(L_CW_PIN, OUTPUT);
  
  digitalWrite(L_CW_PIN, HIGH); // Sets left stepper to spin correctly. High is clockwise.
  digitalWrite(R_CW_PIN, LOW);

  pitchServo.attach(SERVO_PIN);
}


void loop() {

  setPitch(50);
  
//  // Take one TF Mini distance measurement
//  uint16_t dist = tfmini.getDistance();
//  uint16_t strength = tfmini.getRecentSignalStrength();
//
//  // Display the measurement
//  Serial.print(dist);
//  Serial.print(" cm      sigstr: ");
//  Serial.println(strength);
//
//  // Wait some short time before taking the next measurement
//  delay(25);  
}


/* @Description
 *  High level function for getting point cloud coordinates.
 *  
 * @Params 
 * pitch_degrees: The amount of degrees to increment in the pitch direction from 0 to 180 degrees. Default is (FILL ME IN!!) degrees.
 * yaw_degree_increments: The amount of degrees to increment in the yaw direction from 0 to 360 degrees. Default is 2.3 degrees.
 */
vector<*vector<uint16_t>> get3DCoords(vector<float> pitch_degrees, float yaw_degree_increments){

  return;
}


 /* @Description
 *  Lower level function for getting 2D coordinates. Used by get3DCoords().
 *  
 * @Params 
 *  yaw_degree_increments: The amount of degrees to increment in the yaw direction from 0 to 360 degrees. Default is 2.3 degrees.
 *  
 * Notes:
 * https://www.symphotony.com/wp-content/uploads/DE-LiDAR-TFmini-Datasheet-V1.5-EN-17.pdf
 * The FOV of the TFmini is 2.3 degrees which determine the side lengths of different detection ranges of LiDAR. (Page 6)
 */
vector<uint16_t> get2DCoords(float yaw_degree_increments){
  
  return;
}


/* @Description
 *  Low level helper function for rotating clockwise. Used by get2DCoords().
 *  
 * @Params 
 *  step_delay: The delay between each stepper step.
 *  steps: The amount of times to step a step.
 */
void rotateClockwise(float step_delay, unsigned int steps)
{
  digitalWrite(L_CW_PIN, HIGH);
  digitalWrite(R_CW_PIN, HIGH);

  for(unsigned int i = 0; i<steps; ++i){
    digitalWrite(R_CLK_PIN, HIGH);
    digitalWrite(L_CLK_PIN, HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(R_CLK_PIN, LOW);
    digitalWrite(L_CLK_PIN, LOW);
  }

  return;
}


/* @Description
 *  Low level helper function for rotating counterclockwise. Used by get2DCoords().
 *  
 * @Params 
 *  step_delay: The delay between each stepper step.
 *  steps: The amount of times to step a step.
 */
void rotateCounterClockwise(float step_delay, unsigned int steps)
{
  digitalWrite(L_CW_PIN, LOW);
  digitalWrite(R_CW_PIN, LOW);

  for(unsigned int i = 0; i<steps; ++i){
    digitalWrite(R_CLK_PIN, HIGH);
    digitalWrite(L_CLK_PIN, HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(R_CLK_PIN, LOW);
    digitalWrite(L_CLK_PIN, LOW);
  }
  
  return;
}

void setPitch(int pos)
{
  pitchServo.write(pos);
  
  return;
}
