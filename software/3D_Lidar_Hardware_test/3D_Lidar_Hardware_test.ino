#include <SoftwareSerial.h>
#include "TFMini.h"
#include <Servo.h>

// LiDar
#define RX_PIN 10
#define TX_PIN 11
#define BAUD_RATE 115200
SoftwareSerial lidar_serial(TX_PIN, RX_PIN);
TFMini tfmini;

// Stepper
#define L_CLK_PIN 6
#define L_CW_PIN 7

// Servo
#define SERVO_PIN 5
Servo pitchServo;


// GLOBALS
#define DEFAULT_STEP_DELAY 5000 // In microseconds
#define LIDAR_SCAN_DELAY 25
#define PITCH_CHANGE_DELAY 200
#define DEFAULT_SIGNAL_STRENGTH 0 // TODO: Currently accepts all signals. Requires testing to find a good number.
#define DEFAULT_YAW_RESOLUTION 50
#define DEFAULT_PITCH_RESOLUTION 3
#define FULL_YAW_REVOLUTION_STEPS 215 // TODO: Needs verification. How many stepper steps in a full rotation?
#define MAX_YAW_RESOLUTION 360
#define MAX_PITCH_RESOLUTION 180 // TODO: Needs verification. Might need a min and max angle because of roof.


// HIGH LEVEL FUNCTIONS
void testHardWare(uint16_t yaw_resolution=DEFAULT_YAW_RESOLUTION, uint16_t pitch_resolution=DEFAULT_PITCH_RESOLUTION);


// HARDWARE DRIVER FUNCTIONS
// Stepper
void rotateYawClockwise(unsigned int step_delay=DEFAULT_STEP_DELAY, uint16_t steps=1);
void rotateYawCounterClockwise(unsigned int step_delay=DEFAULT_STEP_DELAY, uint16_t steps=1);

// Servo
void setPitch(uint16_t pos);

// LiDar
uint16_t getLidarDistance(uint16_t minSignalStrength=DEFAULT_SIGNAL_STRENGTH, unsigned int scanDelay=LIDAR_SCAN_DELAY);


void setup() {
  Serial.begin(BAUD_RATE);
  
  // LiDar setup
  while (!Serial);// wait for serial port to connect. Needed for native USB port only
  Serial.println ("Initializing...");
  lidar_serial.begin(TFMINI_BAUDRATE);
  tfmini.begin(&lidar_serial);

  // Stepper setup
  pinMode(L_CLK_PIN, OUTPUT);
  pinMode(L_CW_PIN, OUTPUT);
  
  digitalWrite(L_CW_PIN, HIGH); // Sets left stepper to spin correctly. High is clockwise.

  // Servo setup
  pitchServo.attach(SERVO_PIN);
}


void loop() {
  testHardWare();
}


// HIGH LEVEL FUNCTIONS
/* @Description
 *  High level function for testing the hardware. Data is displayed, but not saved.
 *  
 * @Params 
 *  yaw_resolution: Amount of points per point ring. Max is MAX_YAW_RESOLUTION in degrees.
 *  pitch_resolution: Amount of point rings in the data cloud. Max is MAX_PITCH_RESOLUTION in degrees.
 *  
 * @Notes
 *  - There is some data loss because of flooring. Amount of steps the physical stepper motor can do is too low.
 *  - Data loss might offset a rotation by a few steps each time. Might need to add a recalibration function call.
 */
void testHardWare(uint16_t yaw_resolution, uint16_t pitch_resolution)
{

  //TODO: Check edge cases. What if user enters invalid angles?
  
  uint16_t yaw_steps = FULL_YAW_REVOLUTION_STEPS/yaw_resolution;
  uint16_t pitch_steps = MAX_PITCH_RESOLUTION/pitch_resolution;

  for(int i=0; i<MAX_PITCH_RESOLUTION; i+=pitch_steps) // TODO: Pitch might need a min and max. Needs physical testing.
  {
    setPitch(i);
    delay(PITCH_CHANGE_DELAY);
    
    for(int j=0; j<FULL_YAW_REVOLUTION_STEPS; j+=yaw_steps)
    {
      rotateYawClockwise(DEFAULT_STEP_DELAY, yaw_steps);
      Serial.print("Yaw: ");
      Serial.print(j);

      Serial.print(" | Pitch: ");
      Serial.print(i);

      Serial.print(" | Distance: ");
      Serial.println(getLidarDistance());      
    }
  }
  
  return;
}


// HARDWARE DRIVER FUNCTIONS
/* @Description
 *  Low level function for getting distance from LiDar.
 *  
 * @Params 
 *  signalStrength: Minimum signal strength before the distance data is considered invalid.
 *  
 * @Return
 *  Returns a distance in centimeters. Minimum distance is 30cm. Returns 0 if signal strength is too low.
 *  
 * @Notes
 * - https://cdn.sparkfun.com/assets/5/e/4/7/b/benewake-tfmini-datasheet.pdf
 * - Currently all signals are being accepted. Lidar needs more testing to determine a good minimum signal strength number.
 */
uint16_t getLidarDistance(uint16_t minSignalStrength, unsigned int scanDelay)
{
  //return 0; //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
  
  uint16_t dist = tfmini.getDistance(); // Lidar isn't getting enough power and breaking here.
  uint16_t strength = tfmini.getRecentSignalStrength();

  delay(scanDelay);

  if (strength >= minSignalStrength)
    return dist;
  else
    return 0;
}


/* @Description
 *  Low level helper function for rotating the servo.
 *  
 * @Params 
 *  pos: The angle of the servo.
 */
void setPitch(uint16_t pos)
{
  pitchServo.write(pos);
  
  return;
}


/* @Description
 *  Low level helper function for rotating clockwise.
 *  
 * @Params 
 *  step_delay: The delay between each stepper step in microseconds.
 *  steps: The amount of times to step a step.
 */
void rotateYawClockwise(unsigned int step_delay, uint16_t steps)
{
  digitalWrite(L_CW_PIN, HIGH);

  for(unsigned int i = 0; i<steps; ++i){
    digitalWrite(L_CLK_PIN, HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(L_CLK_PIN, LOW);
  }

  return;
}


/* @Description
 *  Low level helper function for rotating counterclockwise.
 *  
 * @Params 
 *  step_delay: The delay between each stepper step in microseconds.
 *  steps: The amount of times to step a step.
 */
void rotateYawCounterClockwise(unsigned int step_delay, uint16_t steps)
{
  digitalWrite(L_CW_PIN, LOW);

  for(unsigned int i = 0; i<steps; ++i){
    digitalWrite(L_CLK_PIN, HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(L_CLK_PIN, LOW);
  }
  
  return;
}
