/* TODO:
 - Reference a static location with a known distance to define the "Front". The servo will use a steep unused angle to scan the top of the rover for such a location.
 - If low on memory , code is upgradeable by using vectors of pointers instead.
*/

#include <StandardCplusplus.h>
#include <vector>
#include <SoftwareSerial.h>
#include "TFMini.h"

// https://github.com/maniacbug/StandardCplusplus
#include <StandardCplusplus.h> 
#include <vector>
using namespace std;

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
#define DEFAULT_SIGNAL_STRENGTH 0 // TODO: Currently accepts all signals. Requires testing to find a good number.
#define DEFAULT_YAW_RESOLUTION 50
#define DEFAULT_PITCH_RESOLUTION 3
#define FULL_YAW_REVOLUTION_STEPS 215 // TODO: Needs verification. How many stepper steps in a full rotation?
#define MAX_YAW_RESOLUTION 360
#define MAX_PITCH_RESOLUTION 180 // TODO: Needs verification. Might need a min and max angle because of roof.

// CLASSES
class Point
{
    public:
        Point(uint16_t new_yaw=0, uint16_t new_pitch=0, uint16_t new_distance=0)
        {
            yaw = new_yaw;
            pitch = new_pitch;
            distance = new_distance;
        }
        
        ~Point()
        {
          yaw = pitch = distance = 0;
        }
        
        void print()
        {
            Serial.print("Yaw: ");
            Serial.print(yaw);

            Serial.print(" | Pitch: ");
            Serial.print(pitch);

            Serial.print(" | Distance: ");
            Serial.print(distance);
            
            return;
        }
        
        uint16_t yaw; // clockwise or counterclockwise
        uint16_t pitch; // up or down
        uint16_t distance ; // Lidar's reading
};


class PointRing
{
    public:
        PointRing(const PointRing &new_point_ring)
        {
            setPointRing(new_point_ring);
        }
        
        PointRing(vector<Point> new_point_ring)
        {
           setPointRing(new_point_ring);
        }
        
         PointRing()
        {
            // Intentionally left blank.
        }
        
        ~PointRing()
        {
            clearPointRing();
        }
        
        void addPoint(Point newPoint)
        {
            data.push_back(newPoint);
            return;
        }
        
        void addPoint(uint16_t yaw, uint16_t pitch, uint16_t distance)
        {
            addPoint(Point(yaw, pitch, distance));
            return;
        }
        
        void clearPointRing()
        {
           data.clear();
           return;   
        }
        
        void setPointRing(PointRing new_point_ring)
        {
            data = new_point_ring.data;
            return;
        }
    
         void setPointRing(vector<Point> new_point_ring)
        {
            data = new_point_ring;
            return;
        }
        
        void print()
        {
            for (uint16_t i=0; i<data.size(); ++i)
                data[i].print();
            return;
        }
        
        vector<Point> data;
};


class PointCloud
{
    public:
        PointCloud(const PointCloud &new_point_cloud)
        {
            setPointCloud(new_point_cloud);
        }
        
        PointCloud(vector<PointRing> new_point_cloud)
        {
           setPointCloud(new_point_cloud);
        }
        
         PointCloud()
        {
            // Intentionally left blank.
        }
        
        ~PointCloud()
        {
            clearPointCloud();
        }
        
        void addPoint(Point newPoint, uint16_t pointRingPos) //TODO: should already know where because of yaw.
        {
            data[pointRingPos].addPoint(newPoint);
            return;
        }
        
        void addPoint(uint16_t yaw, uint16_t pitch, uint16_t distance, uint16_t pointRingPos)
        {
            addPoint(Point(yaw, pitch, distance), pointRingPos);
            return;
        }

        void addPointRing(PointRing newPointRing)
        {
            data.push_back(newPointRing);
            return;
        }

        void addPointRing(vector<Point> newPointRing)
        {
            data.push_back(PointRing(newPointRing));
            return;
        }
        
        void clearPointCloud()
        {
           data.clear();
           return;   
        }
        
        void setPointCloud(PointCloud new_point_cloud)
        {
            data = new_point_cloud.data;
            return;
        }
    
         void setPointCloud(vector<PointRing> new_point_cloud)
        {
            data = new_point_cloud;
            return;
        }
        
        void print()
        {
            for (uint16_t i=0; i<data.size(); ++i)
                data[i].print();
            return;
        }
        
        vector<PointRing> data;
};


// DATA
PointCloud point_cloud;


// HIGH LEVEL FUNCTIONS
void generatePointCloud(PointCloud &point_cloud, uint16_t yaw_resolution=DEFAULT_YAW_RESOLUTION, uint16_t pitch_resolution=DEFAULT_PITCH_RESOLUTION);


// HARDWARE DRIVER FUNCTIONS
// Stepper
void rotateYawClockwise(unsigned int step_delay=DEFAULT_STEP_DELAY, uint16_t steps=1);
void rotateYawCounterClockwise(unsigned int step_delay=DEFAULT_STEP_DELAY, uint16_t steps=1);

// Servo
void setPitch(uint16_t pos);

// LiDar
uint16_t getLidarDistance(uint16_t minSignalStrength=DEFAULT_SIGNAL_STRENGTH);


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
  
//// Take one TF Mini distance measurement
//  uint16_t dist = tfmini.getDistance();
//  uint16_t strength = tfmini.getRecentSignalStrength();
//
//  // Display the measurement
//  Serial.print(dist);
//  Serial.print(" cm      sigstr: ");
//  Serial.println(strength);
//
//  if (dist < 100)
//  {
//    setPitch(100);
//  }
//  else
//  {
//    setPitch(60);
//  }

  generatePointCloud(point_cloud);
  point_cloud.print();

}


// HIGH LEVEL FUNCTIONS
/* @Description
 *  High level function for populating the point cloud.
 *  
 * @Params 
 *  point_cloud: The point cloud to save data into.
 *  yaw_resolution: Amount of points per point ring. Max is MAX_YAW_RESOLUTION in degrees.
 *  pitch_resolution: Amount of point rings in the data cloud. Max is MAX_PITCH_RESOLUTION in degrees.
 *  
 * @Notes
 *  - There is some data loss because of flooring. Amount of steps the physical stepper motor can do is too low.
 *  - Data loss might offset a rotation by a few steps each time. Might need to add a recalibration function call.
 */
void generatePointCloud(PointCloud &point_cloud, uint16_t yaw_resolution, uint16_t pitch_resolution)
{

  //TODO: Check edge cases. What if user enters invalid angles?
  
  uint16_t yaw_steps = FULL_YAW_REVOLUTION_STEPS/yaw_resolution;
  uint16_t pitch_steps = MAX_PITCH_RESOLUTION/pitch_resolution;

  point_cloud.clearPointCloud();

  // This for loop below creates and add point rings into the point cloud.
  for(int i=0; i<MAX_PITCH_RESOLUTION; i+=pitch_steps) // TODO: Pitch might need a min and max. Needs physical testing.
  {
    setPitch(i);
    PointRing new_point_ring;

    // This for loop below creates points and adds them to point rings.
    for(int j=0; j<FULL_YAW_REVOLUTION_STEPS; j+=yaw_steps) // Creates points for point rings
    {
      rotateYawClockwise(DEFAULT_STEP_DELAY, yaw_steps);
      Point new_point(j, i, getLidarDistance());
      new_point_ring.addPoint(new_point);
    }

    point_cloud.addPointRing(new_point_ring);
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
uint16_t getLidarDistance(uint16_t minSignalStrength)
{
  uint16_t dist = tfmini.getDistance();
  uint16_t strength = tfmini.getRecentSignalStrength();

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
