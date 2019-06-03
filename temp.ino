#include <StandardCplusplus.h>
#include <vector>

#define NUMBER_OF_SERVO_ANGLES 3
#define NUMBER_OF_ANGLES 10

#define int_2b unsigned short int

using namespace std;

class Point
{
    public:
        Point(int_2b new_yaw=0, int_2b new_pitch=0, int_2b new_distance=0)
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
        
        int_2b yaw; // clockwise or counterclockwise
        int_2b pitch; // up or down
        int_2b distance ; // Lidar's reading
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
        
        void addPoint(int_2b yaw, int_2b pitch, int_2b distance)
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
            for (int_2b i=0; i<data.size(); ++i)
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
        
        void addPoint(Point newPoint, int_2b pointRingPos) //TODO: should already know where because of yaw.
        {
            data[pointRingPos].addPoint(newPoint);
            return;
        }
        
        void addPoint(int_2b yaw, int_2b pitch, int_2b distance, int_2b pointRingPos)
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
            for (int_2b i=0; i<data.size(); ++i)
                data[i].print();
            return;
        }
        
        vector<PointRing> data;
};

void savePoint(PointCloud &point_cloud, Point point)
{
    
    return;   
}

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:

}
