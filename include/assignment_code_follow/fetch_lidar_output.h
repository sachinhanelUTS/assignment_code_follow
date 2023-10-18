
#ifndef LIDAR_OUTPUT
#define LIDAR_OUTPUT

#include <sensor_msgs/LaserScan.h>

class LaserDetection
{
private:
  double laser_reading_; 
public:
 
  LaserDetection();
 
  bool detectObtacle(sensor_msgs::LaserScan::ConstPtr laserScan);
  
  double getLaserReading(sensor_msgs::LaserScan::ConstPtr laserScan);
};

#endif 