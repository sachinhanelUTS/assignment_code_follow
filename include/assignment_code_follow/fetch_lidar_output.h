
#ifndef fetch_lidar_output_h
#define fetch_lidar_output_h

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