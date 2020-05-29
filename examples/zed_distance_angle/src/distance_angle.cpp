
//#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "ros/ros.h"
#include "zed_distance_angle/DistanceAngle.h"
#include <math.h> 
#define PI 3.14159265
#define baseline 0.12
#define flags 1

ros::Publisher DistanceAngle_pub;
zed_distance_angle::DistanceAngle marker;

void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers ar_pose_marker)
{
  
  int i;

  for (i=0;i<flags;++i)
    {

      float x,y;
      x = ar_pose_marker.markers[i].pose.pose.position.x;
      /* we detect the tags from the left camera. To get the pose with respect the center of the ZED we subtract half  baseline */
      y = ar_pose_marker.markers[i].pose.pose.position.y + baseline/2;

      marker.distance = sqrt(x*x + y*y);

      marker.angle = atan(y/x)*180/PI;
  
      ROS_INFO("The distance is [%f], the angle is [%f]", marker.distance, marker.angle);

      DistanceAngle_pub.publish(marker);

    }
}

int main(int argc, char **argv)
{ 
 
  ros::init(argc, argv, "distance_angle");

  ros::NodeHandle n;

  DistanceAngle_pub = n.advertise<zed_distance_angle::DistanceAngle>("DistanceAngle", 1000);

  ros::Subscriber sub = n.subscribe("pose_marker", 1, chatterCallback);

  ros::spin();

  return 0;
}
