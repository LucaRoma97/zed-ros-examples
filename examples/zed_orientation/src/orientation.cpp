//#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "ros/ros.h"
#include "zed_orientation/Orientation.h"
#include <math.h> 
#define PI 3.14159265
#define baseline 0.12
#define flags 1

ros::Publisher Orientation_pub;
zed_orientation::Orientation marker;

void orientationCallback(const ar_track_alvar_msgs::AlvarMarkers ar_pose_marker)
{
  
  int i;

  for (i=0;i<flags;++i)
    {

	float q0,q1,q2,q3;
	
	q0 = ar_pose_marker.markers[i].pose.pose.orientation.x;
 
	q1 = ar_pose_marker.markers[i].pose.pose.orientation.y;

	q2 = ar_pose_marker.markers[i].pose.pose.orientation.z;      

	q3 = ar_pose_marker.markers[i].pose.pose.orientation.w;
  
 	marker.x = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2))*180/PI;

	marker.y = asin(2*(q0*q2-q3*q1))*180/PI;
	
	marker.z = atan2(2*(q0*q3 + q2*q1),1-2*(q2*q2+q3*q3))*180/PI;
        
        ROS_INFO("angles: [%f],[%f],[%f]", marker.x, marker.y, marker.z);

        Orientation_pub.publish(marker);

    }
}

int main(int argc, char **argv)
{ 
 
  ros::init(argc, argv, "orientation");

  ros::NodeHandle n;

  Orientation_pub = n.advertise<zed_orientation::Orientation>("Orientation", 1000);

  ros::Subscriber sub = n.subscribe("/zed/ar_pose_marker", 1, orientationCallback);

  ros::spin();

  return 0;
}
