
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

void distanceangleCallback(const ar_track_alvar_msgs::AlvarMarkers ar_pose_marker)
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

	/* from quaternion to euler angles */

        float q0,q1,q2,q3, Yaw, Pitch, Roll;
	
	q0 = ar_pose_marker.markers[i].pose.pose.orientation.x;
 
	q1 = ar_pose_marker.markers[i].pose.pose.orientation.y;

	q2 = ar_pose_marker.markers[i].pose.pose.orientation.z;      

	q3 = ar_pose_marker.markers[i].pose.pose.orientation.w;
  
 	Yaw = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2))*180/PI;

	Pitch = asin(2*(q0*q2-q3*q1))*180/PI;
	
	Roll = atan2(2*(q0*q3 + q2*q1),1-2*(q2*q2+q3*q3))*180/PI;

	marker.orientation = 90 + Yaw;
  
        ROS_INFO("Distance: [%f], angle: [%f], orientation: [%f]", marker.distance, marker.angle, marker.orientation);

        DistanceAngle_pub.publish(marker);

    }
}

int main(int argc, char **argv)
{ 
 
  ros::init(argc, argv, "distance_angle");

  ros::NodeHandle n;

  DistanceAngle_pub = n.advertise<zed_distance_angle::DistanceAngle>("DistanceAngle", 1000);

  ros::Subscriber sub = n.subscribe("pose_marker", 1, distanceangleCallback);

  ros::spin();

  return 0;
}
