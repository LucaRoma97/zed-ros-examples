//#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "ros/ros.h"
#include "zed_docking/Docking.h" 
#include "zed_distance_angle/DistanceAngle.h"
#include <math.h> 
#define PI 3.14
#define maxangle 2
#define maxorient 1

float maxdistance {4}; // 4 meters
float maxvel {255};
float minvel {70};
float velgap1 {0.1};
float velgap2 {0.05};

ros::Publisher Velocities_pub;
zed_docking::Docking motor;

void velocitiesfunction(const zed_distance_angle::DistanceAngle station1, float gap);

void velocitiesfunction(const zed_distance_angle::DistanceAngle station1, float gap)
{
      motor.left = minvel + (maxvel-minvel)*(station1.distance/maxdistance); //retta minvel to max vel. The more close we are to the station, the lower has to be the velocity
  
      motor.right = motor.left*(1-gap); 
  
      ROS_INFO("Right Vel is [%u], Left Vel is [%u]", motor.right, motor.left);

      Velocities_pub.publish(motor);

}


void dockingCallback(const zed_distance_angle::DistanceAngle station)
{

float h,b,c; // Pose x and y in the plane 
float Phi;

      h = station.distance*cos(station.angle);

      b = station.distance*sin(station.angle);

      c = h/3;
  
// I have to find a orientation for the first step such that we approach at the perpendicular line with a good distance. 
      Phi = atan(b/c)*180/PI;  

// Phi has not to be too high, otherwise the camera will lose the contact with the AR tags
      if (Phi>60)
     	 c = tan(60)*b;
         
// I can compute always Phi since if the robot follows the orientation of Phi, the Phi becomes constant

// there are 2 cases: turn right (clockwise) and turn left. Approach means approaching to the perpendicular line. Adjust means recover the trajectory if Phi becomes higher

// being on the left of the camera, angle is positive and viceversa. being in clockwise orientation is positive and viceversa


  if ((station.angle < -maxangle && station.orientation < Phi)||(station.angle > maxangle && station.orientation < -Phi))  
    {      

      velocitiesfunction(station,velgap1);

    } 

 else if((station.angle > maxangle && station.orientation > -Phi)||(station.angle < -maxangle && station.orientation > Phi))
    {

      velocitiesfunction(station,-velgap1);
	
    }

else if((station.angle < maxangle) && (station.angle>-maxangle))
    {

	if (station.orientation>maxorient)
	  {

     		 velocitiesfunction(station,-velgap2);

	  } 

	else if (station.orientation<-maxorient)
	  {

     		 velocitiesfunction(station,velgap2);

          }

	else
	  {

 		 velocitiesfunction(station,0);

	  }
    }

}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "docking");

  ros::NodeHandle n;

  Velocities_pub = n.advertise<zed_docking::Docking>("Docking", 1000);

  ros::Subscriber sub = n.subscribe("DistanceAngle", 1, dockingCallback);

  ros::spin();

  return 0;
}
