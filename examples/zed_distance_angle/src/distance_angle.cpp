
//#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "ros/ros.h"
#include <math.h> 
#define PI 3.14159265

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers ar_pose_marker)
{
  float c, d;
int i;
  
for (i=0;i<=0;++i)
{
  c = sqrt(ar_pose_marker.markers[i].pose.pose.position.x*ar_pose_marker.markers[i].pose.pose.position.x +  ar_pose_marker.markers[i].pose.pose.position.y*ar_pose_marker.markers[i].pose.pose.position.y);
  d = atan(ar_pose_marker.markers[i].pose.pose.position.y/ar_pose_marker.markers[i].pose.pose.position.x)*180/PI;
  
  ROS_INFO("The distance is [%f], the angle is [%f]", c, d);
}
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "distance_angle");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("pose_marker", 1, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
