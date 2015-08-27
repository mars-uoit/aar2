#include <ros/ros.h>
#include <cstdio>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float32.h>

tf::Quaternion q;

void broadcastTf(const std_msgs::Float32::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
//These value are wrong for the AAR2, they have to be updated
  transform.setOrigin(tf::Vector3(0.5, 0.0, 0.5) );
  tf::Quaternion q; 
  q.setRPY(0, msg->data, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser" ));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "laserTfBroadcaster_Powerbot");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("pub_theta", 1000, broadcastTf);
  ros::spin();
  return 0;
}

