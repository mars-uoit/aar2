#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>

double theta = -0.78539;
int check = 1;

int main(int argc, char** argv){
  ros::init(argc, argv, "laserTfBroadcaster");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;

  while(ros::ok())
  {
    if (theta > 0.78539)
    {
	check = 0;
    }
    if (theta < -0.78539)
    {
        check = 1;
    }
    if (check == 1)
    {
    	theta += 0.05;
    }
    if (check == 0)
    {
	theta -= 0.05;
    }
    q.setRPY(0, theta, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser" ));
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
