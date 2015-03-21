#include <AccelStepper.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>

int stp = 13;  //connect pin 3 to step input
int dir = 12;  // connect pin 2 to direction
int a = 1;     //  gen counter
double theta;  //laser angle

//ros::NodeHandle  nh;
//
//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;
//
//char base_link[] = "/base_link";
//char laser[] = "/laser";

ros::init(argc, argv, "laserTfBroadcaster");
ros::NodeHandle node;
ros::Rate loop_rate(50);

static tf::TransformBroadcaster br;
tf::Transform transform;
transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
tf::Quaternion q;


void setup() 
{                
//  nh.initNode();
//  broadcaster.init(nh);
  
  node.initNode();
  br.init(node);
  
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);       
}


void loop() 
{
  if (a <  25)  //start at 45 degrees from horizontal; 200 steps per rev
  {
    digitalWrite(dir, HIGH); //downwards
    a++;
    digitalWrite(stp, HIGH);   
    delay(10);               
    digitalWrite(stp, LOW);  
    delay(10);              
  }
  else if(a < 50)
  {
    digitalWrite(dir, LOW); //upwards
    a++;
    digitalWrite(stp, HIGH);  
    delay(10);               
    digitalWrite(stp, LOW);  
    delay(10);
  }
  else if (a>50)  //restart step count
  {
      a = 1;
  }
  
  theta = a * 0.031416; //2*pi rad / 200 steps
 
//  t.header.frame_id = base_link;
//  t.child_frame_id = laser;
//  t.transform.translation.x = 1.0; 
//  t.transform.rotation.x = 0.0;
//  t.transform.rotation.y = theta;
//  t.transform.rotation.z = 0.0; 
//  t.transform.rotation.w = 1.0;  
//  t.header.stamp = nh.now();
//  broadcaster.sendTransform(t);
//  nh.spinOnce();
//  delay(10);

  q.setRPY(0, theta, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser" ));
  ros::spinOnce();
  loop_rate.sleep();
  
}
