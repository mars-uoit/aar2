#include <AccelStepper.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <tf/transform_broadcaster.h>


int stp = 13;  //connect pin 3 to step input
int dir = 12;  // connect pin 2 to direction
int a = 1;     //  gen counter


ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char laser[] = "/laser";

void setup() 
{                
  nh.initNode();
  broadcaster.init(nh);
  
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);       
}


void loop() 
{
  if (a <  50)  //sweep 50 step in dir 1
  {
    digitalWrite(dir, HIGH); //downwards
    a++;
    digitalWrite(stp, HIGH);   
    delay(10);               
    digitalWrite(stp, LOW);  
    delay(10);              
  }
  else if(a < 100)
  {
    digitalWrite(dir, LOW); //upwards
    a++;
    digitalWrite(stp, HIGH);  
    delay(10);               
    digitalWrite(stp, LOW);  
    delay(10);
  }
  else if (a>100)    //sweep 50 in dir 2
  {
      a = 1;
  }
 
  t.header.frame_id = base_link;
  t.child_frame_id = laser;
  t.transform.translation.x = 1.0; 
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = a; 
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = 1.0;  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();
  delay(10);
}
