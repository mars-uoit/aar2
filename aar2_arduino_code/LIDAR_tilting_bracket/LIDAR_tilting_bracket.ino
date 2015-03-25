#include <AccelStepper.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>
//#include <tf/transform_broadcaster.h>
//#include <laser_geometry/laser_geometry.h>

int stp = 13;  //connect pin 3 to step input
int dir = 12;  // connect pin 2 to direction
int a = 1;     //  gen counter
double theta = 0;  //laser angle

ros::NodeHandle  nh;

std_msgs::Float32 theta_msg;
ros::Publisher pub_theta("pub_theta", &theta_msg);

//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;
//char base_link[] = "/base_link";
//char laser[] = "/laser";

//ros::init(argc, argv, "laserTfBroadcaster");
//ros::NodeHandle node;
//ros::Rate loop_rate(50);
//
//static tf::TransformBroadcaster br;
//tf::Transform transform;
//transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
//tf::Quaternion q;


void setup() 
{                
  nh.initNode();
  nh.advertise(pub_theta);
  
//  broadcaster.init(nh);
  
//  node.initNode();
//  br.init(node);
  
  Serial.begin(9600);
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);       
}


void loop() 
{
  if (a <  50)  //start at 45 degrees from horizontal; 200 steps per rev
  {
    digitalWrite(dir, HIGH); //downwards
    a++;
    digitalWrite(stp, HIGH);   
    delay(10);               
    digitalWrite(stp, LOW);  
    delay(10);

    theta = -(45 - 1.8 * a) * 0.01745; //1.8 degrees per step; 2*pi/360    
  }
  else if(a < 100)
  {
    digitalWrite(dir, LOW); //upwards
    a++;
    digitalWrite(stp, HIGH);  
    delay(10);               
    digitalWrite(stp, LOW);  
    delay(10);
    
    theta = -(1.8 * (a - 50) - 45) * 0.01745; //1.8 degrees per step; 2*pi/360
  }
  else //restart step count
  {
      a = 1;
  }
  
  Serial.println(theta);
  
  theta_msg.data = theta;
  pub_theta.publish( &theta_msg );
  nh.spinOnce();
 
//  t.header.frame_id = base_link;
//  t.child_frame_id = laser;
//  
//  t.transform.translation.x = 1.0; 
//  t.transform.rotation.x = 0.0;
//  t.transform.rotation.y = sin(theta/2);
//  t.transform.rotation.z = 0.0; 
//  t.transform.rotation.w = cos(theta/2);  
//  
//  t.header.stamp = nh.now();
//  broadcaster.sendTransform(t);
//  nh.spinOnce();
//  delay(10);




//  q.setRPY(0, theta, 0);
//  transform.setRotation(q);
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser" ));
//  ros::spinOnce();
//  loop_rate.sleep();
}
