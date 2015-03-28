#include <AccelStepper.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>

int stp = 13;  //connect pin 3 to step input
int dir = 12;  // connect pin 2 to direction
int a = 0;     //  gen counter
double theta = 0;  //laser angle

ros::NodeHandle  nh;

std_msgs::Float32 theta_msg;
ros::Publisher pub_theta("pub_theta", &theta_msg);

void setup() 
{                
  nh.initNode();
  nh.advertise(pub_theta);

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
      a = 0;
  }

  theta_msg.data = theta;
  pub_theta.publish( &theta_msg );
  
  nh.spinOnce();
}
