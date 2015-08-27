/**************************************************************
 * LIDAR Tilting Mount v2 for AAR                              *
 * Devantech EMG49 gear-motor with encoder                     *
 * Devantech MD49 Dual 24 Volt 5 Amp H Bridge Motor Driver     *
 * Arduino Mega                                                *
 **************************************************************/
#include <ArduinoHardware.h>
#define USE_USBCON  
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <AccelStepper.h>



//Motor Driver Cmd
#define CMD        (byte)0x00                                 
#define GET_VER          0x29
#define GET_ENC1         0x23
#define GET_ENC2         0x24
#define ENC_RESET        0x35
#define GET_VI           0x2C
#define GET_ERROR        0x2D
#define SET_ACCEL        0x33
#define SET_SPEED1       0x31
#define SET_SPEED2       0x32
#define SET_MODE         0x34
#define DIS_REG          0x36
#define DIS_TIMEOUT          0x38


#define STEP_DEG 0.734694 //One step of the motor in ded
#define STEP_RAD 0.012823 //One step of the motor in rad
#define ANGLE_OFFSET 0.41-1,5707*2 //position/angle of the switch is not 0 

//#define ANGLE_OFFSET 0.52      //position/angle of the switch is not 0 

#define MAX_SPEED 6           //Don't set speed over 7
#define COUNT_MAX 100      //100 steps = 73,47 degrees

const byte LED = 13;
const byte BUTTON = 2;

uint32_t count = 0;
volatile boolean Switch_ON = false;
volatile boolean Return = false;
volatile boolean First_Time = true;
volatile boolean No_signal = false;
int count_init = 0;
int count_total = 0;
int count_temp = 1;
int Count_Error = 0;
int Lidar_Link_Error = 0;
double Angle_deg = 0;
double theta = 0;
double Angle_temp = 0;

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char laser[] = "/base_laser";

void(* resetFunc) (void) = 0; //declare reset function at address 0


// ----- Send the transform to ROS ------
void Send_transform()
{
  t.header.frame_id = base_link;
  t.child_frame_id = laser;

  //Static transform, location of the Lidar :
  t.transform.translation.x = 0.27;
  t.transform.translation.z = 0.785; 
  
  //Dynamic transform, concerning the angle of the Lidar
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = sin(theta/2);
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = cos(theta/2);  

  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();
  delay(10);
}


// ----- Read the encoder value of the motor 2 ------
uint32_t encoder2() {
  char chars[4];
  uint8_t ints[4];
  uint32_t encoder = 0;
  while (Serial2.available() > 0) //Buffer needs to be empty
  {
    Serial2.read();
  }
  Serial2.write(CMD);
  Serial2.write(GET_ENC2);
  delay(25); //Delay to receive all the data
  if (Serial2.available() >= 4) {
    Serial2.readBytes(chars, 4);
    for(int i=0; i<4; i++)
    {
      ints[ i] = (uint8_t) chars[ i];
      //         Serial.print(i);
      //         Serial.write(" : ");
      //         Serial.println(ints[i]);     
    }
    encoder = ((ints[0] << 24) +
      (ints[1] << 16) +
      (ints[2] << 8) +
      (ints[3] << 0));
    No_signal=false;
    return encoder;
  }
  else
    No_signal=true; //Boolean testing the signal received from the motor driver
  return NULL;
}

// ----- Checking if the motor driver communicating well ------
void Check_Signal()
{
  if(No_signal)
  {
    while (Serial2.available() == 0)
    {
      Stop();
      encoder2();
    }
    //Signal is back
    No_signal=false;
    resetFunc(); //Arduino reset  
  }
}


// ----- Checking the value of the encoder ------
void Counter_check()
{
  //Security if the counter does not work
  if(((count_temp==count_total)&&(!First_Time))||((abs(count_total))>200))
  {
    Count_Error++;
    if((Count_Error > 100)||((abs(count_total))>200)) 
    {
      Stop();
      delay(1000);
    }
  }
  else
  {
    Count_Error=0;
  }
  count_temp=count_total;
}

// ----- Checking mecanichal problems ------
void Lidar_Link_Check()
{
  if(digitalRead (BUTTON) == HIGH)
  {
    Lidar_Link_Error++;
    if(Lidar_Link_Error>50)
    {
      Stop();
      delay(2000);
      Clockwise_rotation(MAX_SPEED);
      //If the value is HIGH after a clockwise rotation, the tilting macanism is loose
      while(digitalRead (BUTTON) == HIGH)
      { //Arduino stops sending message to the motor driver
      } //After 2 sec without message, the motor will stop
    }
  }
  else
  {
    Lidar_Link_Error=0;
  }
}

void Clockwise_rotation(int s)
{
  Serial2.write(CMD);
  Serial2.write(SET_SPEED2);
  Serial2.write(128+s);
}

void Counterclockwise_rotation(int s)
{
  Serial2.write(CMD);
  Serial2.write(SET_SPEED2);
  Serial2.write(128-s);
}

void Stop()
{
  Serial2.write(CMD);
  Serial2.write(SET_SPEED2);
  Serial2.write(128);
}

void LED_Test()
{
  if (digitalRead (BUTTON) == HIGH)
  {
    digitalWrite (LED, HIGH);
  }
  else
  {
    digitalWrite (LED, LOW);
  }
}

void interrupt()
{  
  Switch_ON = true;
  First_Time = false; 
}

// ----- The init loop of the Arduino ------
void setup()
{ 
  nh.initNode();
  broadcaster.init(nh);
  Serial2.begin(9600);

  Serial2.write(CMD);
  Serial2.write(SET_MODE);
  Serial2.write(0);    //Mode 0 :  0 (Full Reverse)  128 (Stop)   255 (Full Forward)

  delay(500); //To avoid an init bug

  Serial2.write(CMD);
  Serial2.write(ENC_RESET);

  //Interruption on rising edge
  attachInterrupt(0, interrupt, RISING);

  //switch input
  pinMode(2,INPUT);
  digitalWrite(2, INPUT_PULLUP); //To avoid switch bounces 

  //Led Output
  pinMode(13,OUTPUT);
  digitalWrite(13, LOW);
  //Pin controlling the relay
  pinMode(50,OUTPUT);
  digitalWrite(50, LOW);

}

void loop()
{   
  count = encoder2();

  if(!First_Time)
  {
    //Normal behaviour

    count_total=count-count_init;

    //Angle_deg=count_total*STEP_DEG;
    theta=count_total*STEP_RAD+ANGLE_OFFSET;

    if((abs(count_total)>COUNT_MAX)&&(!Return))
    {
      Counterclockwise_rotation(MAX_SPEED);
      Return = true;
    }

    else
    {
      if(Return)
      {
        Counterclockwise_rotation(MAX_SPEED);
      }
      else
      {
        Clockwise_rotation(MAX_SPEED);
      }
    }
  }
  else
  {
    //Initial behaviour : looking for the switch

    if(digitalRead (BUTTON) == HIGH)  //When the Lidar is on the switch at the init state
    {
      Switch_ON = true;
      First_Time = false;
    }
    count = encoder2();
    count_init=count;
    Counterclockwise_rotation(MAX_SPEED);
  }

  if(Switch_ON)
  { 
    count = encoder2();
    count_total=count-count_init;
    if(count_total<0)
    {
      count_init=count;    //Counter initilised
    }
    Clockwise_rotation(MAX_SPEED);
    Switch_ON = false;
    Return = false;
  }

  if((count_total>=0)&&(count_total<=100))
  {
    Send_transform();
  }

  Counter_check();
  Lidar_Link_Check();
  Check_Signal();
}

