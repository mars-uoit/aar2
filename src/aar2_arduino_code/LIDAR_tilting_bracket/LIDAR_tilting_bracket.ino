int stp = 13;  //connect pin 3 to step input
int dir = 12;  // connect pin 2 to direction
int a = 0;     //  gen counter

void setup() 
{                
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
  else 
  {
    digitalWrite(dir, LOW); //upwards
    a++;
    digitalWrite(stp, HIGH);  
    delay(10);               
    digitalWrite(stp, LOW);  
    delay(10);
    
    if (a>100)    //sweep 50 in dir 2
    {
      a = 0;
    }
  }
}
