#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define USMIN  1000//600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2000//2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint16_t Pos1;
uint16_t Pos2;
uint16_t Pos3;
uint16_t Pos0;
 int ang;
 int ang2;
 int ang3;
 int ang0;
 int X;  //variable storing data from the index finger (landmark 8)
 int Y;  //variable storing data from the middle finger(landmark 12)
 int Z;  //variable storing data from the ring finger (landmark 16)
 int J;  //variable storing data from the bottom of the palm (landmark 0)
 int K;  //variable storing data from the thumb finger (landmark )
 int L;  //variable storing data from the thumb finger (landmark )
 const int RELAY_PIN = 3; // auction pump relay pin
 const int RELAY_PIN = 3;

void setup() {
  Serial.begin(9600);
  Serial.println("Ready"); // prints ready on the serial port
  pinMode(RELAY_PIN, OUTPUT);  //setting relay pin to output
  digitalWrite(RELAY_PIN, LOW);  //putting off relay pin
  Pos1 = 1500;
  Pos2 = 1500;
  Pos3 = 1500;
  Pos0 = 1500;
  pwm.begin();


  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  for(int i = 0; i < 4; i++) {  // loop to position the servo motors in their initial state after startup
  
    pwm.writeMicroseconds(i, Pos1);
    delay(10);

}}



void Postn()
{
    
     if ((X < 90) && (Pos1 > USMIN) && (Pos1 < USMAX)) // if the tracked landmark (stored as X) is lower than the lower trigger (90) and
    {                                                  // and the current position (pos1) of the servo is greater than its minimum limit (usmin)
      Pos1 -= 20;                                      //and the current position of the servo is less than its maximum limit (usmax)
    pwm.writeMicroseconds(1, Pos1);                     // the servo position moves in steps of 20
    Serial.print("X = ");                               // a signal is sent to the servo to effect the change in position (pos1)
    Serial.println(Pos1);
      if (Pos1 == USMIN)                            // if the variable storing the position of the servo gets to the min limit (servomax)
      {Pos1 = USMIN + 20;}                          // the servo will get stuck; hence, this line reduces its value at the end of this section of the code. you can comment this line and run a test to better understand 
    }

     if ((X > 150) && (Pos1 > USMIN) && (Pos1 < USMAX))  // if the tracked landmark (stored as X) is higher than the uppwer trigger (150) and
    {                                                     // and the current position (pos1) of the servo is greater than its minimum limit (usmin)
      Pos1 += 20;                                         //and the current position of the servo is less than its maximum limit (usmax)
    pwm.writeMicroseconds(1, Pos1);                         // a signal is sent to the servo to effect the change in position (pos1)
    Serial.print("X = ");
    Serial.println(Pos1);
    if (Pos1 == USMAX)                             // if the variable storing the position of the servo gets to the max limit (servomax)
      {Pos1 = USMAX - 20;}                      // the servo will get stuck; hence, this line reduces its value at the end of this section of the code. you can comment this line and run a test to better understand 
    }



    if ((Y < 70) && (Pos2 > USMIN) && (Pos2 < USMAX))   // the logic above applies to the here and for the lest of this function
    {
      Pos2 -= 20;
    pwm.writeMicroseconds(2, Pos2);
    Serial.print("Y = ");
    Serial.println(Pos2);
      if (Pos2 == USMIN)
      {Pos2 = USMIN + 20;} 
    }

     if ((Y > 130) && (Pos2 > USMIN) && (Pos2 < USMAX))
    {
      Pos2 += 20;
    pwm.writeMicroseconds(2, Pos2);
    Serial.print("Y = ");
    Serial.println(Pos2);
    if (Pos2 == USMAX)
      {Pos2 = USMAX - 20;} 
    }



       if ((Z < 70) && (Pos3 > USMIN) && (Pos3 < USMAX))
      {
      Pos3 += 20;
    pwm.writeMicroseconds(3, Pos3);
    Serial.print("Z = ");
    Serial.println(Pos3);
    if (Pos3 == USMAX)
      {Pos3 = USMAX - 20;} 
    }

     if ((Z > 130) && (Pos3 > USMIN) && (Pos3 < USMAX))
  {
      Pos3 -= 20;
    pwm.writeMicroseconds(3, Pos3);
    Serial.print("Z = ");
    Serial.println(Pos3);
      if (Pos3 == USMIN)
      {Pos3 = USMIN + 20;} 
    }
    


    if ((J < 220) && (Pos0 > USMIN) && (Pos0 < USMAX))
    {
      Pos0 += 20;
    pwm.writeMicroseconds(0, Pos0);
    Serial.print("J = ");
    Serial.println(Pos0);
    if (Pos0 == USMAX)
      {Pos0 = USMAX - 20;} 
    }

     if ((J > 370) && (Pos0 > USMIN) && (Pos0 < USMAX))
    {
      Pos0 -= 20;
    pwm.writeMicroseconds(0, Pos0);
    Serial.print("J = ");
    Serial.println(Pos0);
      if (Pos0 == USMIN)
      {Pos0 = USMIN + 20;} 
    }

      if (K > L)   //The pump function checks if hand landmark 4 (tip of thumb) is to the right or left side 
    {                 //of hand landmark 3 (IP joint), this in turn switches on and off the suction pump.
     digitalWrite(RELAY_PIN, LOW);
     Serial.print("K =");
    Serial.println(K);
    Serial.print("L =");
    Serial.println(L);
 
    }

     if (K < L)
    {
     digitalWrite(RELAY_PIN, HIGH);
      Serial.print("K");
    Serial.println(K);
    Serial.print("L");
    Serial.println(L);
    }
    

    }

  


void loop() {//   The ‘loop’ function checks the serial port for data being transmitted 
                 //from the computer; depending on the data it gets, it calls upon the ‘postn’ function.
                 // or the pump function

if(Serial.available() > 0)
  {
    if(Serial.read() == 'X')
    {
      X = Serial.parseInt();
      if(Serial.read() == 'Y')
      {
        Y = Serial.parseInt();
        if(Serial.read() == 'Z')
       {
        Z = Serial.parseInt();
        if(Serial.read() == 'J')
       {
        J = Serial.parseInt();
        if(Serial.read() == 'K')
       {
        K = Serial.parseInt();
       if(Serial.read() == 'L')
       {
        L = Serial.parseInt();
       Postn();
       }
      }
      }
      }
      }
    }
    while(Serial.available() > 0)
    {
      Serial.read();
    }
  
}

   
}
