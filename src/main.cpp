/*
          _____                    _____                    _____          
         /\    \                  /\    \                  /\    \         
        /::\    \                /::\    \                /::\    \        
       /::::\    \               \:::\    \              /::::\    \       
      /::::::\    \               \:::\    \            /::::::\    \      
     /:::/\:::\    \               \:::\    \          /:::/\:::\    \     
    /:::/__\:::\    \               \:::\    \        /:::/  \:::\    \    
   /::::\   \:::\    \              /::::\    \      /:::/    \:::\    \   
  /::::::\   \:::\    \    ____    /::::::\    \    /:::/    / \:::\    \  
 /:::/\:::\   \:::\____\  /\   \  /:::/\:::\    \  /:::/    /   \:::\ ___\ 
/:::/  \:::\   \:::|    |/::\   \/:::/  \:::\____\/:::/____/     \:::|    |
\::/    \:::\  /:::|____|\:::\  /:::/    \::/    /\:::\    \     /:::|____|
 \/_____/\:::\/:::/    /  \:::\/:::/    / \/____/  \:::\    \   /:::/    / 
          \::::::/    /    \::::::/    /            \:::\    \ /:::/    /  
           \::::/    /      \::::/____/              \:::\    /:::/    /   
            \::/____/        \:::\    \               \:::\  /:::/    /    
             ~~               \:::\    \               \:::\/:::/    /     
                               \:::\    \               \::::::/    /      
                                \:::\____\               \::::/    /       
                                 \::/    /                \::/____/        
                                  \/____/                  ~~              

By ChronEngi                             
*/
#include <Arduino.h>
#include <ESP32Servo.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <PID_v1.h>

//  Definizione dei parametri del sistema
const int DIST_MIN_CALIB = 50;      // distanza minima misurata con il sensore (mm) (pallina a -6 cm)
const int DIST_MAX_CALIB = 150;     // distanza massima misurata con il sensore (mm) (pallina a +6 cm)

float POS_MIN = -4.0;  // posizione in cm corrispondente a DIST_MIN_CALIB
float POS_MAX =  4.0;  // posizione in cm corrispondente a DIST_MAX_CALIB
float ball_x;

// -- FUNCTIONS -- //
//  Convert sensor value to position in X
float distanzaToPosizione(int);
//  Move the rotor but in the relative center degree
void move(int);
//  Check any entry serial to change parameters
void serialSet();
//  Change the frequency dynamically
void dynamicFrequency();
//  LED output
void LED(int R, int G, int B);

//  Motor component
Servo motor;

//  Definisci sensore
VL53L0X lox;

//  Define Variables we'll be connecting to
double Setpoint, Input, Output;

//  Specify the links and initial tuning parameters
double Kp=1, Ki=1.2, Kd=0.5;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double Error = 0;

//  PID tolleranza
float tollerance = 1.8;

//  Variables for stabilitation
unsigned long stableStartTime = 0;
bool stable = false;

//  String for serial communication
String incomingString = ""; 

void setup()
{
  //  RGB
  LED(255,0,255);

  //  Attach to the motor
  motor.attach(33);

  //  Initialize the motor position
  move(0);
  delay(2000);

  // I2C distance sensor
  Wire.begin(25, 26); // SDA, SCL per ESP32
  Wire.setClock(100000); // Use standard frequency 100kHz

  //  Serial comunication
  Serial.begin(115200);
  
  Serial.println("Ricerca sensore distanza...");
  
  //  Inizialitation of sensor
  lox.setTimeout(500);
  while (!lox.init()) 
  {
    Serial.println("Sensor no reply...");

    //  Ping pong ball on sensor
    move(-5);
    delay(300);
    move(10);
    delay(200);
  } 

  //  Display that the sensor is found and ready!
  Serial.println("Sensor ready!");

  //  Set the timing to 20ms (50Hz)
  if (!lox.setMeasurementTimingBudget(20000)) 
  {
    Serial.println("Error while setting the budget.");
  }

  //  Turn on the LED, everthing is ready!
  LED(0,0,255);

  //  PID!!!
  //  Turn the PID on
  myPID.SetMode(AUTOMATIC);

  //  Set up limits
  myPID.SetOutputLimits(-40,40);

  //  PID parameters
  Setpoint = 0;
}

void loop()
{
  //  Chech any K change
  serialSet();

  // Read the value
  uint16_t distanza_mm = lox.readRangeSingleMillimeters();

  if (!lox.timeoutOccurred())
  {
    // Get position in X axis
    float posizione_cm = distanzaToPosizione(distanza_mm);

    //  Serial
    Serial.print(">");
    Serial.print("posizione_cm:");
    Serial.print(posizione_cm,2);
    Serial.print("\r\n");

    ball_x = posizione_cm;
  }
  else
  {
    Serial.println("Misure not valid");
  }

  //  PID!!!!

  //  Set our Input (position of ball)
  Input = ball_x;

  //  Computer all the math of the PID
  myPID.Compute();
  
  //  Calculate the error
  Error = Setpoint - Input;

  //  Set the servo to the position to fix the error
  move(-Output);

  //  Change the frequency dynamically
  dynamicFrequency();
}

float distanzaToPosizione(int distanza_mm) {
    // Clamping della lettura nel range di calibrazione
    if (distanza_mm < DIST_MIN_CALIB) distanza_mm = DIST_MIN_CALIB;
    if (distanza_mm > DIST_MAX_CALIB) distanza_mm = DIST_MAX_CALIB;

    // Calcolo del rapporto (distanza_mm - min) / (max - min)
    float ratio = (float)(distanza_mm - DIST_MIN_CALIB) / (float)(DIST_MAX_CALIB - DIST_MIN_CALIB);
    
    // Mapping lineare dal range [DIST_MIN_CALIB, DIST_MAX_CALIB] al range [POS_MIN, POS_MAX]
    float posizioneX = POS_MIN + ratio * (POS_MAX - POS_MIN);

    return posizioneX;
}
void move(int value)
{
  //  Limit the range
  if (value > 40)
  {
    value = 40;
  }
  if (value < -40)
  {
    value = -40;
  }

  //  Move the motor but at the relative center
  if(stable == false)
    motor.write(value+100-3);

  //  Close the function
  return;
}
void serialSet() 
{
    // Read the incoming string until a newline character is found
    if (Serial.available() > 0)
    {
        incomingString = Serial.readStringUntil('\n');
        
        if(incomingString.startsWith("Kp") && incomingString.length() > 3)
        {
          Kp = incomingString.substring(3).toDouble();
          Serial.print("Kp changed to: "); Serial.println(Kp);
          myPID.SetTunings(Kp, Ki, Kd);
          return;
        }

        if(incomingString.startsWith("Ki") && incomingString.length() > 3)
        {
          Ki = incomingString.substring(3).toDouble();
          Serial.print("Ki changed to: "); Serial.println(Ki);
          myPID.SetTunings(Kp, Ki, Kd);
          return;
        }

        if(incomingString.startsWith("Kd") && incomingString.length() > 3)
        {
          Kd = incomingString.substring(3).toDouble();
          Serial.print("Kd changed to: "); Serial.println(Kd);
          myPID.SetTunings(Kp, Ki, Kd);
          return;
        }

        if(incomingString.startsWith("Sp") && incomingString.length() > 3)
        {
          Setpoint = incomingString.substring(3).toDouble();
          Serial.print("Setpoint changed to: "); Serial.println(Setpoint);
          myPID.SetTunings(Kp, Ki, Kd);
          return;
        }

        if(incomingString.startsWith("POS_MAX") && incomingString.length() > 3)
        {
          POS_MAX = incomingString.substring(8).toFloat();
          Serial.print("POS_MAX changed to: "); Serial.println(POS_MAX);
          return;
        }

        if(incomingString.startsWith("POS_MIN") && incomingString.length() > 3)
        {
          POS_MIN = incomingString.substring(8).toFloat();
          Serial.print("POS_MIN changed to: "); Serial.println(POS_MIN);
          return;
        }
    }
}
void dynamicFrequency()
{
  // Check if error is acceptable
  if (Error > -tollerance && Error < tollerance) {
    if (!stable)
    {
      // First moment inside range
      stableStartTime = millis();
      stable = true;
    } 
    else if 
    (millis() - stableStartTime >= 1500) 
    {
      // If stable for 1.5s

      //  Change the frequency
      lox.setMeasurementTimingBudget(450000);
      LED(0,255,0);
    }
  } 
  else 
  {
    // Out of range
    stable = false;

    //  Change the frequency to 50Hz
    lox.setMeasurementTimingBudget(20000);
    LED(0,0,255);
  }
}
void LED(int R, int G, int B)
{
  analogWrite(21, R);
  analogWrite(19, G);
  analogWrite(18, B);

  return;
}