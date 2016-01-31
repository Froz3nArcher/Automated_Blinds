////////////////////////////////////////////////////////////////////////
// Automated Blinds system.  Uses a Light Dependent Resistor (LDR) and a
// Temperature sensor (TBD) to open or close the blinds.
// Motor type, configuration TBD.
// 
// Requirements:
//   1. When dark out, the blinds are closed. (night time)
//   2. When in light, a warm temperature should shut the blinds,
//      a cold temperature should open the blinds.  Open and close temperatures
//      TBD.
//   3. Battery operated, but perhaps needs to detect power source to accommodate
//      desire #1 below.
// Desires
//   1. Somehow be configurable without re-programming.
//      Ideas: USB connection that connects to Wifi?  USB connection that sets
//        the values?  Wifi itself?  Probably not, since that would require
//        continuous operation, and I want to save batteries.  Bluetooth?
//   2. Solar power recharging?
////////////////////////////////////////////////////////////////////////

#include <LowPower.h>
#include <Servo.h>
#include <TMP36.h>

// #define DEBUG_SERIAL

// #define AREF_Voltage 5.0 
#define AREF_Voltage 3.5

#define RUN_INTERVAL 50  // Run every 50 ms

// Number of degrees per interval to move the servo.
#define SERVO_INTERVAL 5

// Delay for 5 seconds after power up.  Allows me to center the servo
// and then disconnect in order to attach/debug.
#define CENTERING_DELAY 5000

////////////////////////////////////////////////////////////////////////
// Temperature Sensor Pin
const int tempInputPin = A1;

// LDR Sensor Pin
const int ldrInputPin = A0;

// define the TMP36 object
TMP36 tempSensor (tempInputPin, AREF_Voltage);
float temperature = 0.0;

const int servoPin = 9;  // Servo Library only supports pin 9 and 10
Servo angleServo;
int motorAngle = 0;

// constants used for state and action
const int OPEN = 0;
const int MOVING = 1;
const int CLOSED = 2;
const int PAUSED = 3;

// current state of the blinds
int motorState = OPEN;
int prevState = CLOSED;

int motorDirection = OPEN;  // 0 = Open, 1 = Moving, 2 = Closed

// flags/vars to handle the movement of the blinds
boolean takeAction = false;
int actionInProgress = 0;

float lightValue = 0.0;  // analog reading

unsigned long previousTime = 0;
unsigned long currentTime = 0;

float DARK_LEVEL = ((500 / 1024.0) * AREF_Voltage);   // TBD level measured to determine darkness
float LIGHT_LEVEL = ((600 / 1024.0) * AREF_Voltage);  // TBD level measured to determine enough light on sensor

float MAXIMUM_TEMPERATURE = 30.0;

// Motor moves at SERVO_INTERVAL between these values.
int SERVO_OPEN = 15;      // completely rotated counterclockwise, angles blinds down
int SERVO_CENTER = 90;
int SERVO_CLOSED = 160;  // completely rotated clockwise, angles blinds upwards

float convertLight (int reading, float refVolts)
{
   return ((reading / 1024.0) * refVolts);
}
   
////////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////////
void setup()
{
#ifdef DEBUG_SERIAL
   Serial.begin (9600);
#endif

   // attach the servo and center it to start.
   angleServo.attach (servoPin);
   motorAngle = SERVO_CENTER;
   angleServo.write (motorAngle);
   
}

////////////////////////////////////////////////////////////////////////
// Compute detrmines whether action needs to be taken according
// to the light level, and sets the direction indication for the
// motor control.
////////////////////////////////////////////////////////////////////////
boolean Compute (float light, float theTemp)
{
   boolean actionNeeded = false;

   switch (motorState)
   {
      ////////////////////////////////////////////////////////////////////////
      case PAUSED:
         // If the light level is below the DARK level, then close the blinds
         // Or if it gets too hot, close the blinds.
         if ((light < DARK_LEVEL) || (theTemp > MAXIMUM_TEMPERATURE))
         {
            actionNeeded = true;
            motorState = MOVING;
            motorDirection = CLOSED;
            angleServo.attach (servoPin);
         }
         else if ((light > LIGHT_LEVEL) && (theTemp < MAXIMUM_TEMPERATURE))
         {
            actionNeeded = true;
            motorState = MOVING;
            motorDirection = OPEN;
            angleServo.attach (servoPin);
         }
         break;
         
      ////////////////////////////////////////////////////////////////////////
      case OPEN:
         // if light is below the dark level, keep the blinds closed regardless.
         if ((light < DARK_LEVEL) || (theTemp > MAXIMUM_TEMPERATURE))
         {
#ifdef DEBUG_SERIAL
            Serial.print ("Close: ");
            Serial.print (light);
            Serial.print (" ");
            Serial.print (theTemp);
            Serial.print ("\n");
#endif
            
            actionNeeded = true;
            motorState = MOVING;
            motorDirection = CLOSED;
            angleServo.attach (servoPin);
         }
         break;

      ////////////////////////////////////////////////////////////////////////
      case MOVING:
         // Guard against moving past the minimum or maximum servo angles.
         if ((motorDirection == OPEN) && (motorAngle > SERVO_OPEN))
         {
            motorAngle = motorAngle - SERVO_INTERVAL;
            if (motorAngle <= SERVO_OPEN)
            {
               // limit the angle commanded
               motorAngle = SERVO_OPEN;
            }
         }
         else if ((motorDirection == CLOSED) && (motorAngle < SERVO_CLOSED))
         {
            motorAngle = motorAngle + SERVO_INTERVAL;
            if (motorAngle >= SERVO_CLOSED)
            {
               // limit the angle commanded
               motorAngle = SERVO_CLOSED;
            }
         }
         else
         {
            if (motorDirection == OPEN)
            {
               motorState = OPEN;
            }
            else
            {
               motorState = CLOSED;
            }
            angleServo.detach ();
         }
         angleServo.write (motorAngle);
         break;

      ////////////////////////////////////////////////////////////////////////
      case CLOSED:
         // if it's daylight out, then we may want to open the blinds.  However,
         // if the temperature is above MAXIMUM, then it's too hot, we want to
         // close the blinds.
         if ((light > LIGHT_LEVEL) && (theTemp < MAXIMUM_TEMPERATURE))
         {
#ifdef DEBUG_SERIAL
            Serial.print ("Open: ");
            Serial.print (light);
            Serial.print (" ");
            Serial.print (theTemp);
            Serial.print ("\n");
#endif
            
            actionNeeded = true;
            motorState = MOVING;
            motorDirection = OPEN;
            angleServo.attach (servoPin);
         }
         break;
   }

   prevState = motorState;
   
   return actionNeeded;
}

////////////////////////////////////////////////////////////////////////
// Main loop
////////////////////////////////////////////////////////////////////////

void loop()
{
   currentTime = millis ();
   
   if (currentTime < CENTERING_DELAY)
   {
      motorState = PAUSED;
   }
   else
   {
      // put your main code here, to run repeatedly:
      if ((currentTime > previousTime + RUN_INTERVAL) || (currentTime < previousTime))
      {
         if (motorState != MOVING)
         {
            // Read the LDR and temperature sensors
            // Don't read the sensors while moving the motor - it throws
            // the values off (at least for temperature)
            lightValue = convertLight (analogRead (ldrInputPin), AREF_Voltage);
            
            temperature = tempSensor.Read ();
         }
         
         // Compute whether action needs to be taken, and which direction.
         takeAction = Compute (lightValue, temperature);

         previousTime = currentTime;
      
         // Only go to sleep if the motor is done moving and there's no action
         // to take.  If there is action, or the motor is moving, we want to
         // complete the event before we go to sleep.
         if (!takeAction && (motorState != MOVING))
         {
            LowPower.powerDown (SLEEP_8S, ADC_OFF, BOD_OFF);
         }
         
      } // currentTime > previousTime + RUN_INTERVAL
      
   } // currentTime > CENTERING_DELAY
}
