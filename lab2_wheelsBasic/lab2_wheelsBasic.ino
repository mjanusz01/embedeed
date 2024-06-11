#include "Wheels.h"
#include <LiquidCrystal_I2C.h>
#include "PinChangeInterrupt.h"
#include <IRremote.hpp>
#include <Servo.h>
#include "ticker.h"

// defined ports
#define IR_RECEIVE_PIN 9
#define SERVO 11
#define INTINPUT0 A2
#define INTINPUT1 A3
#define TRIG A0
#define ECHO A1

// defined states of app
#define AUTONOMIC_MODE 0
#define CONTROL_MODE 1

// defined states of car
#define STOP_STATE 100
#define FORWARD_STATE 101
#define BACK_STATE 102
#define LEFT_STATE 103
#define RIGHT_STATE 104

// predefined constants
volatile unsigned int CRASH_DISTANCE = 10; // Critical distance of a car, can't go further
volatile unsigned int SERWO_DELAY = 600; // Delay in ms after one read of distance data
volatile unsigned int LEFT_ANGLE = 0;
volatile unsigned int STRAIGHT_ANGLE = 90;
volatile unsigned int RIGHT_ANGLE = 180;

// global variables
unsigned int appState = AUTONOMIC_MODE; // holds current state of app
unsigned int carState = BACK_STATE; // holds current state of vehicle
unsigned int lastDistance = 0; // holds last distance to the obstacle
unsigned int speed = 100; // speed to be displayed
unsigned int leftCount = 0; // counter for left wheel
unsigned int rightCount = 0; // counter for right wheel
volatile int speedLeft = 150; // current speed for left wheel
volatile int speedRight = 150; // current speed for right wheel

// ticker init
Ticker lcd_ticker(500, displayLCD);
Ticker sonar_ticker(500, changeCarState);
Ticker speed_ticker(1000, manageSpeed);

// hardware init
byte LCDAddress = 0x27;
LiquidCrystal_I2C lcd(LCDAddress, 16, 2);
Wheels w;
byte angle;
Servo serwo;

void increment() {
  if(digitalRead(INTINPUT0))
    leftCount++;
  else if(digitalRead(INTINPUT1))
    rightCount++;
}

void displayLCD(){
  lcd.clear();
  lcd.setCursor(0,0);
  if(carState == STOP_STATE){
    lcd.print("Stop");
  } else if(carState == BACK_STATE){
    lcd.print("Back");
  } else if(carState == FORWARD_STATE){
    lcd.print("Forward");
  }
  lcd.setCursor(0,1);
  lcd.print("v=");
  lcd.setCursor(2,1);
  lcd.print(speed);
  lcd.setCursor(7,1);
  lcd.print("d=");
  lcd.setCursor(9,1);
  lcd.print(lastDistance);
}

// TODO: carState = newCarState przed zmianą stanu kół
void appStateChange(int newCarState){
  if(newCarState == STOP_STATE){
    w.stop();
  } else if(newCarState == FORWARD_STATE){
    w.forward();
  } else if(newCarState == LEFT_STATE){
    w.left();
  } else if(newCarState == RIGHT_STATE){
    w.right();
  } else if(newCarState == BACK_STATE){
    w.left();
    w.left();
  }
  carState = newCarState;
}

void manageSpeed(){

  Serial.println("COUNT");
  Serial.print(leftCount);
  Serial.print(" ");
  Serial.print(rightCount);
  float c = float(leftCount)/float(rightCount);
  speedRight = 100;
  w.setSpeedRight(speedRight);
  //w.setSpeedRight(speedRight);

  Serial.print(" SPEED ");
  Serial.print(speedLeft);
  Serial.print(" ");
  Serial.println(speedRight);

  leftCount = 0;
  rightCount = 0;

}

void setup() {
  Serial.println("SETUP ...");

  // Sonar setup
  pinMode(TRIG, OUTPUT);    // TRIG startuje sonar
  pinMode(ECHO, INPUT);     // ECHO odbiera powracający impuls
  Serial.println("Sonar setup correctly.");

  // Wheels sensors setup
  attachPCINT(digitalPinToPCINT(INTINPUT0), increment, CHANGE);
  attachPCINT(digitalPinToPCINT(INTINPUT1), increment, CHANGE);
  Serial.println("Wheels sensors correctly.");

  w.attach(5,4,3,2,7,6);
  w.setSpeedLeft(speedLeft);
  w.setSpeedRight(speedRight);
  Serial.println("Wheels attached correctly.");

  Serial.begin(9600);
  Serial.println("Serial begin on port 9600 setup correctly.");

  serwo.attach(SERVO);
  Serial.println("Serwo attached correctly.");
  
  //IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, 0); // Start the receiver
  //Serial.println("IrReceiver setup correctly.");

  lcd.init();
  lcd.backlight();
  Serial.println("LCD setup correctly");

  if(appState == AUTONOMIC_MODE){
    w.forward();
  }
}

void changeCarState(){
    // Car is going forward. Look forward:
    unsigned int distance = lookAndTellDistance(frontAngle);
    lastDistance = distance;
    if(distance >= 20){
      return;
    }

    // There is an obstacle in front of the car. Check if the car can go left:
    appStateChange(STOP_STATE);
    distance = lookAndTellDistance(LEFT_ANGLE);
    if(distance >= 20){
      appStateChange(LEFT_STATE);
      appStateChange(FORWARD_STATE);
      return;
    }

    // There is an obstacle in front of the car and on the left. Check if the car can go right:
    distance = lookAndTellDistance(RIGHT_ANGLE);
    if(distance >= 20){
      appStateChange(RIGHT_STATE);
      appStateChange(FORWARD_STATE);
      return;
    }

    // Car can't go further in front/left/right direction. The car has to go back

    appStateChange(BACK_STATE);
    appStateChange(FORWARD_STATE);
}

void loop10(){
  if (IrReceiver.decode()) {
      getButtonName(IrReceiver.decodedIRData.decodedRawData);
      //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
      //IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
      //IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
      IrReceiver.resume(); // Enable receiving of the next value
  }
}

void getButtonName(IRRawDataType data){
  switch(data){
    case 3125149440: Serial.println("1"); break;
    case 3108437760: Serial.println("2"); break;
    case 3091726080: Serial.println("3"); break;
    case 3877175040: Serial.println("up"); w.forward(); break;
    case 2907897600: Serial.println("down"); w.back(); break;
    case 4144561920: Serial.println("left"); w.forwardLeft(); break;
    case 2774204160: Serial.println("right"); w.forwardRight(); break;
    case 3810328320: Serial.println("OK");  break;
    default: Serial.println(data); break;
  }
}

unsigned int lookAndTellDistance(byte angle) {
  
  unsigned long tot;      // czas powrotu (time-of-travel)
  unsigned int distance;

  Serial.print("Patrzę w kącie ");
  Serial.print(angle);
  serwo.write(angle);
  delay(SERWO_DELAY);
  
  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  tot = pulseIn(ECHO, HIGH);

  distance = tot/58;

  Serial.print(": widzę coś w odległości ");
  Serial.println(distance);

  return distance;
}

void loop(){
  test_ticker.check();
  sonar_ticker.check();
  speed_ticker.check();
}
