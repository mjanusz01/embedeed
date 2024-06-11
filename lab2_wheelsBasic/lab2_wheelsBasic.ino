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

// defined states of car
#define STOP_STATE 100
#define FORWARD_STATE 101
#define BACK_STATE 102
#define LEFT_STATE 103
#define RIGHT_STATE 104

// predefined constants
volatile unsigned int CRASH_DISTANCE = 10; // Critical distance of a car, can't go further
volatile unsigned int SERWO_DELAY = 600 // Delay in ms after one read of distance data

// global variables
unsigned int appState = BACK_STATE; // holds current state of vehicle
unsigned int lastDistance = 0; // holds last distance to the obstacle
unsigned int speed = 100; // speed to be displayed
unsigned int leftCount = 0; // counter for left wheel
unsigned int rightCount = 0; // counter for right wheel
volatile int speedLeft = 100; // current speed for left wheel
volatile int speedRight = 100; // current speed for right wheel

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
  if(appState == STOP_STATE){
    lcd.print("Stop");
  } else if(appState == BACK_STATE){
    lcd.print("Back");
  } else if(appState == FORWARD_STATE){
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

Ticker test_ticker(500, displayLCD);
Ticker test_ticker2(500, makeDecisionWhereToGo);
Ticker test_ticker3(1000, manageSpeed);

void appStateChange(int newAppState){
  if(newAppState == STOP_STATE){
    w.stop();
  } else if(newAppState == FORWARD_STATE){
    w.forward();
  } else if(newAppState == LEFT_STATE){
    w.left();
  } else if(newAppState == RIGHT_STATE){
    w.right();
  } else if(newAppState == BACK_STATE){
    w.back();
  }
  appState = newAppState;
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

  pinMode(TRIG, OUTPUT);    // TRIG startuje sonar
  pinMode(ECHO, INPUT);     // ECHO odbiera powracający impuls

  speed = 0;
  speedLeft = 150;
  speedRight = 150;

  attachPCINT(digitalPinToPCINT(INTINPUT0), increment, CHANGE);
  attachPCINT(digitalPinToPCINT(INTINPUT1), increment, CHANGE);

  //w.attach(9,2,3,4,10,5);
  // 2 4 OK
  //w.attach(2,4,3,6,7,5);
  // OK na odwrot w.attach(2,6,3,4,7,5);
  // OK na odwrot kola w.attach(2,7,3,6,4,5);
  //w.attach(6,2,3,7,4,5);
  w.attach(5,4,3,2,7,6);
    //w.attach(4,,3,5,7,6);


  w.setSpeedLeft(speedLeft);
  w.setSpeedRight(speedRight);
  // 4 OK

  Serial.begin(9600);

  serwo.attach(SERVO);

  Serial.println("Start");

  //w.stop(lcd);
  
  //IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK, 0); // Start the receiver
    
  angle = 0;
  w.forward();

  Serial.println("Initialize LCD");
  lcd.init();
  lcd.backlight();
  Serial.println("LCD initialized correctly");

}

void makeDecisionWhereToGo(){
    // Look forward:

    byte frontAngle = 90;
    serwo.write(frontAngle);
    unsigned int distance = lookAndTellDistance(frontAngle);
    lastDistance = distance;

    // If distance < 20, search through different angles and make a choice:

    if(distance < 20){
      appStateChange(STOP_STATE);
      serwo.write(0);
      delay(SERWO_DELAY);
      distance = lookAndTellDistance(0);
      if(distance >= 20){
        appStateChange(RIGHT_STATE);
        appStateChange(FORWARD_STATE);
      } else{
        serwo.write(180);
        delay(SERWO_DELAY);
        distance = lookAndTellDistance(180);
        if(distance >= 20){
          appStateChange(LEFT_STATE);
          appStateChange(FORWARD_STATE);
        } else{
          serwo.write(90);
          appStateChange(BACK_STATE);
        }
      }
    }

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
  //w.forward();
  //delay(5000);
  test_ticker.check();
  test_ticker2.check();
  test_ticker3.check();
}
