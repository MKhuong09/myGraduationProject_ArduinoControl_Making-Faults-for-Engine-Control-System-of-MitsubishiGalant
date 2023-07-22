/*=======================================================Graduation Project 2022-2023==================================================================*/
/*=====================================================Automotive Engineering Technology===============================================================*/
/*==============================================Mitsubishi Galant 1991: Model of Engine Control System=================================================*/
/*======================================Creating Pan and controlling Engine Speed by Phone Bluetooth Control===========================================*/
/*Author: Tran Vinh Manh Khuong*/
/*Student ID: 19145252*/
/*HoChiMinh city University of Technology and Education*/
/*Teamate: Nguyen Trung Nguyen*/
/*Instructor: MS. Nguyen Thien Dinh*/


#include "String.h"
#include "SoftwareSerial.h"


#define ST          12
#define RPM_read    A0
#define IGKey       13
#define R_PWM       11
#define L_PWM       10
#define rx_pin      A2
#define tx_pin      A3
#define INPUT_PULLDOWN  (0x3)

class Switch{
  private:
    uint8_t Pin;
    uint8_t numberSW;
  public:
    Switch(uint8_t Pin, uint8_t numberSW){
      this->numberSW = numberSW;
      this->Pin = Pin;
      pinMode(Pin, OUTPUT);
      digitalWrite(Pin, LOW);
    }
    int getNumberSW (void){
      return numberSW;
    }
    void ON_OFF (String command){
      if(command == "ON")
      {
        digitalWrite(this->Pin, LOW);
      }
      if(command == "OFF")
      {
        digitalWrite(this->Pin, HIGH);
      }
    }
};

//=====================================================
SoftwareSerial mySerial(rx_pin,tx_pin);
uint16_t speedControl = 0;
uint16_t rpmReq = 0;
bool bluetooth = 0;
bool statusEngine = 0;
bool statusIG = 0;

uint64_t lastIGT1 = 0, lastIGT2 = 0, timeIGT1 = 0, timeIGT2 = 0;

String message_rx, message_rx_switch;
uint8_t message_rx_throttle = 50;
uint8_t swNumber = 0;

void SpeedMotor(void);
void motor_init(void);
void relay_init(void);
void getMessage (void);
void client_process (void);
void fault_process(void);
//=====================================================

/////////////////Interrupt IGT1-IGT2///////////////////
void IGT1(){
  lastIGT1 = millis();
}

void IGT2(){
  lastIGT2 = millis();
}

////////////////////////Relay Declaration/////////////////////////////

Switch sw[7] = {{9,1},{8,2},{7,3},{6,4},{5,5},{4,6},{A1,7}};
//////////////////////////////////////////////////////////////////////

void setup() {
  
  mySerial.begin(9600);
  //VCC for HC-05 module//
  pinMode(1,OUTPUT);
  digitalWrite(1,HIGH);
  
  //Start Motor Init:
  pinMode(ST,INPUT_PULLDOWN);
  pinMode(IGKey,INPUT_PULLDOWN);
  pinMode(RPM_read, INPUT);
  motor_init();

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, IGT1, CHANGE);
  attachInterrupt(1, IGT2, CHANGE); 
  ///////Wait for Starting Engine///////
  while(digitalRead(ST)==0);
}

void loop() {
  
  /////////////////Check status Ignition////////////////////
  
  statusIG = digitalRead(IGKey);
  
  ///////ON-OFF Motor control/////////
  if(digitalRead(ST)==1)
  {
    statusEngine = 1;
  }
  
  ////////////////Engine Speed Control//////////////// 
  if((statusEngine == 1) && (statusIG == 1))
  {
     SpeedMotor();
  }
  else
  {
    analogWrite(L_PWM,0);
    analogWrite(R_PWM,0);
    delay(1000);
    statusEngine = 0;
  }
  
  /////////////////////////////////////////////////////

  ////////////////////Check SPARK event//////////////////////
  timeIGT1 = millis() - lastIGT1;
  timeIGT2 = millis() - lastIGT2;

  if((timeIGT1 >= 100) && (timeIGT2 >= 100))
  {
    statusEngine = 0;
  }
  else
  {
    statusEngine = 1;
  }
  ///////////////////////////////

  ////////////////////Phone Bluetooth Control/////////////////////
  
   getMessage();
   /////////////Make Faults/////////////
   if(bluetooth)
    {
      if(swNumber > 0){
        fault_process();
        swNumber = 0;
      }
    }
   /////////////Check whether Connected to Phone Control//////////////////////
   if(message_rx == "BEGIN"){
      bluetooth = 1;
      message_rx ="";
    }
    if(message_rx == "END"){
      bluetooth = 0;
      message_rx ="";
    }

  //////////////////////////////////////////////////////////////////////////
  delay(10);
  
}

void motor_init()
{
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(A5, OUTPUT);      //R_EN PIN
  pinMode(A4, OUTPUT);      //L_EN PIN
  digitalWrite(A5,HIGH);
  digitalWrite(A4,HIGH);
}

void SpeedMotor ()
{
    //Speed Control=================================================
    
    if(bluetooth == 1){
      analogWrite(L_PWM,message_rx_throttle);
      analogWrite(R_PWM,0);
    }
    else{
      speedControl = analogRead(RPM_read);
      rpmReq = map(speedControl,0,1023,50,255);
      analogWrite(L_PWM,rpmReq);  
      analogWrite(R_PWM,0);
    } 
}

void client_process ()
{
  if((message_rx.indexOf("W") >= 0))
  {
    message_rx_switch = message_rx.substring(message_rx.indexOf("W")+2,message_rx.indexOf("s"));
    swNumber = message_rx.substring(message_rx.indexOf("W")+1,message_rx.indexOf("W")+2).toInt(); 
  }
  
  if((message_rx.indexOf("H") >= 0)){
    message_rx_throttle = message_rx.substring(message_rx.indexOf("H")+1,message_rx.indexOf("p")).toInt();
  }
}

void getMessage ()
{
      message_rx ="";
  if(mySerial.available())
  {
    while(mySerial.available())
    {
      char data = (char)mySerial.read();
      message_rx += data;
      //message_rx = Serial.readString();
    }
    client_process();
  }
}

void fault_process(){
    sw[swNumber-1].ON_OFF(message_rx_switch);
}
/////////////////////////////////////////////The End///////////////////////////////////////////////////
