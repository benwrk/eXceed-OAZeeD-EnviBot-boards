#include<pt.h>
#include<Servo.h>
#include<string.h>
#include<wire.h>

#define PT_DELAY(pt, ms, ts) \
  ts = millis(); \
  PT_WAIT_WHILE(pt, millis()-ts < (ms));

#define G_sensor A0
#define IR_sensor A1 
#define trigPin 12
#define echoPin 11
#define Buzzer 8
#define LED_B 4
#define LED_G 2
#define TEMP A2
#define MIC A3

Servo neck;

#define Motor1L 3
#define Motor1R 5
#define Motor2L 6
#define Motor2R 9


int Gval ;
int IRval;
int Ultraval;
int example_value_input = 0;
String  Serialval;
char Sensorval[50];
char MV[2];
char TN[2];
char HD[2];
long distance, duration;
int temps;
int mic;

struct pt pt_taskGas;
struct pt pt_taskIR;
struct pt pt_taskUltra;
struct pt pt_taskServo;
struct pt pt_taskSendSerial;
struct pt pt_taskMotor;
struct pt pt_taskTemp;
struct pt pt_taskMic;

//////////////////////////////////////////////
PT_THREAD(taskSendSerial(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
    sendSerial();
    PT_DELAY(pt, 600, ts);
  }
  PT_END(pt);
}

void sendSerial() {
    String fromboard = "EnviBotREPT_GAS[" + String(Gval) + "]_IR[" + String(IRval) + "]_UT[" + String(distance) +"]_TM["+String(temps) +"]_MIC["+String(mic)+"]_!EndREPT";
    Serial.println(fromboard);
    Serial.println(Gval);
    Serial.flush();
    Serial1.print(fromboard);
    Serial1.print('\r');
}

void serialEvent() {
  if (Serial1.available() > 0) { 
    Serialval = Serial1.readStringUntil('\r');
//    Serial.print("value Recieve : ");
//    Serial.println(Serialval);
    if (Serialval.length() > 15){
      memcpy( MV, &Serialval[15], 1);
//      Serial.println(MV);
      memcpy( TN, &Serialval[21], 1);
//      Serial.println(TN);   
      memcpy( HD, &Serialval[27], 1);
    }
    Serial1.flush();
  }
}
//////////////////////////////////////////////    
PT_THREAD(taskGas(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
    Gval = analogRead(G_sensor);
//    Serial.print("Gas = ");
//    Serial.println(Gval);
    
//    Serial.print("\n");
    PT_DELAY(pt, 1000, ts);
  }
  PT_END(pt);
}
/////////////////////////////////////////
PT_THREAD(taskIR(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
    IRval = analogRead(IR_sensor);
//    Serial.print("Infrared = ");
//    Serial.println(IRval);
//    Serial.print("\n");
    PT_DELAY(pt, 1000, ts);
  }
  PT_END(pt);
}
//////////////////////////////////////////
PT_THREAD(taskUltra(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
    //long distance, duration;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) / 29.1;
//    Serial.print("distance = ");
//    Serial.println(distance);
//    Serial.print("\n\n\n");
    
     
    PT_DELAY(pt, 1000, ts);
  }
  PT_END(pt);
}
/////////////////////////////////////////
PT_THREAD(taskServo(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
    //Serial.println("HI");
    switch(HD[0]){
      case '1':
      neck.write(170);
      break;
      case '0':
      neck.write(110);
      break;
      case '2':
      neck.write(55);
      break;
      default:
      break;
    }
    PT_DELAY(pt, 500, ts);
  }
  PT_END(pt);
}
///////////////////////////////////////////
PT_THREAD(taskMotor(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
    switch ( MV[0] ){
      case '0':
      Serial.println("STOP MOVE");
      digitalWrite(Motor1L, LOW);
      digitalWrite(Motor1R, LOW);
      digitalWrite(Motor2L, LOW);
      digitalWrite(Motor2R, LOW);
      break;

      case '1':
      Serial.println("FORWARD");
      if(TN[0] == '1'){
         digitalWrite(Motor1L, LOW);
        digitalWrite(Motor1R, LOW);
        digitalWrite(Motor2L, LOW);
        digitalWrite(Motor2R, HIGH);
      }else if(TN[0] == '2'){
        digitalWrite(Motor1L, LOW);
        digitalWrite(Motor1R, HIGH);
        digitalWrite(Motor2L, LOW);
        digitalWrite(Motor2R, LOW);
      }else {
        digitalWrite(Motor1L, 0);
        digitalWrite(Motor1R, 1);
        digitalWrite(Motor2L, 0);
        digitalWrite(Motor2R, 1);
      }
      break;

      case '2':
      Serial.println("BACKWARD");
      if(TN[0] == '1'){
        digitalWrite(Motor1L, LOW);
        digitalWrite(Motor1R, LOW);
        digitalWrite(Motor2L, HIGH);
        digitalWrite(Motor2R, LOW);
      }else if(TN[0] == '2'){
        digitalWrite(Motor1L, HIGH);
        digitalWrite(Motor1R, LOW);
        digitalWrite(Motor2L, LOW);
        digitalWrite(Motor2R, LOW);
      }else {
        digitalWrite(Motor1L, 1);
        digitalWrite(Motor1R, 0);
        digitalWrite(Motor2L, 1);
        digitalWrite(Motor2R, 0);
      }
      break;

      default:
      break;
    }
    switch ( TN[0] ){

      case '1':
      if(MV[0] == '0'){
        Serial.println("TURN LEFT");
        digitalWrite(Motor1L, HIGH);
        digitalWrite(Motor1R, LOW);
        digitalWrite(Motor2L, LOW);
        digitalWrite(Motor2R, HIGH);
      }
      break;

      case '2':
      if(MV[0] == '0'){
        Serial.println("TURN RIGHT");
        digitalWrite(Motor1L, LOW);
        digitalWrite(Motor1R, HIGH);
        digitalWrite(Motor2L, HIGH);
        digitalWrite(Motor2R, LOW);
      }
      break;

      default:
      break;
    }
     PT_DELAY(pt, 1000, ts);
  }
  PT_END(pt);
}
///////////////////////////////////////////
PT_THREAD(taskTemp(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
   temps = ((25*analogRead(TEMP)-2050)/100)-10;
//   Serial.println(temps);
   PT_DELAY(pt, 1000, ts);
  }
  PT_END(pt);
}
///////////////////////////////////////////
PT_THREAD(taskMic(struct pt* pt))
{
  static uint32_t ts;
  PT_BEGIN(pt);
  while (1)
  {
   mic = analogRead(MIC);
   Serial.println(mic);
   PT_DELAY(pt, 1000, ts);
  }
  PT_END(pt);
}
///////////////////////////////////////////
void setup() {
  Serial1.begin(115200);
  Serial.begin(9600);
//Gas
  pinMode(G_sensor,INPUT);
//Servo  
  neck.attach(10);
  neck.write(10);
//Motor
  pinMode(Motor1L, OUTPUT);
  pinMode(Motor1R, OUTPUT);
  pinMode(Motor2L, OUTPUT);
  pinMode(Motor2R, OUTPUT);
//Ultrasonic
 // distance=0;
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
//Buzzer
  pinMode(Buzzer, OUTPUT);
//LEDs
  pinMode(LED_B, OUTPUT);
  pinMode(LED_G, OUTPUT);
//Temperature
  pinMode(TEMP, INPUT);
//SOUND
  pinMode(MIC, INPUT);
  
  PT_INIT(&pt_taskSendSerial);
  PT_INIT(&pt_taskGas);
  PT_INIT(&pt_taskIR);
  PT_INIT(&pt_taskUltra);
  PT_INIT(&pt_taskServo);
  PT_INIT(&pt_taskMotor);
  PT_INIT(&pt_taskTemp);
}
///////////////////////////////////////////
void loop() {
  serialEvent();
  taskSendSerial(&pt_taskSendSerial);
  taskGas(&pt_taskGas);
  taskIR(&pt_taskIR);  
   if (Gval > 500 || IRval < 100){
    digitalWrite(Buzzer,HIGH);
    digitalWrite(LED_B,1);
    digitalWrite(LED_G,0);
  }
  else{
    digitalWrite(Buzzer,LOW);
    digitalWrite(LED_B,0);
    digitalWrite(LED_G,1);
  }
//  digitalWrite(Motor1R, 1);
//  digitalWrite(Motor2R, 1);
//analogWrite(Motor1R, 255);
//analogWrite(Motor2R, 255);
  taskUltra(&pt_taskUltra);
  taskServo(&pt_taskServo);
  taskMotor(&pt_taskMotor);
  taskTemp(&pt_taskTemp);
  taskMic(&pt_taskMic);
}