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
#define Motor1L 3
#define Motor1R 5
#define Motor2L 6
#define Motor2R 9

Servo neck;
int Gval, IRval, Ultraval, temps, mic;
int example_value_input = 0;
String Serialval;
char Sensorval[50];
char MV[2];
char TN[2];
char HD[2];
long distance, duration;

struct pt pt_taskGas;
struct pt pt_taskIR;
struct pt pt_taskUltra;
struct pt pt_taskServo;
struct pt pt_taskSendSerial;
struct pt pt_taskMotor;
struct pt pt_taskTemp;
struct pt pt_taskMic;

PT_THREAD(taskSendSerial(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
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
        if (Serialval.length() > 15) {
            memcpy(MV, &Serialval[15], 1);
            memcpy(TN, &Serialval[21], 1);
            memcpy(HD, &Serialval[27], 1);
        }
        Serial1.flush();
    }
}

PT_THREAD(taskGas(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        Gval = analogRead(G_sensor);
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskIR(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        IRval = analogRead(IR_sensor);
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskUltra(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance = (duration / 2) / 29.1;     
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskServo(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        switch (HD[0]) {
            case '1':
                neck.write(170);
                break;
            case '0':
                neck.write(110);
                break;
            case '2':
                neck.write(55);
                break;
        }
        PT_DELAY(pt, 500, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskMotor(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        switch (MV[0]) {
            case '0':
                // Stop.
                digitalWrite(Motor1L, LOW);
                digitalWrite(Motor1R, LOW);
                digitalWrite(Motor2L, LOW);
                digitalWrite(Motor2R, LOW);
                break;
            case '1':
                // Forward.
                if (TN[0] == '1') {
                    digitalWrite(Motor1L, LOW);
                    digitalWrite(Motor1R, LOW);
                    digitalWrite(Motor2L, LOW);
                    digitalWrite(Motor2R, HIGH);
                } else if (TN[0] == '2') {
                    digitalWrite(Motor1L, LOW);
                    digitalWrite(Motor1R, HIGH);
                    digitalWrite(Motor2L, LOW);
                    digitalWrite(Motor2R, LOW);
                } else {
                    digitalWrite(Motor1L, 0);
                    digitalWrite(Motor1R, 1);
                    digitalWrite(Motor2L, 0);
                    digitalWrite(Motor2R, 1);
                }
                break;
            case '2':
                // Backward.
                if (TN[0] == '1') {
                    digitalWrite(Motor1L, LOW);
                    digitalWrite(Motor1R, LOW);
                    digitalWrite(Motor2L, HIGH);
                    digitalWrite(Motor2R, LOW);
                } else if (TN[0] == '2') {
                    digitalWrite(Motor1L, HIGH);
                    digitalWrite(Motor1R, LOW);
                    digitalWrite(Motor2L, LOW);
                    digitalWrite(Motor2R, LOW);
                } else {
                    digitalWrite(Motor1L, 1);
                    digitalWrite(Motor1R, 0);
                    digitalWrite(Motor2L, 1);
                    digitalWrite(Motor2R, 0);
                }
                break;
        }
        switch (TN[0]) {
            case '1':
                // Left.
                if (MV[0] == '0') {
                    digitalWrite(Motor1L, HIGH);
                    digitalWrite(Motor1R, LOW);
                    digitalWrite(Motor2L, LOW);
                    digitalWrite(Motor2R, HIGH);
                }
                break;
            case '2':
                // Right.
                if (MV[0] == '0') {
                    digitalWrite(Motor1L, LOW);
                    digitalWrite(Motor1R, HIGH);
                    digitalWrite(Motor2L, HIGH);
                    digitalWrite(Motor2R, LOW);
                }
                break;
        }
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskTemp(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        temps = ((25 * analogRead(TEMP) - 2050) / 100) - 10;
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskMic(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        mic = analogRead(MIC);
        Serial.println(mic);
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

void setup() {
    // Begin serial connections.
    // Serial1 -> NodeMCU.
    Serial1.begin(115200);
    // Serial -> USB.
    Serial.begin(9600);
    
    // Gas Sensor Pin Initialization.
    pinMode(G_sensor, INPUT);
    // Servo Motor Pin Initialization.
    neck.attach(10);
    neck.write(10);
    // Motor Pins Initialization.
    pinMode(Motor1L, OUTPUT);
    pinMode(Motor1R, OUTPUT);
    pinMode(Motor2L, OUTPUT);
    pinMode(Motor2R, OUTPUT);
    // Ultrasonic Pins Initialzation.
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    // Buzzer Pin Initialization.
    pinMode(Buzzer, OUTPUT);
    // LEDs Pins Initialization.
    pinMode(LED_B, OUTPUT);
    pinMode(LED_G, OUTPUT);
    // Temperature Sensor Pin Initialization.
    pinMode(TEMP, INPUT);
    // Sound Sensor Pin Initialization.
    pinMode(MIC, INPUT);

    // Initialize ProtoThreads.
    PT_INIT(&pt_taskSendSerial);
    PT_INIT(&pt_taskGas);
    PT_INIT(&pt_taskIR);
    PT_INIT(&pt_taskUltra);
    PT_INIT(&pt_taskServo);
    PT_INIT(&pt_taskMotor);
    PT_INIT(&pt_taskTemp);
}

void loop() {
    serialEvent();
    taskSendSerial(&pt_taskSendSerial);
    taskGas(&pt_taskGas);
    taskIR(&pt_taskIR);
    taskUltra(&pt_taskUltra);
    taskServo(&pt_taskServo);
    taskMotor(&pt_taskMotor);
    taskTemp(&pt_taskTemp);
    taskMic(&pt_taskMic);
    if (Gval > 500 || IRval < 100) {
        digitalWrite(Buzzer,HIGH);
        digitalWrite(LED_B,1);
        digitalWrite(LED_G,0);
    } else {
        digitalWrite(Buzzer,LOW);
        digitalWrite(LED_B,0);
        digitalWrite(LED_G,1);
    }
}
