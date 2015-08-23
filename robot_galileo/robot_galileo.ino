#include<pt.h>
#include<Servo.h>
#include<string.h>
#include<wire.h>

// ProtoThread.
#define PT_DELAY(pt, ms, ts) \
  ts = millis(); \
  PT_WAIT_WHILE(pt, millis()-ts < (ms));


// GPIO Pins Definition.
#define G_SENSOR A0
#define IR_SENSOR A1
#define TEMP_SENSOR A2
#define SOUND_SENSOR A3
#define LED_G 2
#define MOTOR1_RV 3
#define LED_B 4
#define MOTOR1_FW 5
#define MOTOR2_RV 6
#define BUZZER 8
#define MOTOR2_FW 9
#define SERVO 10
#define ULTRA_ECHO 11
#define ULTRA_TRIG 12

Servo head;
int gasValue, irValue, tempValue, micValue;
String receivedSerial;
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
    String fromboard = "EnviBotREPT_GAS[" + String(gasValue) + "]_IR[" + String(irValue) + "]_UT[" + String(distance) +"]_TM["+String(tempValue) +"]_MIC["+String(micValue)+"]_!EndREPT";
    Serial.println(fromboard);
    Serial.println(gasValue);
    Serial.flush();
    Serial1.print(fromboard);
    Serial1.print('\r');
}

void serialEvent() {
    if (Serial1.available() > 0) { 
        receivedSerial = Serial1.readStringUntil('\r');
        if (receivedSerial.length() > 15) {
            memcpy(MV, &receivedSerial[15], 1);
            memcpy(TN, &receivedSerial[21], 1);
            memcpy(HD, &receivedSerial[27], 1);
        }
        Serial1.flush();
    }
}

PT_THREAD(taskGas(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        gasValue = analogRead(G_SENSOR);
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskIR(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        irValue = analogRead(IR_SENSOR);
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskUltra(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        digitalWrite(ULTRA_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRA_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRA_TRIG, LOW);
        duration = pulseIn(ULTRA_ECHO, HIGH);
        distance = (duration / 2) / 29.1;     
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskServo(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    
    /////////////////////////////////////
    // Head Ctrl Variables Description //
    /////////////////////////////////////
    // HD[0] <- Head Control Value     //
    //   -> '0' == Center              //
    //   -> '1' == Look Left           //
    //   -> '2' == Look Right          //
    /////////////////////////////////////
    
    while (1) {
        switch (HD[0]) {
            case '1':
                head.write(170);
                break;
            case '0':
                head.write(110);
                break;
            case '2':
                head.write(55);
                break;
        }
        PT_DELAY(pt, 500, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskMotor(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    
    /////////////////////////////////////
    // Movement Variables Description  //
    /////////////////////////////////////
    // MV[0] <- Movement Control Value //
    //   -> '0' == Hold Postion (Stop) //
    //   -> '1' == Move Forward        //
    //   -> '2' == Move Backward       //
    /////////////////////////////////////
    // TN[0] <- Turn Control Value     //
    //   -> '0' == No Turning          //
    //   -> '1' == Turn Left           //
    //   -> '2' == Turn Right          //
    /////////////////////////////////////
    
    while (1) {
        switch (MV[0]) {
            case '0':
                // Stop.
                digitalWrite(MOTOR1_RV, LOW);
                digitalWrite(MOTOR1_FW, LOW);
                digitalWrite(MOTOR2_RV, LOW);
                digitalWrite(MOTOR2_FW, LOW);
                break;
            case '1':
                // Forward.
                if (TN[0] == '1') {
                    digitalWrite(MOTOR1_RV, LOW);
                    digitalWrite(MOTOR1_FW, LOW);
                    digitalWrite(MOTOR2_RV, LOW);
                    digitalWrite(MOTOR2_FW, HIGH);
                } else if (TN[0] == '2') {
                    digitalWrite(MOTOR1_RV, LOW);
                    digitalWrite(MOTOR1_FW, HIGH);
                    digitalWrite(MOTOR2_RV, LOW);
                    digitalWrite(MOTOR2_FW, LOW);
                } else {
                    digitalWrite(MOTOR1_RV, LOW);
                    digitalWrite(MOTOR1_FW, HIGH);
                    digitalWrite(MOTOR2_RV, LOW);
                    digitalWrite(MOTOR2_FW, HIGH);
                }
                break;
            case '2':
                // Backward.
                if (TN[0] == '1') {
                    digitalWrite(MOTOR1_RV, LOW);
                    digitalWrite(MOTOR1_FW, LOW);
                    digitalWrite(MOTOR2_RV, HIGH);
                    digitalWrite(MOTOR2_FW, LOW);
                } else if (TN[0] == '2') {
                    digitalWrite(MOTOR1_RV, HIGH);
                    digitalWrite(MOTOR1_FW, LOW);
                    digitalWrite(MOTOR2_RV, LOW);
                    digitalWrite(MOTOR2_FW, LOW);
                } else {
                    digitalWrite(MOTOR1_RV, HIGH);
                    digitalWrite(MOTOR1_FW, LOW);
                    digitalWrite(MOTOR2_RV, HIGH);
                    digitalWrite(MOTOR2_FW, LOW);
                }
                break;
        }
        switch (TN[0]) {
            case '1':
                // Left.
                if (MV[0] == '0') {
                    digitalWrite(MOTOR1_RV, HIGH);
                    digitalWrite(MOTOR1_FW, LOW);
                    digitalWrite(MOTOR2_RV, LOW);
                    digitalWrite(MOTOR2_FW, HIGH);
                }
                break;
            case '2':
                // Right.
                if (MV[0] == '0') {
                    digitalWrite(MOTOR1_RV, LOW);
                    digitalWrite(MOTOR1_FW, HIGH);
                    digitalWrite(MOTOR2_RV, HIGH);
                    digitalWrite(MOTOR2_FW, LOW);
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
        tempValue = ((25 * analogRead(TEMP_SENSOR) - 2050) / 100) - 10;
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

PT_THREAD(taskMic(struct pt* pt)) {
    static uint32_t ts;
    PT_BEGIN(pt);
    while (1) {
        micValue = analogRead(SOUND_SENSOR);
        Serial.println(micValue);
        PT_DELAY(pt, 1000, ts);
    }
    PT_END(pt);
}

void setup() {
    // Begin serial connections.
    // Serial1 -> NodeMCU.
    Serial1.begin(115200);
    // Serial -> Serial over USB.
    Serial.begin(9600);
    
    // Gas Sensor Pin Initialization.
    pinMode(G_SENSOR, INPUT);
    // Servo Motor Pin Initialization.
    head.attach(SERVO);
    head.write(SERVO);
    // Motor Pins Initialization.
    pinMode(MOTOR1_RV, OUTPUT);
    pinMode(MOTOR1_FW, OUTPUT);
    pinMode(MOTOR2_RV, OUTPUT);
    pinMode(MOTOR2_FW, OUTPUT);
    // Ultrasonic Pins Initialzation.
    pinMode(ULTRA_TRIG, OUTPUT);
    pinMode(ULTRA_ECHO, INPUT);
    // BUZZER Pin Initialization.
    pinMode(BUZZER, OUTPUT);
    // LEDs Pins Initialization.
    pinMode(LED_B, OUTPUT);
    pinMode(LED_G, OUTPUT);
    // Temperature Sensor Pin Initialization.
    pinMode(TEMP_SENSOR, INPUT);
    // Sound Sensor Pin Initialization.
    pinMode(SOUND_SENSOR, INPUT);

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
    serialEvent();                          // Receive control data from NodeMCU.
    taskSendSerial(&pt_taskSendSerial);     // Send environment data to NodeMCU.
    taskGas(&pt_taskGas);                   // Read gas sensor's value.
    taskIR(&pt_taskIR);                     // Read infrared light sensor's value.
    taskUltra(&pt_taskUltra);               // Read ultrasonic sensor's value.
    taskTemp(&pt_taskTemp);                 // Read temperature sensor's value.
    taskMic(&pt_taskMic);                   // Read sound sensor's value.
    taskServo(&pt_taskServo);               // Control head by using servo motor.
    taskMotor(&pt_taskMotor);               // Control movement using motors.
    
    // Visual and audible alerts for any abnormal environment value.
    if (gasValue > 500 || irValue < 100) {
        digitalWrite(BUZZER, HIGH);
        digitalWrite(LED_B, HIGH);
        digitalWrite(LED_G, LOW);
    } else {
        digitalWrite(BUZZER, LOW);
        digitalWrite(LED_B, LOW);
        digitalWrite(LED_G, HIGH);
    }
}
