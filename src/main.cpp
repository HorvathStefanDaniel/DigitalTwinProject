#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

//plate spin values
#define PLATE_SPIN_STOP 1500
#define PLATE_SPIN_MAX 2500
#define PLATE_SPIN_MIN 1550
#define TIME_PER_DEGREE 4.16524 //ms per degree aproximation. Should use sensors for accurate tracking
#define TIME_PER_DEGREE_SLOW 16.61 //ms per degree aproximation when using the slowest speed.

#define UDP_TIMER 10 //timer for udp messages
#define SERVO_SPEED 50// servo speed in ms
#define SEND_SENSOR_TIMER 100 //timer for sending sensor data
#define DEBUG 0     //set to 1 to debug stuff to console

#define DEFAULT_MOVE_SPEED 1    //how many degrees to move, this thing's stopped working as intended, probably the servos went bad. Sometimes when it's at position 90, and the target is 180, trying to write the position + 1 (91) fails to move it.

// Network configuration
constexpr char SSID_NAME[] = "StefanIOT";
constexpr char SSID_PASSWORD[] = "stefaniot";

//Default positions for the joints
int arm_A_pos = 90;   // desired angle for A
int arm_B_pos = 150;  // desired angle for B
int arm_C_pos = 90;    // desired angle for C
int plate_pos = 0;    // desired angle for plate

// Button state management
bool lastButtonState = HIGH;  // Assume button starts unpressed
bool servosAttached = false;   // Assume servos start attached

// Servo pin definitions
#define servo_A 2
#define servo_B 4
#define servo_C 25
#define servo_D 32

#define pot_A 33
#define pot_B 34
#define pot_C 35

//sensor pins
#define trig 26
#define echo 27

// Button pin
#define BUTTON_PIN 0

//servo correction value
#define SERVO_CORRECTION 1

// Supersampling for potentiometer readings
constexpr int POT_SAMPLES = 64;

// Function to read from a servo with compensation. 
int readServoCompensated(Servo& servo) {
    int rawRead = servo.read();  // Get the raw reading
    int compensatedRead = rawRead + SERVO_CORRECTION;  // Add compensation to the read value
    return compensatedRead;  // Return the compensated value
}

// Servo objects
Servo arm_A;        //180 is towards the plate, 0 is away from the plate, this is the head servo
Servo arm_B;        
Servo arm_C;        //180 is towards the plate, 0 is away from the plate
Servo plate;        // takes 2.2 seconds for a full rotation

// Function to attach all servos
void attach_servos() {
    arm_A.attach(servo_A);
    arm_B.attach(servo_B);
    arm_C.attach(servo_C);
    servosAttached = true;
    arm_A.write(arm_A_pos);
    arm_B.write(arm_B_pos);
    arm_C.write(arm_C_pos);
}

// Function to detach all servos
void detach_servos() {
    arm_A.detach();
    arm_B.detach();
    arm_C.detach();
    servosAttached = false;
}

bool servoPosCheck()
{
    int arm_a_temp = readServoCompensated(arm_A);
    int arm_b_temp = readServoCompensated(arm_B);
    int arm_c_temp = readServoCompensated(arm_C);

    //Serial.println("Servo A: " + String(arm_A_pos) + " " + String(arm_a_temp));
    //Serial.println("Servo B: " + String(arm_B_pos) + " " + String(arm_b_temp));
    //Serial.println("Servo C: " + String(arm_C_pos) + " " + String(arm_c_temp));

    if (arm_A_pos != arm_a_temp ) return false;
    if (arm_B_pos != arm_b_temp ) return false;
    if (arm_C_pos != arm_c_temp ) return false;

    return true;
}



// UDP communication
WiFiUDP Udp;
IPAddress RECEIVER_IP_ADDRESS(10, 126, 56, 116);       //192.168.1.93 10.126.56.116
constexpr int RECEIVER_PORT = 50195;
constexpr int LOCAL_PORT = 3002;
String UDPDataString = "";
char UDPPacketBuffer[255];

// Timing for sensor and actuator updates
constexpr int SENSOR_CYCLE_DELAY = 100;
long lastSensorCycle = 0;
constexpr int ACTUATOR_CYCLE_DELAY = 10;
long lastActuatorCycle = 0;
long printMilis = 0;

// Potentiometer input
int potentiometerValue = 0;

bool currentButtonState;

long readSonar() {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH);
    long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and return)
    return distance;    // in cm
}

void button_logic() {
    currentButtonState = digitalRead(BUTTON_PIN);  // Read the current button state
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        // Button was pressed
        if (servosAttached) {
            // Detach servos
            detach_servos();
            Serial.println("Servos detached");
        } else {
            // Attach servos
            attach_servos();
            delay(500);  // Ensure servos have time to move to initial position
        }
    }
    lastButtonState = currentButtonState;  // Update the last button state
}

void set_servos_to_angle(int target_A, int target_B, int target_C) {
    arm_A_pos = target_A;
    arm_B_pos = target_B;
    arm_C_pos = target_C;
}

/*

A:0|B:14|C:8

This is 0 in unity.

Angled towards the plate the values are A:84 B:120 C:90

Angled away from the plate the values are A: -96 B: -44 C: -90

*/

int getAngleA() {
    // Supersampling to reduce noise
    int potValueA = 0;
    for (int i = 0; i < POT_SAMPLES; i++) {
        potValueA += analogRead(pot_A);
    }
    potValueA /= POT_SAMPLES;

    // Map potentiometer values to angles based on custom ranges
    return map(potValueA, 3567, 193, -90, 90); // Reversed mapping
}

int getAngleB() {
    // Supersampling to reduce noise
    int potValueB = 0;
    for (int i = 0; i < POT_SAMPLES; i++) {
        potValueB += analogRead(pot_B);
    }
    potValueB /= POT_SAMPLES;

    // Map potentiometer values to angles based on custom ranges
    return map(potValueB, 455, 4095, -45, 120); // Extended range mapping
}
int getAngleC() {
    // Supersampling to reduce noise
    int potValueC = 0;
    for (int i = 0; i < POT_SAMPLES; i++) {
        potValueC += analogRead(pot_C);
    }
    potValueC /= POT_SAMPLES;

    // Map potentiometer values to angles based on custom ranges
    return map(potValueC, 4095, 0, -90, 90); // Reversed mapping
}

//assuming the plate starts rotating from the start
int getAnglePlate(){
    //long rotation_time = millis() % 2200;
    long rotation_time = millis() % 5980;   //for slow rotation speed
    int angle = map(rotation_time, 0, 5980, 0, 360);
    return angle;
}

// Utility function to extract and round float values from the received UDP string
int getValue(const String& data, char separator) {
    int separatorPos = data.indexOf(separator);
    if (separatorPos == -1) {
        return -1;
    }
    int colonPos = data.indexOf(':', separatorPos);
    int nextSeparatorPos = data.indexOf('|', colonPos);

    float value = data.substring(colonPos + 1, nextSeparatorPos).toFloat();
    int roundedValue = round(value);
    // Cap the value between 10 and 170 to not break the robot arm
    if (roundedValue < 10) {
        roundedValue = 10;
    } else if (roundedValue > 170) {
        roundedValue = 170;
    }


    return roundedValue;
}


// Utility function to extract string values for commands
String getValueStr(const String& data, char separator) {
    int separatorPos = data.indexOf(separator);
    int colonPos = data.indexOf(':', separatorPos);
    int nextSeparatorPos = data.indexOf('|', colonPos);

    return data.substring(colonPos + 1, nextSeparatorPos);
}

// Function to parse servo position commands and apply them
void parseServoCommands(const String& msg) {
    int posA = getValue(msg, 'A');
    int posB = getValue(msg, 'B');
    int posC = getValue(msg, 'C');
    String commandD = getValueStr(msg, 'D');

    set_servos_to_angle(posA, posB, posC);

    if(posA == -1 || posB == -1 || posC == -1){
        //don't write
    }else{
        arm_A.write(posA);
        arm_B.write(posB);
        arm_C.write(posC);
    }

    if (commandD == "start") {
        plate.write(PLATE_SPIN_MIN);
    } else if (commandD == "start_fast") {
        plate.write(PLATE_SPIN_MAX);
    } else if(commandD == "stop"){
        plate.write(PLATE_SPIN_STOP);
    } else{
        //invalid or no command, don't care
    }
}

// Function to receive and parse commands for servos and handle UDP messages
void receiveUDPMessage() {
    if (Udp.parsePacket()) {
        int length = Udp.read(UDPPacketBuffer, 255);
        if (length > 0) {
            UDPPacketBuffer[length] = 0;
            
            if( DEBUG == 1 ){
                Serial.println("Received UDP message: " + String(UDPPacketBuffer));
            }

            // Parse the received message
            String msg = String(UDPPacketBuffer);
            if (msg.startsWith("Servo")) {
                // Expected message format: "Servo|A:90|B:90|C:90|D:stop"
                parseServoCommands(msg);
            }
        }
    }
}

void printServoReadings() {
    Serial.println("Arm_A read: " + String (readServoCompensated(arm_A)) + " Arm_B read: " + String (readServoCompensated(arm_B)) + " Arm_C read: " + String (readServoCompensated(arm_C)));
}
void printServoTargets(){
    Serial.println("Arm_A target: " + String (arm_A_pos) + " Arm_B target: " + String (arm_B_pos) + " Arm_C target: " + String (arm_C_pos));
}

// Function to send sensor data to Unity
void sendSensorData() {
    int angleA = getAngleA();
    int angleB = getAngleB();
    int angleC = getAngleC();
    int angle_plate = getAnglePlate();
    long distance = readSonar();

    String message = "Sensors|";
    message += "A:" + String(angleA) + "|";
    message += "B:" + String(angleB) + "|";
    message += "C:" + String(angleC) + "|";
    message += "Plate:" + String(angle_plate) + "|";
    message += "Dist:" + String(distance);

    Udp.beginPacket(RECEIVER_IP_ADDRESS, RECEIVER_PORT);
    Udp.print(message);
    Udp.endPacket();

    if (DEBUG) {
        Serial.println("Sent sensor data: " + message);
    }
    Serial.println("Sent sensor data: " + message);
}

//increment plate agle by value; isn't very accurate because of spin up time. Won't use as it is now.
void incrementPlateAngle(int value){
    plate_pos = plate_pos % 360;    //keep the value between 0 and 360
    int current_angle = plate_pos;
    plate_pos += value;
    // Check bounds and mechanical limits here if necessary

    float time = abs(plate_pos - current_angle) * TIME_PER_DEGREE_SLOW;

    unsigned long start_time = millis();
    plate.write(PLATE_SPIN_MIN);
    while (millis() - start_time < (unsigned long)round(time)){
    }
    plate.write(PLATE_SPIN_STOP);
}

// Loop function
bool doOnce = false;
long servo_time = millis();
long udp_time = millis();
long send_sensor_timer = millis();

//faster movement
bool moveFastA = false;
bool moveFastB = false;
bool moveFastC = false;

void do_servo_movement(){
    // Servo cycle. This just moves the sero to the target positions at a fixed speed. 
    //Something went wrong with the servos or the wiring, so a compensation for reading and writing is added.
    if ((millis()-servo_time) >= SERVO_SPEED) {

        if (DEBUG == 1){
            printServoReadings();
            printServoTargets();
            Serial.println("Plate angle: " + String(getAnglePlate()));
        }
        servo_time = millis(); // save time reference for next position update

        if (arm_A_pos > readServoCompensated(arm_A)){
            //to ensure that the servo doesn't get stuck
            if(readServoCompensated(arm_A) % 2 == 1){
                moveFastA = false;
            }
            arm_A.write(readServoCompensated(arm_A) + DEFAULT_MOVE_SPEED + (moveFastA ? 2 : 1));
            moveFastA = true;
        }
        else if (arm_A_pos < readServoCompensated(arm_A)){
            //to ensure that the servo doesn't get stuck
            if(readServoCompensated(arm_A) % 2 == 1){
                moveFastA = false;
            }
            arm_A.write(readServoCompensated(arm_A) - DEFAULT_MOVE_SPEED - (moveFastA ? 1 : 0));
            moveFastA = true;
        } else{
            moveFastA = false;
        }

        if (arm_B_pos > readServoCompensated(arm_B)){
            //to ensure that the servo doesn't get stuck
            if(readServoCompensated(arm_B) % 2 == 1){
                moveFastB = false;
            }
            arm_B.write(readServoCompensated(arm_B) + DEFAULT_MOVE_SPEED + (moveFastB ? 2 : 1));
            moveFastB = true;
        }
        else if (arm_B_pos < readServoCompensated(arm_B)){
            //to ensure that the servo doesn't get stuck
            if(readServoCompensated(arm_B) % 2 == 1){
                moveFastB = false;
            }
            arm_B.write(readServoCompensated(arm_B) - DEFAULT_MOVE_SPEED - (moveFastB ? 1 : 0));
            moveFastB = true;
        } else{
            moveFastB = false;
        }

        if (arm_C_pos > readServoCompensated(arm_C)){
            //to ensure that the servo doesn't get stuck
            if(readServoCompensated(arm_C) % 2 == 1){
                moveFastC = false;
            }
            arm_C.write(readServoCompensated(arm_C) + DEFAULT_MOVE_SPEED + (moveFastC ? 2 : 1));
            moveFastC = true;
        }
        else if (arm_C_pos < readServoCompensated(arm_C)){
            //to ensure that the servo doesn't get stuck
            if(readServoCompensated(arm_C) % 2 == 1){
                moveFastC = false;
            }
            arm_C.write(readServoCompensated(arm_C) - DEFAULT_MOVE_SPEED - (moveFastC ? 1 : 0));
            moveFastC = true;
        } else{
            moveFastC = false;
        }
  }
}


// Setup function
void setup() {
    Serial.begin(9600);
    WiFi.begin(SSID_NAME, SSID_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(SSID_NAME);
        delay(1000);
    }

    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Udp.begin(LOCAL_PORT);
    Serial.println("UDP Started");

    // Set up the potentiometer pins
    pinMode(pot_A, INPUT);
    pinMode(pot_B, INPUT);
    pinMode(pot_C, INPUT);

    //echolocator
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Attach the servos with a suitable pulse width range
    attach_servos();
    // Plate servo separate because it should always spin
    plate.attach(servo_D);
    plate.write(PLATE_SPIN_MIN);

    // Set up the button pin
    currentButtonState = digitalRead(BUTTON_PIN);

    delay(500); // Ensure servos have time to move to initial position
}



void loop() {
    // Button logic
    button_logic();
    //incrementPlateAngle(90);
    // Receive UDP messages
    if(millis() - udp_time >= UDP_TIMER){
        receiveUDPMessage();
        udp_time = millis();
    }

    if(millis() - send_sensor_timer >= SEND_SENSOR_TIMER){
        send_sensor_timer = millis();
        sendSensorData();
    }
    //do_servo_movement();
}
