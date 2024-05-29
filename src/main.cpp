#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define UDP_TIMER 100 //timer for udp messages
#define SERVO_SPEED 100// servo speed in ms
#define SEND_SENSOR_TIMER 100 //timer for sending sensor data
#define DEBUG 0     //set to 1 to debug stuff to console

// Network configuration
constexpr char SSID_NAME[] = "StefanIOT";
constexpr char SSID_PASSWORD[] = "stefaniot";

//Default positions for the joints
int arm_A_pos = 90;   // desired angle for front left leg
int arm_B_pos = 90;  // desired angle for front right leg
int arm_C_pos = 90;    // desired angle for rear left leg

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
Servo arm_A;        //180 is towards the plate, 0 is away from the plate
Servo arm_B;        
Servo arm_C;        //180 is towards the plate, 0 is away from the plate
Servo plate;        // takes 2.2 seconds for a full rotation

// Function to attach all servos
void attach_servos() {
    arm_A.attach(servo_A);
    arm_B.attach(servo_B);
    arm_C.attach(servo_C);
    plate.attach(servo_D);
    servosAttached = true;
}

// Function to detach all servos
void detach_servos() {
    arm_A.detach();
    arm_B.detach();
    arm_C.detach();
    plate.detach();
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
IPAddress RECEIVER_IP_ADDRESS(192, 168, 87, 175);
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

// Button state management
bool lastButtonState = HIGH;  // Assume button starts unpressed
bool servosAttached = false;   // Assume servos start attached

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
            //bring it back to default positions
            plate.write(1500);
        }
    }
    lastButtonState = currentButtonState;  // Update the last button state
}

void set_servos_to_angle(int target_A, int target_B, int target_C) {
    arm_A_pos = target_A;
    arm_B_pos = target_B;
    arm_C_pos = target_C;
}

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

// Utility function to extract values from the received UDP string
int getValue(const String& data, char separator) {
    int separatorPos = data.indexOf(separator);
    int colonPos = data.indexOf(':', separatorPos);
    int nextSeparatorPos = data.indexOf('|', colonPos);

    return data.substring(colonPos + 1, nextSeparatorPos).toInt();
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

    arm_A_pos = posA;
    arm_B_pos = posB;
    arm_C_pos = posC;

    arm_A.write(posA);
    arm_B.write(posB);
    arm_C.write(posC);

    if (commandD == "left") {
        plate.write(1600); // Assuming 1600 is the left command position
    } else if (commandD == "right") {
        plate.write(1400); // Assuming 1400 is the right command position
    } else {
        plate.write(1500); // Stop position
    }
}

// Function to receive and parse commands for servos and handle UDP messages
void receiveUDPMessage() {
    if (Udp.parsePacket()) {
        int length = Udp.read(UDPPacketBuffer, 255);
        if (length > 0) {
            UDPPacketBuffer[length] = 0;
            Serial.println("Received UDP message: " + String(UDPPacketBuffer));

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
    long distance = readSonar();

    String message = "Sensors|";
    message += "A:" + String(angleA) + "|";
    message += "B:" + String(angleB) + "|";
    message += "C:" + String(angleC) + "|";
    message += "Dist:" + String(distance);

    Udp.beginPacket(RECEIVER_IP_ADDRESS, RECEIVER_PORT);
    Udp.print(message);
    Udp.endPacket();

    if (DEBUG) {
        Serial.println("Sent sensor data: " + message);
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

    // Initialize servo positions
    Serial.println("Setting initial servo positions");
    arm_A.write(90);
    arm_B.write(90);
    arm_C.write(90);
    plate.write(1500);

    // Set up the button pin
    currentButtonState = digitalRead(BUTTON_PIN);

    delay(500); // Ensure servos have time to move to initial position
}

// Loop function
bool doOnce = false;
long servo_time = millis();
long udp_time = millis();
long send_sensor_timer = millis();

void loop() {
    // Button logic
    button_logic();
    
    // Receive UDP messages
    if(millis() - udp_time >= UDP_TIMER){
        receiveUDPMessage();
        udp_time = millis();
    }

    if(millis() - send_sensor_timer >= SEND_SENSOR_TIMER){
        printServoReadings();
        sendSensorData();
        send_sensor_timer = millis();
    }
    

    // Servo cycle. This just moves the sero to the target positions at a fixed speed. 
    //Something went wrong with the servos or the wiring, so a compensation for reading and writing is added.
    if ((millis()-servo_time) >= SERVO_SPEED) {
        if (DEBUG == 1){
            printServoReadings();
            printServoTargets();
        }
        servo_time = millis(); // save time reference for next position update

        if (arm_A_pos > readServoCompensated(arm_A)){
            arm_A.write(readServoCompensated(arm_A) + 1);
        }
        else if (arm_A_pos < readServoCompensated(arm_A)){
            arm_A.write(readServoCompensated(arm_A) - 1);
        } 

        if (arm_B_pos > readServoCompensated(arm_B)) arm_B.write(readServoCompensated(arm_B) + 1);
        else if (arm_B_pos < readServoCompensated(arm_B)) arm_B.write(readServoCompensated(arm_B) - 2);

        if (arm_C_pos > readServoCompensated(arm_C)) arm_C.write(readServoCompensated(arm_C) + 1);
        else if (arm_C_pos < readServoCompensated(arm_C)) arm_C.write(readServoCompensated(arm_C) - 2);
  }
}
