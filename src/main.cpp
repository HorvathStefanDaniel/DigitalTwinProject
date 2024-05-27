#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define SERVO_SPEED 100// servo speed in ms
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
#define SERVO_CORRECTION 2


// Function to read from a servo with compensation
int readServoCompensated(Servo& servo) {
    int rawRead = servo.read();  // Get the raw reading
    int compensatedRead = rawRead + SERVO_CORRECTION;  // Add compensation to the read value
    return compensatedRead;  // Return the compensated value
}

// Servo objects
Servo arm_A;        //180 is towards the plate, 0 is away from the plate
Servo arm_B;        
Servo arm_C;        //180 is towards the plate, 0 is away from the plate
Servo plate;

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
bool servosAttached = true;   // Assume servos start attached

void button_logic() {
    currentButtonState = digitalRead(BUTTON_PIN);  // Read the current button state
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        // Button was pressed
        if (servosAttached) {
            // Detach servos
            arm_A.detach();
            arm_B.detach();
            arm_C.detach();
            servosAttached = false;
            Serial.println("Servos detached");
        } else {
            // Attach servos
            arm_A.attach(servo_A);
            arm_B.attach(servo_B);
            arm_C.attach(servo_C);
            servosAttached = true;
            Serial.println("Servos attached");
            //bring it back to default positions
            arm_A.write(90);
            arm_B.write(90);
            arm_C.write(90);
            plate.write(1500);
        }
        delay(1000);  // Debounce
    }
    lastButtonState = currentButtonState;  // Update the last button state
}

void set_servos_to_angle(int target_A, int target_B, int target_C) {
    arm_A_pos = target_A;
    arm_B_pos = target_B;
    arm_C_pos = target_C;
}

int getAngleA() {
    int potValueA = analogRead(pot_A);
    // Map potentiometer values to angles based on custom ranges
    return map(potValueA, 3567, 193, -90, 90); // Reversed mapping
}
int getAngleB() {
    int potValueB = analogRead(pot_B);
    // Map potentiometer values to angles based on custom ranges
    return map(potValueB, 455, 4095, -45, 120); // Extended range mapping
}
int getAngleC() {
    int potValueC = analogRead(pot_C);
    // Map potentiometer values to angles based on custom ranges
    return map(potValueC, 4095, 0, -90, 90); // Reversed mapping
}


// UDP functions (you may activate these if you need to handle incoming UDP data)
void sendUDPDataString() {
    Udp.beginPacket(RECEIVER_IP_ADDRESS, RECEIVER_PORT);
    Udp.print(UDPDataString);
    Udp.endPacket();
    Serial.print("Sent UDP message: ");
    Serial.println(UDPDataString);
}

void receiveUDPMessage() {
    if (Udp.parsePacket()) {
        int length = Udp.read(UDPPacketBuffer, 255);
        if (length > 0) {
            UDPPacketBuffer[length] = 0;
            Serial.print("Received UDP message: ");
            Serial.println(UDPPacketBuffer);
            // Additional message handling here
        }
    }
}

void printServoReadings() {
    Serial.println("Arm_A read: " + String (readServoCompensated(arm_A)) + " Arm_B read: " + String (readServoCompensated(arm_B)) + " Arm_C read: " + String (readServoCompensated(arm_C)));
}
void printServoTargets(){
    Serial.println("Arm_A target: " + String (arm_A_pos) + " Arm_B target: " + String (arm_B_pos) + " Arm_C target: " + String (arm_C_pos));
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
    arm_A.attach(servo_A, 500, 2500); 
    arm_B.attach(servo_B, 500, 2500); 
    arm_C.attach(servo_C, 500, 2500);  
    plate.attach(servo_D, 500, 2500);                  //this just spins

    // Initialize servo positions
    Serial.println("Setting initial servo positions");
    arm_A.write(0);
    arm_B.write(90);
    arm_C.write(90);
    plate.write(1500);  //500 clockwise, 2500 counter-clockwise, 1500 stop

    // Set up the button pin
    currentButtonState = digitalRead(BUTTON_PIN);

    delay(500); // Ensure servos have time to move to initial position
}

// Loop function
bool doOnce = false;
long servo_time = millis();
void loop() {
    // Button logic
    button_logic();

    // UDP message handling
    receiveUDPMessage();

    // Servo cycle
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
            arm_A.write(readServoCompensated(arm_A) - 2);
        } 

        if (arm_B_pos > readServoCompensated(arm_B)) arm_B.write(readServoCompensated(arm_B) + 1);
        else if (arm_B_pos < readServoCompensated(arm_B)) arm_B.write(readServoCompensated(arm_B) - 2);

        if (arm_C_pos > readServoCompensated(arm_C)) arm_C.write(readServoCompensated(arm_C) + 1);
        else if (arm_C_pos < readServoCompensated(arm_C)) arm_C.write(readServoCompensated(arm_C) - 2);
  }
}

