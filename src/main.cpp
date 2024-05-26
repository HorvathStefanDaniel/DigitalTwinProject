#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define SERVO_SPEED 10// servo speed in ms

// Network configuration
constexpr char SSID_NAME[] = "StefanIOT";
constexpr char SSID_PASSWORD[] = "stefaniot";

//Default positions for the joints
int arm_A_pos = 0;   // desired angle for front left leg
int arm_B_pos = 0;  // desired angle for front right leg
int arm_C_pos = 0;    // desired angle for rear left leg

// Servo pin definitions
#define servo_A 2
#define servo_B 4
#define servo_C 25
#define servo_D 32

// Servo objects
Servo arm_A;        //180 is towards the plate, 0 is away from the plate
Servo arm_B;        
Servo arm_C;        //180 is towards the plate, 0 is away from the plate
Servo plate;

bool servoPosCheck()
{
    if (arm_A_pos != arm_A.read()) return false;
    if (arm_B_pos != arm_B.read()) return false;
    if (arm_C_pos != arm_C.read()) return false;
    return true;
}

// top, middle, base
void setServoPos(int armAPos, int armBPos, int armCPos)
{
    arm_A_pos = armAPos;
    arm_B_pos = armBPos;
    arm_C_pos = armCPos;

    //for testing
    arm_A.write(arm_A_pos);
    arm_B.write(arm_B_pos);
    arm_C.write(arm_C_pos);
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
constexpr int potentiometerPin = 33;
int potentiometerValue = 0;

// LED output
constexpr int LEDPin = 23;
int LEDValue = 0;

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

    pinMode(potentiometerPin, INPUT);
    pinMode(LEDPin, OUTPUT);

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
    arm_A.write(90);
    arm_B.write(90);
    arm_C.write(90);
    plate.write(1500);  //500 clockwise, 2500 counter-clockwise, 1500 stop

    delay(5000); // Ensure servos have time to move to initial position
}

int test_pos = 1;


static unsigned long servo_time = millis();
void loop() {
    arm_A.write(90);
    delay(1000);
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
