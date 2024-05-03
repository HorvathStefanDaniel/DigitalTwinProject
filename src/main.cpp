
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/* Function Declarations*/
void sendUDPDataString();
void receiveUDPMessage();

/* WiFi */
//Network name (SSID) and password (WPA)
constexpr char SSID_NAME[] = "StefanIOT";
constexpr char SSID_PASSWORD[] = "stefaniot";

/* UDP */
//Udp object
WiFiUDP Udp;

//Receiver IP-address and port
//copy here what's written in unity console when first running the unity project
IPAddress RECEIVER_IP_ADDRESS (192, 168, 87, 175); 
constexpr int RECEIVER_PORT = 50195;
constexpr int LOCAL_PORT = 3002;

//Data string used to send UDP messages
String UDPDataString = "";

//Char array used to receive UDP messages (assuming max packet size is 255 bytes)
char UDPPacketBuffer[255];

/* UPDATE Cycles*/
//Delay interval between each sensor cycle
constexpr int SENSOR_CYCLE_DELAY = 100;
long lastSensorCycle = 0;


//Delay interval between each sensor cycle
constexpr int ACTUATOR_CYCLE_DELAY = 100;
long lastActuatorCycle = 0;

/* SENSORS */
constexpr int potentiometerPin = 33;
int potentiometerValue = 0;
                
/* ACTUATORS */
constexpr int LEDPin = 23;
int LEDValue = 0;
            
void setup() {
    
    /* SERIAL */
    Serial.begin(9600);
    while (!Serial);

    /* WiFi */
    //Begin WiFi
    WiFi.begin(SSID_NAME, SSID_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(SSID_NAME);
			
			delay(1000);
    }
    Serial.println("Connected to WiFi");

    //print this device's IP address to be used in unity
    Serial.print("IP Address: " + WiFi.localIP().toString() + "\n");

    /* UDP */
    //Begin UDP
    Udp.begin(LOCAL_PORT);
    Serial.println("UDP Begun");

    /* SENSORS */
    pinMode(potentiometerPin, INPUT);

    /* ACTUATORS */
    pinMode(LEDPin, OUTPUT);
}

void loop() {
    /* SEND UDP */
    if((millis() - lastSensorCycle) > SENSOR_CYCLE_DELAY){ 
	    lastSensorCycle = millis();

      //Send potentiometer state
      potentiometerValue = analogRead(potentiometerPin);
      UDPDataString = "potentiometer|" + String(potentiometerValue);
      sendUDPDataString();
  }

    /* RECEIVE UDP */
    if((millis() - lastActuatorCycle) > ACTUATOR_CYCLE_DELAY){ 
	    lastActuatorCycle = millis();

      //Receive LED state
      receiveUDPMessage();
    }
}

/* UDP */
//Send current UDPDataString to Unity
void sendUDPDataString() {
  Udp.beginPacket(RECEIVER_IP_ADDRESS, RECEIVER_PORT);
  Udp.print(UDPDataString); 
  Udp.endPacket();
  Serial.print("Send UDP message: ");
  Serial.println(UDPDataString);
}

//Receive UDP DataString from Unity
void receiveUDPMessage() {
  if (Udp.parsePacket()) {
    int length = Udp.read(UDPPacketBuffer, 255);
    if (length > 0) {
      UDPPacketBuffer[length] = 0;
      Serial.print("Received UDP message: ");
      Serial.println(UDPPacketBuffer);
    }

    // Parse the message
    char* part;
    char actuatorID[255];
    int value;

    // Get the actuator ID
    part = strtok(UDPPacketBuffer, "|");
    if (part != NULL) {
      strcpy(actuatorID, part);
    }

    // Get the actuator value
    part = strtok(NULL, "|");
    if (part != NULL) {
      value = atoi(part); // Convert string to integer
    }

    // Set the LED state
    if(strcmp(actuatorID, "LED") == 0) {
        LEDValue = value;
        digitalWrite(LEDPin, LEDValue);
    }
  }
}