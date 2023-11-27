#include <ArduinoBLE.h>

BLEService windowService("19b2"); 
BLEByteCharacteristic windowChar("19b2", BLERead | BLEWrite); 

// pin number for the LED
const int ledPin = 13;
// pin number for dc motor
int motorPin1 = 13;
int motorPin2 = 12;
int flag = 0;  

void setup() {
  Serial.begin(9600);
  while (!Serial);  

  // Set the LED pin as an output - DEBUG
  // pinMode(ledPin, OUTPUT); 

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");    
    while (1);
  } 

  BLE.setLocalName("Window");  
  BLE.setAdvertisedService(windowService);  // add the characteristic to the service
  windowService.addCharacteristic(windowChar);  // add service
  BLE.addService(windowService);  // set the initial value for the characeristic:

  windowChar.writeValue(0);
  BLE.advertise();  
  Serial.println("BLE Window Peripheral");

  // dc motor setup
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

}

void loop() {
  
  BLEDevice central = BLE.central();  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    while (central.connected()) {
      
      // Open/Close window with current servo position
      if (flag%2 == 0){
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
        // time = length of string
        delay(7000);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
        delay(1000);
      }
      else{
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
        // time = length of string
        delay(7000);
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
        delay(1000);
      }
      
      // DEBUG 
      // // Turn the LED on
      // digitalWrite(ledPin, HIGH);
      // delay(1000);  // Wait for 1 second

      // // Turn the LED off
      // digitalWrite(ledPin, LOW);
      // delay(1000);  // Wait for 1 second
    } 

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    flag += 1; 
  }
}