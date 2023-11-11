#include <ArduinoBLE.h>
#include <Servo.h>

BLEService myService("19b3"); 
BLEByteCharacteristic myChar("19b3", BLERead | BLEWrite); 

// Pin number for the LED
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  while (!Serial);  

  // HARDWARE SETUP - CHANGE
  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT); 

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");    
    while (1);
  } 

  BLE.setLocalName("LED");  
  BLE.setAdvertisedService(myService);  // add the characteristic to the service
  myService.addCharacteristic(myChar);  // add service
  BLE.addService(myService);  // set the initial value for the characeristic:

  myChar.writeValue(0);
  BLE.advertise();  
  Serial.println("BLE LED Peripheral");
}

void loop() {
  
  BLEDevice central = BLE.central();  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // Read the current state of the LED
    int ledState = digitalRead(ledPin);
    Serial.print("LED State: ");
    Serial.println(ledState);

    while (central.connected()) {
      
      // Turn on/off led with current state - CHANGE
      if (ledState){
        digitalWrite(ledPin, LOW);
        delay(1000); 
      }
      else {
        digitalWrite(ledPin, HIGH);
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
  }
}