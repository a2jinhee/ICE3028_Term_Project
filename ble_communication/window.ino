#include <ArduinoBLE.h>
#include <Servo.h>

BLEService windowService("19b2"); 
BLEByteCharacteristic windowChar("19b2", BLERead | BLEWrite); 

Servo myServo;
// Pin number for the LED
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  while (!Serial);  

  // HARDWARE SETUP - CHANGE
  myServo.attach(9);
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
}

void loop() {
  
  BLEDevice central = BLE.central();  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // Read the current position of the servo
    int servoPosition = myServo.read();
    Serial.print("Current Servo Position: ");
    Serial.println(servoPosition);

    while (central.connected()) {
      
      // Open/Close window with current servo position - CHANGE
      // if less than 90, assume open and close
      // if bigger than 90, assume close and open
      if (servoPosition<90){
        myServo.write(0);
        delay(1000); 
      }
      else if (servoPosition>90){
        myServo.write(180);
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