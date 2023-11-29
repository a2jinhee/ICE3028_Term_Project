#include <ArduinoBLE.h>
#include <ESP32_Servo.h>

BLEService doorService("19b1"); 
BLEByteCharacteristic doorChar("19b1", BLERead | BLEWrite); 

// Pin number for the LED
int motorPin = 16;
int motorPin2 = 17;

// int motorPin_2 = 14;
// int motorPin2_2 = 27;
int flag = 0; // 0: close 1: open
static const int servoPin = 27;

Servo servo1;

void setup() {

  Serial.begin(115200);
  while (!Serial);  

  // HARDWARE SETUP - CHANGE
  // Set the LED pin as an output - DEBUG
  // pinMode(ledPin, OUTPUT); 

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");    
    while (1);
  } 

  servo1.attach(servoPin); // D27에 서보모터가 연결되었습니다.
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  // pinMode(motorPin_2, OUTPUT);
  // pinMode(motorPin2_2, OUTPUT);

  BLE.setLocalName("Door");  
  BLE.setAdvertisedService(doorService);  // add the characteristic to the service
  doorService.addCharacteristic(doorChar);  // add service
  BLE.addService(doorService);  // set the initial value for the characeristic:

  doorChar.writeValue(0);
  BLE.advertise();  
  Serial.println("BLE Door Peripheral");
}

void loop() {
  
  BLEDevice central = BLE.central();  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    int i = servo1.read();
    if (i==0){
      flag = 0;
    }
    else{
      flag = 1;
    }

    // Read the current position of the servo

    while (central.connected()) {
      
      // Open/Close door with current servo position - CHANGE
      // if less than 90, assume open and close
      // if bigger than 90, assume close and open

      if(!flag){  //door close
        servo1.write(180);
        delay(1000);

        digitalWrite(motorPin, LOW);
        digitalWrite(motorPin2, HIGH);
        delay(1000);
        digitalWrite(motorPin, LOW);
        digitalWrite(motorPin2, LOW);
        flag=1;

      }

      else{

        //  for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
        //   servo1.write(posDegrees); // 모터의 각도를 설정합니다.
        //   Serial.println(posDegrees);
        //   delay(20);
        // }
        digitalWrite(motorPin, HIGH);
        digitalWrite(motorPin2, LOW);
        delay(1000);
        digitalWrite(motorPin, LOW);
        digitalWrite(motorPin2, LOW);

        servo1.write(0);
        delay(1000);
        flag=0;

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