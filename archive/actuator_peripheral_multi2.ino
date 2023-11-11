/*
  BLE_Peripheral.ino

  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE
  as a peripheral device and specifies a service and a characteristic. Depending
  of the value of the specified characteristic, an on-board LED gets on.

  The circuit:
  - Arduino Nano 33 BLE.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

enum
{
  GESTURE_NONE = -1,
  GESTURE_UP = 0,
  GESTURE_DOWN = 1,
  GESTURE_LEFT = 2,
  GESTURE_RIGHT = 3
};

// Unique UUID code for service
const char *service_uuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
// Unique UUID code for characteristic
const char *char_gesture_uuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char *char_devicenum_uuid = "19b10002-e8f2-537e-4f6c-d104768a1214";

int gesture = -1;
int devicenum = 2;

// Initialization of BLE service and characteristic for peripheral code (변수 선언)
BLEService gestureService(service_uuid);
BLEByteCharacteristic gestureChar(char_gesture_uuid, BLERead | BLEWrite);
BLEByteCharacteristic devicenumChar(char_devicenum_uuid, BLERead | BLEWrite);

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  // Initialize LEDs and set their initial states
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize the BLE module
  if (!BLE.begin())
  {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  /*
  // Set the name of the peripheral device,
  // and define the services and characteristics
  */
  BLE.setLocalName("window");
  // the service that the peripheral will advertise
  // used to specify which service you want to actively advertise to other devices.
  BLE.setAdvertisedService(gestureService);
  // adds a characteristic to 'gestureService'
  gestureService.addCharacteristic(gestureChar);
  gestureService.addCharacteristic(devicenumChar);
  // adds 'gestureService' to the list of services provided by the peripheral, necessary to expose the service to central devices
  // registers the service with the BLE stack, making it available for central devices to discover when they connect to the peripheral.
  BLE.addService(gestureService);
  // initial value of 'gestureCharacteristic' set to -1
  gestureChar.writeValue(-1);
  devicenumChar.writeValue(devicenum);
  // This function call starts the advertising process
  BLE.advertise();

  Serial.println("window");
  Serial.println(" ");
}

void loop()
{
  // Checks for a central device's connection
  BLEDevice central = BLE.central();
  Serial.println("- Discovering central device...");
  delay(500);

  if (central)
  {
    Serial.println("* Connected to central device!");
    Serial.print("* Device MAC address: ");
    Serial.println(central.address());
    Serial.println(" ");

    while (central.connected())
    {
      // Check if characteristic was written to by central
      if (gestureChar.written())
      {
        // Update the LEDs based on the received gesture value
        gesture = gestureChar.value();
        writeGesture(gesture);
      }
    }
    Serial.println("* Disconnected to central device!");
  }
}

// Handle changes in the characteristic's value and update LEDs
void writeGesture(int gesture)
{
  Serial.println("- Characteristic <gesture_type> has changed!");

  switch (gesture)
  {
  case GESTURE_UP:
    Serial.println("* Actual value: UP (red LED on)");
    Serial.println(" ");
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    break;
  case GESTURE_DOWN:
    Serial.println("* Actual value: DOWN (green LED on)");
    Serial.println(" ");
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    break;
  case GESTURE_LEFT:
    Serial.println("* Actual value: LEFT (blue LED on)");
    Serial.println(" ");
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    break;
  case GESTURE_RIGHT:
    Serial.println("* Actual value: RIGHT (built-in LED on)");
    Serial.println(" ");
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    break;
  default:
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    break;
  }
}