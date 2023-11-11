/*
  BLE_Central_Device.ino
  https://docs.arduino.cc/tutorials/nano-33-ble-sense/ble-device-to-device

  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE Sense
  as a central device and looks for a specified service and characteristic in a
  peripheral device. If the specified service and characteristic is found in a
  peripheral device, the last detected value of the on-board gesture sensor of
  the Nano 33 BLE Sense, the APDS9960, is written in the specified characteristic.

  The circuit:
  - Arduino Nano 33 BLE Sense.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>

// Unique UUID code for service
const char *deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
// Unique UUID code for characteristic
const char *deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

int gesture = -1;
int oldGestureValue = -1;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  // Initialize the APDS9960 sensor for gesture recognition
  if (!APDS.begin())
  {
    Serial.println("* Error initializing APDS9960 sensor!");
  }

  APDS.setGestureSensitivity(80);

  // Initialize BLE
  if (!BLE.begin())
  {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  /*
  // Set the name of the central device,
  // and start advertising to make itself
  // discoverable to other peripheral devices
  */
  BLE.setLocalName("Nano 33 BLE (Central)");
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE Sense (Central Device)");
  Serial.println(" ");
}

void loop()
{
  connectToPeripheral();
}

void connectToPeripheral()
{
  BLEDevice peripheral;

  Serial.println("- Discovering peripheral device...");

  // Scan for a peripheral with a specific service UUID
  // find available device and store in 'peripheral'
  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    peripheral = BLE.available();
  } while (!peripheral);

  // Peripheral Info
  if (peripheral)
  {
    Serial.println("* Peripheral device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.print("* Device name: ");
    Serial.println(peripheral.localName());
    Serial.print("* Advertised service UUID: ");
    // This should be same as central service uuid
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan(); // stop scan when found

    // Control the peripheral once found
    controlPeripheral(peripheral);
  }
}

void controlPeripheral(BLEDevice peripheral)
{
  // Check if peripheral is connected
  Serial.println("- Connecting to peripheral device...");
  if (peripheral.connect())
  {
    Serial.println("* Connected to peripheral device!");
    Serial.println(" ");
  }
  else
  {
    Serial.println("* Connection to peripheral device failed!");
    Serial.println(" ");
    return;
  }

  // Check for peripheral's services and characteristics.
  Serial.println("- Discovering peripheral device attributes...");
  if (peripheral.discoverAttributes())
  {
    Serial.println("* Peripheral device attributes discovered!");
    Serial.println(" ");
  }
  else
  {
    Serial.println("* Peripheral device attributes discovery failed!");
    Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  // Store peripheral's characteristic container to central's characteristic
  BLECharacteristic gestureCharacteristic = peripheral.characteristic(deviceServiceCharacteristicUuid);

  // Check if peripheral device initialized/writable characteristic
  if (!gestureCharacteristic)
  {
    Serial.println("* Peripheral device does not have gesture_type characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!gestureCharacteristic.canWrite())
  {
    Serial.println("* Peripheral does not have a writable gesture_type characteristic!");
    peripheral.disconnect();
    return;
  }

  // Write value to gesture_type characteristic
  while (peripheral.connected())
  {
    gesture = gestureDetectection();
    // Check if gesture changed from before
    if (oldGestureValue != gesture)
    {
      oldGestureValue = gesture;
      Serial.print("* Writing value to gesture_type characteristic: ");
      Serial.println(gesture);
      gestureCharacteristic.writeValue((byte)gesture);
      Serial.println("* Writing value to gesture_type characteristic done!");
      Serial.println(" ");
    }
  }
  Serial.println("- Peripheral device disconnected!");
}

// Gesture detection function
int gestureDetectection()
{
  if (APDS.gestureAvailable())
  {
    gesture = APDS.readGesture();

    switch (gesture)
    {
    case GESTURE_UP:
      Serial.println("- UP gesture detected");
      break;
    case GESTURE_DOWN:
      Serial.println("- DOWN gesture detected");
      break;
    case GESTURE_LEFT:
      Serial.println("- LEFT gesture detected");
      break;
    case GESTURE_RIGHT:
      Serial.println("- RIGHT gesture detected");
      break;
    default:
      Serial.println("- No gesture detected");
      break;
    }
  }
  return gesture;
}