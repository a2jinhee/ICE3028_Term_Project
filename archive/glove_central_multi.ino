/*
  glove_central_multi.ino
*/

#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>

// Unique UUID code for service
const char *service_uuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
// Unique UUID code for characteristic
const char *char_gesture_uuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char *char_devicenum_uuid = "19b10002-e8f2-537e-4f6c-d104768a1214";

int gesture = -1;
int oldGestureValue = -1;
char check_devicenum = false;

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
  BLE.setLocalName("magic_glove");
  BLE.advertise();

  Serial.println("magic_glove");
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
    BLE.scanForUuid(service_uuid);
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
  BLECharacteristic gestureChar = peripheral.characteristic(char_gesture_uuid);
  BLECharacteristic devicenumChar = peripheral.characteristic(char_devicenum_uuid);

  // Check if peripheral device initialized/writable characteristic
  if (!gestureChar || !devicenumChar)
  {
    Serial.println("* Peripheral device does not have gesture_type characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!gestureChar.canWrite() || !devicenumChar.canWrite())
  {
    Serial.println("* Peripheral does not have a writable gesture_type characteristic!");
    peripheral.disconnect();
    return;
  }

  /* TFLite model inference part */
  gesture = gestureDetection();

  /* Check if the connected peripheral is the intended one */
  // assuming devienumChar is int and gesture is an array
  check_devicenum = checkPeripheral(devicenumChar.readValue(), gesture);
  if (!check_devicenum){
    Serial.println("* This peripheral is not the intended target. Disconnecting...");
    peripheral.disconnect();
    return; 
  }

  // Write value to gesture_type characteristic
  while (peripheral.connected())
  {
    // Check if gesture changed from before
    if (oldGestureValue != gesture)
    {
      oldGestureValue = gesture;
      Serial.print("* Writing value to gesture_type characteristic: ");
      Serial.println(gesture);
      gestureChar.writeValue((byte)gesture);
      Serial.println("* Writing value to gesture_type characteristic done!");
      Serial.println(" ");
    }
  }
  Serial.println("- Peripheral device disconnected!");
}

/* Check if the connected peripheral is the intended one */
bool checkPeripheral(int devicenum, *gesture)
{ 
  // assuming gesture[0] holds which peripheral device is the target
  // and gesture[1] holds the actual gesture info
  if (gesture[0]!=devicenum){
    return false;
  }
  return true; 
}

/* TFLite model inference part */
int gestureDetection()
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