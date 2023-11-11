#include <ArduinoBLE.h>

BLEService doorService("19b1"); // uuid 변경하기// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19b1", BLERead | BLEWrite); //uuid 변경하기const int doorPin = LED_BUILTIN; // door 관련된 핀으로 변경하기
int flag = 0;
int flag2 = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // set LED pin to output mode
  pinMode(doorPin, OUTPUT);  //door pin output이겠지?  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");    while (1);
  }  // set advertised local name and service UUID:
  BLE.setLocalName("Door");  
  BLE.setAdvertisedService(doorService);  // add the characteristic to the service
  doorService.addCharacteristic(switchCharacteristic);  // add service
  BLE.addService(doorService);  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);  // start advertising
  BLE.advertise();  Serial.println("BLE Door Peripheral");
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (flag == 0){
        if (flag2 == 0){
          Serial.println("Door open");
          flag2++;
        }
      }      //dooropen();                       // door open 함수 만들어야함.
      if (flag == 1){
        if (flag2 == 0){
          Serial.println("Door close");
          flag2++;
        }
      }
      //doorclose();                      // door close 함수 만들어야함.
    }    // when the central disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    flag2 = 0;
    flag++;
    if (flag >= 2){
      flag = 0;
    }
  }
}