// https://www.hackster.io/gov/imu-to-you-ae53e1?ref=channel&ref_id=12470_trending___&offset=5

#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

// Custom BLE IMU Service
BLEService imuService("75600916-874C-4ADB-A3A1-DB5B71487453");

// Custom BLE IMU Characteristics
BLEFloatCharacteristic characteristicAccelX("E6B56A81-BA9C-4781-B2FF-A79329B81F40", BLERead | BLENotify);
BLEFloatCharacteristic characteristicAccelY("6D5C2469-6A3A-4811-A900-822B8CD48BF4", BLERead | BLENotify);
BLEFloatCharacteristic characteristicAccelZ("A3F66B9D-E156-48C7-A9B6-F37E0D21F3FE", BLERead | BLENotify);

float accelX, accelY, accelZ;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Nano33IMU");
  BLE.setAdvertisedService(imuService); // add the service UUID
  imuService.addCharacteristic(characteristicAccelX); 
  imuService.addCharacteristic(characteristicAccelY); 
  imuService.addCharacteristic(characteristicAccelZ);
  
  BLE.addService(imuService); // Add the battery service
  characteristicAccelX.writeValue(0.0);
  characteristicAccelY.writeValue(0.0);
  characteristicAccelZ.writeValue(0.0);
   
  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // listen for BLE peripherals to connect
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());

    while (central.connected()) {
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accelX, accelY, accelZ);
        Serial.print(accelX);
        Serial.print('\t');
        Serial.print(accelY);
        Serial.print('\t');
        Serial.println(accelZ);

        characteristicAccelX.writeValue(accelX);
        characteristicAccelY.writeValue(accelY);
        characteristicAccelZ.writeValue(accelZ);
      }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
