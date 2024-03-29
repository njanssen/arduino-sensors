#define DEBUG

#include <WiFi101.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include <OSCMessage.h>
#include "credentials.h"
#include "calibration.h"

#define OSC_PORT 9000
IPAddress osc_ip(255,255,255,255);
WiFiUDP udp;
int status;

OSCBundle oscBundle;
OSCMessage oscQuaternion("/ic/imu/quaternion");
OSCMessage oscEuler("/ic/imu/euler");
OSCMessage oscAccel("/ic/imu/accel");
OSCMessage oscLinearAccel("/ic/imu/linearaccel");
OSCMessage oscGyroscope("/ic/imu/gyroscope");
OSCMessage oscMagnetometer("/ic/imu/magnetometer");
OSCMessage oscBreath("/ic/breath/resistance");

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define SAMPLERATE_DELAY_MS (10) // 100Hz
unsigned long currentMillis = 0;
unsigned long previousSampleMillis = 0;

#define DEVICE_IDENTIFIER "FEATHER/CORE"
#define IMU_IDENTIFIER "BNO"

// Stretch sensor
#define BREATH_SERIESRESISTOR 10000
#define BREATH_PIN A0

void setup() {
  Serial.begin(9600);

#ifdef DEBUG
  //while (!Serial);
#endif

  if (!bno.begin()) {
    Serial.print("Unable to initialize BNO055 (no sensor detected)");
    while (true);
  }

  // Calibration data (@ home)
  adafruit_bno055_offsets_t calibrationData;
  calibrationData.accel_offset_x = CALIBRATION_ACCEL_OFFSET_X;
  calibrationData.accel_offset_y = CALIBRATION_ACCEL_OFFSET_Y;
  calibrationData.accel_offset_z = CALIBRATION_ACCEL_OFFSET_Z;
  calibrationData.gyro_offset_x = CALIBRATION_GYRO_OFFSET_X;
  calibrationData.gyro_offset_y = CALIBRATION_GYRO_OFFSET_Y;
  calibrationData.gyro_offset_z = CALIBRATION_GYRO_OFFSET_Z;
  calibrationData.mag_offset_z = CALIBRATION_MAG_OFFSET_Z;
  calibrationData.mag_offset_x = CALIBRATION_MAG_OFFSET_X;
  calibrationData.mag_offset_y = CALIBRATION_MAG_OFFSET_Y;
  calibrationData.accel_radius = CALIBRATION_ACCEL_RADIUS;
  calibrationData.mag_radius = CALIBRATION_MAG_RADIUS;  
 
  bno.setSensorOffsets(calibrationData);
  

#ifdef DEBUG
  displaySensorDetails();
  displaySensorStatus();
#endif

  bno.setExtCrystalUse(true);

  // Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8, 7, 4, 2);
  status = WiFi.status();

  // Check for the presence of WiFi shield
  if (status == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    status = WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(10000);
  }

  // We are connected to WiFi now
#ifdef DEBUG
  printWiFiStatus();
#endif

  // Open UDP port for sending/receiving OSC messages
  udp.begin(OSC_PORT);
}

void loop() {
  // Start listening for websocket/socket.io discovery messages
  // If not connected to a server, try to connect
  // Only use websocket/socket.io when connected to server (connection is managed via OSC discovery)
  OSCMessage oscMessage;

  int size;
  if ( (size = udp.parsePacket()) > 0)
  {
    while (size--)
      oscMessage.fill(udp.read());

    if (!oscMessage.hasError()) {
      // Message: /ic/connector/socket.io (sf) host:port      
      oscMessage.route("/ic/controller/socket.io", oscDiscovery);      
    }
  }

  currentMillis = millis();
  if (currentMillis - previousSampleMillis >= SAMPLERATE_DELAY_MS) {
    previousSampleMillis = currentMillis;

    oscBundle.empty();    

    imu::Quaternion quat = bno.getQuat();
    oscQuaternion
      .empty()
      .add(DEVICE_IDENTIFIER)
      .add(IMU_IDENTIFIER)
      .add(quat.w())
      .add(quat.x())
      .add(quat.y())
      .add(quat.z());
    oscBundle.add(oscQuaternion);
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    oscEuler
      .empty()
      .add(DEVICE_IDENTIFIER)
      .add(IMU_IDENTIFIER)
      .add(euler.x())
      .add(euler.y())
      .add(euler.z());
    oscBundle.add(oscEuler);  

    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    oscGyroscope
      .empty()
      .add(DEVICE_IDENTIFIER)
      .add(IMU_IDENTIFIER)
      .add(gyroscope.x())
      .add(gyroscope.y())
      .add(gyroscope.z());
    oscBundle.add(oscGyroscope);  

    imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    oscMagnetometer
      .empty()
      .add(DEVICE_IDENTIFIER)
      .add(IMU_IDENTIFIER)
      .add(magnetometer.x())
      .add(magnetometer.y())
      .add(magnetometer.z());
    oscBundle.add(oscMagnetometer);  

    imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    oscLinearAccel
      .empty()  
      .add(DEVICE_IDENTIFIER)
      .add(IMU_IDENTIFIER)
      .add(linearAccel.x())
      .add(linearAccel.y())
      .add(linearAccel.z());
    oscBundle.add(oscLinearAccel);

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    oscAccel
      .empty() 
      .add(DEVICE_IDENTIFIER)
      .add(IMU_IDENTIFIER)
      .add(accel.x())
      .add(accel.y())
      .add(accel.z());
    oscBundle.add(oscAccel);
    
    float breath;
    breath = analogRead(BREATH_PIN);
    
    // convert the pin value to resistance
    breath = (1023 / breath) - 1;    // (1023/ADC - 1)
    breath = BREATH_SERIESRESISTOR / breath; // 10K / (1023/ADC - 1)

    oscBreath
      .empty() 
      .add(DEVICE_IDENTIFIER)
      .add(IMU_IDENTIFIER)
      .add(breath);
    oscBundle.add(oscBreath);

    udp.beginPacket(osc_ip, OSC_PORT);
    oscBundle.send(udp);
    udp.endPacket(); 

 #ifdef DEBUG
    //displayCalStatus();
#endif
  }
}

void oscDiscovery(OSCMessage & msg, int addrOffset ) {
  Serial.println("oscDiscovery: received /ic/connector/socket.io");
  //iterate through all the analog pins

  if (msg.isString(0) && msg.isInt(1)) {
    // Message contains a server and port
    char ip[15];
    int len = msg.getString(0, ip);
    int port = msg.getInt(1);
#ifdef DEBUG
    Serial.print(ip);
    Serial.print(":");
    Serial.println(port);
#endif

    // @TODO Set OSC server to ip 
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("Connect to WiFi with SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
}
