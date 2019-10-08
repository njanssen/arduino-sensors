#define DEBUG

#include <WiFi101.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "credentials.h"
#define OSC_PORT 9000
IPAddress osc_ip(255,255,255,255);
WiFiUDP udp;
int status;

OSCMessage oscEuler("/ic/imu/bno055/euler");
OSCMessage oscAccel("/ic/imu/bno055/accel");

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define BNO055_SAMPLERATE_DELAY_MS (100) // 1Hz
unsigned long currentMillis = 0;
unsigned long previousBnoSampleMillis = 0;

void setup() {
  Serial.begin(9600);

#ifdef DEBUG
  while (!Serial);
#endif

  if (!bno.begin()) {
    Serial.print("Unable to initialize BNO055 (no sensor detected)");
    while (true);
  }

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
      oscMessage.route("/ic/connector/socket.io", oscDiscovery);
    }
  }

  currentMillis = millis();
  if (currentMillis - previousBnoSampleMillis >= BNO055_SAMPLERATE_DELAY_MS) {
    previousBnoSampleMillis = currentMillis;
    // Time to check our BNO055 sensor
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//
//    
//    oscEuler.add(euler.x());
//    oscEuler.add(euler.y());
//    oscEuler.add(euler.z());
//    udp.beginPacket(osc_ip, OSC_PORT);
//    oscEuler.send(udp);
//    udp.endPacket(); 
//    oscEuler.empty(); 

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    oscAccel.empty();     
    oscAccel.add(accel.x());
    oscAccel.add(accel.y());
    oscAccel.add(accel.z());
    udp.beginPacket(osc_ip, OSC_PORT);
    oscAccel.send(udp);
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
