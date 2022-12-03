#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_SHT4x.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define LED_PIN 13
#define RGB_DATA_PIN 0
#define RGB_PWR_PIN 2
#define SOIL_SENSOR_PIN 34

const String NODE_NAME = "ESP32-2";

WiFiUDP udp;
Adafruit_NeoPixel pixels(1, RGB_DATA_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

const char* ssid     = "Tanks!";
const char* password = "BattleBegin";
const IPAddress broadcastIp = IPAddress(255,255,255,255);
const uint16_t port = 12000;

String msg = "";
uint32_t nextSensorSampleTime = UINT32_MAX;
bool hasSoilSensor = false;
bool hasTempSensor = false;

void setLed(bool state){
  digitalWrite(LED_PIN, state);
}

void setRGB(uint8_t state, uint32_t duration = 0){
  if(state == 1){
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  }
  else if(state == 2){
    pixels.setPixelColor(0, pixels.Color(255, 191, 0));
  }
  else if(state == 3){
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  }
  else if(state == 4){
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  }
  else{
    pixels.clear();
  }
  pixels.show();
  delay(duration);
}

void updateSensorSampleTime(){
  nextSensorSampleTime = millis() + 30000 + random(-5000, 6000);
}

uint16_t readSoilSensor(){
  return analogRead(SOIL_SENSOR_PIN);
}

void setup() {
  // Serial init
  Serial.begin(115200);
  Serial.println("Initializing...");

  // I2C init
  Wire.begin();

  // LED init
  pinMode(LED_PIN, OUTPUT);
  setLed(true);

  // RGB LED init
  pinMode(RGB_PWR_PIN, OUTPUT);
  digitalWrite(RGB_PWR_PIN, HIGH);
  pixels.begin();
  pixels.setBrightness(50);
  // setRGB(1, 500);
  // setRGB(2, 500);
  // setRGB(3, 500);
  // setRGB(4, 500);
  setRGB(0);
  
  // Humidity / temp sensor init
  hasTempSensor = sht4.begin();
  if(hasTempSensor){
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    Serial.println("Found SHT40 sensor...");
  }
  else{
    Serial.println("Unable to find SHT40 sensor...");
  }

  // Soil sensor init
  pinMode(SOIL_SENSOR_PIN, INPUT);
  hasSoilSensor = true;

  // Sensor sample init
  if(hasSoilSensor || hasTempSensor){
    randomSeed(analogRead(39));
    updateSensorSampleTime();
  }

  // WiFi init
  WiFi.begin(ssid, password); // Connect to WiFi
  while(WiFi.status() != WL_CONNECTED); // Wait until connected to WiFi
  // WiFi.config(ip, WiFi.gatewayIP(), WiFi.subnetMask());
  // while(WiFi.localIP() != ip); // Wait for IP to update
  while(!udp.begin(port)); // Begin UDP connection

  Serial.println("Initialization complete!");
  setLed(false);
}

void loop() {
  // Update sensor message
  if(millis() > nextSensorSampleTime){
    setRGB(1, 100);
    // Serial.println("Updating sensor values...");
    String currTemp = "";
    String currHumidity = "";
    String currSoilMoisture = "";

    // Temp sensor update
    if(hasTempSensor){
      sensors_event_t humidity, temp;
      sht4.getEvent(&humidity, &temp);
      currTemp = String(temp.temperature);
      currHumidity = String(humidity.relative_humidity);
      // Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
      // Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    }

    // Soil sensor update
    if(hasSoilSensor){
      currSoilMoisture = String(readSoilSensor());
    }

    // Update message
    msg = "$" + NODE_NAME + "," + currTemp+ "," + currHumidity + "," + currSoilMoisture;
    updateSensorSampleTime();
    Serial.println("Generated new message:\t" + msg);
    setRGB(0);
  }

  uint8_t msgLength = udp.parsePacket();
  if(msgLength > 0){
    setRGB(4, 100);
    char msgBuffer[msgLength];
    udp.read(msgBuffer, msgLength);  // Read received data
    String rxMsg = String(msgBuffer);
    Serial.println("Received new message:\t" + rxMsg);
    setRGB(0);
  }

  // Connect and transmit if message is ready
  if(msg.length() > 0){
    setRGB(2, 100);

    uint8_t msgLength = msg.length() + 1;
    uint8_t msgBuffer[msgLength];
    msg.getBytes(msgBuffer, msgLength);

    while(true){
      setRGB(3);
      uint32_t txStartTime = micros();
      if(udp.beginPacket(broadcastIp, port)){  // Begin packet and set destination
        udp.write(msgBuffer, msgLength);   // Write bytes from buffer to packet
        if(udp.endPacket()){        //Send packet
          uint32_t txDuration = micros() - txStartTime;
          Serial.println("Sent message <" + msg + "> in " + String(txDuration) + "us");
          msg = "";
          break;
        }
      }
      else{
        Serial.println("Unable to connect to next hop...");
      }
      delay(2000);
    }

    setRGB(0);
  }
}
