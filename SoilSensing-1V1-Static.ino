#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_SHT4x.h"
#include <BluetoothSerial.h>

#define LED_PIN 13
#define RGB_DATA_PIN 0
#define RGB_PWR_PIN 2
#define SOIL_SENSOR_PIN 25

const String NODE_NAME = "ESP32-1";
const String NEXT_HOP_NAME = "ESP32-0";

BluetoothSerial SerialBT;
Adafruit_NeoPixel pixels(1, RGB_DATA_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

String msg = "";
uint32_t nextSensorSampleTime = UINT32_MAX;
bool hasSoilSensor = false;
bool hasTempSensor = false;

void setLed(bool state){
  digitalWrite(LED_PIN, state);
}

void setRGB(uint8_t state, uint32_t duration = 100){
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
    randomSeed(analogRead(26));
    updateSensorSampleTime();
  }

  // Bluetooth init
  SerialBT.begin(NODE_NAME);
  SerialBT.setTimeout(1000);

  Serial.println("Initialization complete!");
  setLed(false);
}

void loop() {
  // Update sensor message
  if(millis() > nextSensorSampleTime){
    setRGB(1, 250);
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
    Serial.println("Generated new message: " + msg);
    setRGB(0);
  }

  // Check if received message
  else if(SerialBT.available()){
    setRGB(3, 250);
    msg = SerialBT.readStringUntil('\n');
    Serial.println("Received new message: " + msg);
    setRGB(0);
  }

  // Connect and transmit if message is ready
  if(msg.length() > 0 && NEXT_HOP_NAME.length() > 0){
    setRGB(2, 250);

    // Change to master mode
    SerialBT.disconnect();
    SerialBT.end();
    SerialBT.begin(NODE_NAME + "-M", true);
    
    while(true){
      // Connect to next hop node
      if(SerialBT.connect(NEXT_HOP_NAME)){
        // Send message
        Serial.println("Sent message <" + msg + "> to " + NEXT_HOP_NAME);
        SerialBT.println(msg);

        // Change to client mode
        SerialBT.disconnect();
        SerialBT.end();
        SerialBT.begin(NODE_NAME);
        msg = "";
        break;
      }
      else{
        Serial.println("Unable to connect to next hop...");
      }
      delay(2000);
    }

    setRGB(0);
  }
}
