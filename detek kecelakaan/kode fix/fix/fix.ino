#include <Wire.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h> 
#include "fis_header.h"

IPAddress server(34,243,178,169);
char ssid[] = "HUAWEI-A0E4";
char pass[] = "05690998";
int status = WL_IDLE_STATUS;

WiFiEspClient espClient;
PubSubClient client(espClient);
SoftwareSerial wifi(5, 6);

TinyGPS gps; // create gps object 
float lat = 28.5458,lon = 77.1703;
SoftwareSerial gpsSerial(3, 4);

const int MPU6050_addr=0x68;
int16_t AccX,AccY,AccZ,GyroX,GyroY,GyroZ;

char nilai;
char hasil[8];

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  gpsSerial.begin(9600); // connect gps sensor 
  
  Serial.begin(9600);
  wifi.begin(9600);
  WiFi.init(&wifi);  
  if (WiFi.status() == WL_NO_SHIELD) {
  Serial.println("WiFi shield not present");
  while (true);
  }

  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("You're connected to the network");
  
  client.setServer(server, 17014);
  client.setCallback(callback);
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i=0;i<length;i++) {
    char receivedChar = (char)payload;
    Serial.print(receivedChar);  
  }
  Serial.println();
}

void loop() {
  if (!client.connected()) {
  reconnect();
  }
  bacaMPU();
  bacaGPS();
  fuzzyfikasi();
  client.loop();
  delay(1000);
}

void fuzzyfikasi()  {
  
}

void bacaGPS()  {
    while(gpsSerial.available()){  
      if(gps.encode(gpsSerial.read()))  {  
      gps.f_get_position(&lat,&lon);
      // display position 
      Serial.print("Position: "); 
      Serial.print("Latitude:"); 
      Serial.print(lat,6); 
      Serial.print(";"); 
      Serial.print("Longitude:"); 
      Serial.println(lon,6);   
      } 
  } 

  nilai = lat;
  client.publish("lat", char(nilai));
  Serial.println("Sending lat."); 

  nilai = lon;
  client.publish("lon", char(nilai));   
  delay(100); 
}

void bacaMPU() {
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
  
  Serial.print("accX = "); Serial.print(AccX);
  dtostrf(AccX, 6, 2, hasil);
  client.publish("accX", hasil);

  Serial.print(" accY = "); Serial.print(AccY);
  dtostrf(AccY, 6, 2, hasil);
  client.publish("accY", hasil);

  Serial.print(" accZ = "); Serial.print(AccZ);
  dtostrf(AccZ, 6, 2, hasil);
  client.publish("accZ", hasil);

  Serial.print(" gyroX = "); Serial.print(GyroX);
  dtostrf(GyroX, 6, 2, hasil);
  client.publish("gyroX", hasil);

  Serial.print(" gyroY = "); Serial.print(GyroY);
  dtostrf(GyroY, 6, 2, hasil);
  client.publish("gyroY", hasil);

  Serial.print(" gyroZ = "); Serial.print(GyroZ);
  dtostrf(GyroZ, 6, 2, hasil);
  client.publish("gyroZ", hasil);

  
  delay(100);
}
  
void reconnect() {
  while (!client.connected()) {
  Serial.print("Attempting connection... ");
  if (client.connect("esp8","mggxilrp","jw7BEm0GZrDT")) {
  Serial.println("connected"); 
  } else {
  Serial.print("failed");
  Serial.print(client.state());
  Serial.println(" try again in 5 seconds");
  // Wait 5 seconds before retrying
  delay(5000);
  }
  }
}
