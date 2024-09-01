//AVOIDING OBSTRACALE

#include <AFMotor.h>  
#include <NewPing.h>//for sonar sensor working
#include <Servo.h> 
 
#define TRIG_PIN A0 
#define ECHO_PIN A1
#define MAX_DISTANCE 100 //distance of ultrasonographic
#define MAX_SPEED 150 // sets speed of DC  motors
#define MAX_SPEED_OFFSET 20//for motor
 NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
 
AF_DCMotor motor1(1, MOTOR12_64KHZ); 
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR34_64KHZ);
AF_DCMotor motor4(4, MOTOR34_64KHZ);
Servo myservo;   
 
boolean goesForward=false;//intitally off
int distance = 100;
int speedSet = 0;//intial speed

//FALL DETECTION
#include <Wire.h>
#include <ESP8266WiFi.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred
byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;
// WiFi network info.
const char *ssid = 
"Alsan"; // Enter your Wi-Fi Name
const char *pass = "01234567890"; // Enter your Wi-Fi Password
void send_event(const char *event);
const char *host = "maker.ifttt.com";
const char *privateKey = "xxx-xxx-xxx-xxx-xxx-xxx-xxx";

//GPS TRACKER
#include "Adafruit_FONA.h"
#include <TinyGPS++.h>
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

#define FONA_RX 9
#define FONA_TX 8
#define FONA_RST 10

char replybuffer[255];
String commands = "";
String YourArduinoData = "";
char latitude[15];
char longitude[15];
char fonaNotificationBuffer[64];
char smsBuffer[250];

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);



void setup(){
	//AVOIDING OBSTRACLE
  myservo.attach(A2 );  //survomotor attach with arduino a2
  myservo.write(115); // speed of survomotor
  delay(2000);// for stop program for miliseconds, it decleare in miliseconds
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPi
  delay(100);
  distance = readPing();
  delay(100);
  
  //FALL DETECTION
   Serial.begin(115200);
 Wire.begin();
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
 Serial.println("Wrote to IMU");
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");              // print ... till not connected
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  //GPS TRACKER
  
  getloc();
  char *bufPtr = fonaNotificationBuffer;
  if (fona.available()) {
    int slot = 0;
    int charCount = 0;
    do {
      *bufPtr = fona.read();
      oled.clear();
      oled.print(fonaNotificationBuffer);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer) - 1)));
    
    *bufPtr = 0;
    if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) {
      Serial.print("slot: ");
      Serial.println(slot);
      char callerIDbuffer[32];

      if (!fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: "));
      Serial.println(callerIDbuffer);

      uint16_t smslen;
      if (fona.readSMS(slot, smsBuffer, 190, &smslen)) {
        Serial.println(smsBuffer);
        commands = smsBuffer;
        oled.clear();
        oled.print(smsBuffer);
        Serial.println(commands);
        if (commands == "get loc") {
          digitalWrite(13, 1);
          YourArduinoData += ("https://www.google.com/maps/place/");
          YourArduinoData.concat(latitude);
          YourArduinoData.concat(",");
          YourArduinoData.concat(longitude);
          Serial.println(YourArduinoData);
          char message[51];
          YourArduinoData.toCharArray(message, 51);

          if (!fona.sendSMS(callerIDbuffer, message)) {
            Serial.println(F("Failed"));
            oled.clear();
            oled.println("Failed");
          } else {
            Serial.println(F("Sent!"));
            oled.clear();
            oled.println("Location Sent!");
            digitalWrite(13, 0);
          }
        }
        if ((fona.deleteSMS(slot)) || (fona.deleteSMS(1)) || (fona.deleteSMS(2)) || (fona.deleteSMS(3))) {

        } else {
          fona.print(F("AT+CMGD=?\r\n"));
        }
      }
    }
  }
	
	
	
}

void loop(){
	//AVOIDING OBSTRACKLE
	
 int distanceR = 0;//initial distance of right
 int distanceL = 0;//intial distance of left
 delay(40);
 
 if(distance<=25)
 {
  moveStop();//for stop the chair using stop function
  delay(100);
  moveBackward();//for moving backward the chair using backward function
  delay(200);
  moveStop();
  delay(200);
  distanceR = lookRight();
  delay(200);
  distanceL = lookLeft();
  delay(200);
 
  if(distanceR>=distanceL)
  {
    turnRight();
    moveStop();
  } 
 
  else
 
  {
    turnLeft();
    moveStop();
  }
 } 
 
 else
 {
  moveForward();
 }
 distance = readPing();
	
	//FALL DETECTION
	
	mpu_read();
 ax = (AcX-2050)/16384.00;
 ay = (AcY-77)/16384.00;
 az = (AcZ-1947)/16384.00;
 gx = (GyX+270)/131.07;
 gy = (GyY-351)/131.07;
 gz = (GyZ+136)/131.07;
 // calculating Amplitute vactor for 3 axis
 float Raw_Amp = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
 int Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
 Serial.println(Amp);
 if (Amp<=2 && trigger2==false){ //if AM breaks lower threshold (0.4g)
   trigger1=true;
   Serial.println("TRIGGER 1 ACTIVATED");
   }
 if (trigger1==true){
   trigger1count++;
   if (Amp>=12){ //if AM breaks upper threshold (3g)
     trigger2=true;
     Serial.println("TRIGGER 2 ACTIVATED");
     trigger1=false; trigger1count=0;
     }
 }
 if (trigger2==true){
   trigger2count++;
   angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5); Serial.println(angleChange);
   if (angleChange>=30 && angleChange<=400){ //if orientation changes by between 80-100 degrees
     trigger3=true; trigger2=false; trigger2count=0;
     Serial.println(angleChange);
     Serial.println("TRIGGER 3 ACTIVATED");
       }
   }
 if (trigger3==true){
    trigger3count++;
    if (trigger3count>=10){ 
       angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
       //delay(10);
       Serial.println(angleChange); 
       if ((angleChange>=0) && (angleChange<=10)){ //if orientation changes remains between 0-10 degrees
           fall=true; trigger3=false; trigger3count=0;
           Serial.println(angleChange);
             }
       else{ //user regained normal orientation
          trigger3=false; trigger3count=0;
          Serial.println("TRIGGER 3 DEACTIVATED");
       }
     }
  }
 if (fall==true){ //in event of a fall detection
   Serial.println("FALL DETECTED");
   send_event("fall_detect"); 
   
   //GPS TRACKER
   getloc();
  char *bufPtr = fonaNotificationBuffer;
  if (fona.available()) {
    int slot = 0;
    int charCount = 0;
    do {
      *bufPtr = fona.read();
      oled.clear();
      oled.print(fonaNotificationBuffer);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaNotificationBuffer) - 1)));
    
    *bufPtr = 0;
    if (1 == sscanf(fonaNotificationBuffer, "+CMTI: " FONA_PREF_SMS_STORAGE ",%d", &slot)) {
      Serial.print("slot: ");
      Serial.println(slot);
      char callerIDbuffer[32];

      if (!fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: "));
      Serial.println(callerIDbuffer);

      uint16_t smslen;
      if (fona.readSMS(slot, smsBuffer, 190, &smslen)) {
        Serial.println(smsBuffer);
        commands = smsBuffer;
        oled.clear();
        oled.print(smsBuffer);
        Serial.println(commands);
        if (commands == "get loc") {
          digitalWrite(13, 1);
          YourArduinoData += ("https://www.google.com/maps/place/");
          YourArduinoData.concat(latitude);
          YourArduinoData.concat(",");
          YourArduinoData.concat(longitude);
          Serial.println(YourArduinoData);
          char message[51];
          YourArduinoData.toCharArray(message, 51);

          if (!fona.sendSMS(callerIDbuffer, message)) {
            Serial.println(F("Failed"));
            oled.clear();
            oled.println("Failed");
          } else {
            Serial.println(F("Sent!"));
            oled.clear();
            oled.println("Location Sent!");
            digitalWrite(13, 0);
          }
        }
        if ((fona.deleteSMS(slot)) || (fona.deleteSMS(1)) || (fona.deleteSMS(2)) || (fona.deleteSMS(3))) {

        } else {
          fona.print(F("AT+CMGD=?\r\n"));
        }
      }
    }
  }
   
   fall=false;
   }
 if (trigger2count>=6){ //allow 0.5s for orientation change
   trigger2=false; trigger2count=0;
   Serial.println("TRIGGER 2 DECACTIVATED");
   }
 if (trigger1count>=6){ //allow 0.5s for AM to break upper threshold
   trigger1=false; trigger1count=0;
   Serial.println("TRIGGER 1 DECACTIVATED");
   }
  delay(100);
  
	
}


//AVOIDING OBSTRACKEL
int lookRight()
{
    myservo.write(50); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
}
 
int lookLeft()
{
    myservo.write(170); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
    delay(100);
}
 
int readPing() { 
  delay(100);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}
 
void moveStop() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  } 
 
void moveForward() {
 
 if(!goesForward)
  {
    goesForward=true;
    motor1.run(FORWARD);      
    motor2.run(FORWARD);
    motor3.run(FORWARD); 
    motor4.run(FORWARD);     
   for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
   {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
   }
  }
}
 
void moveBackward() {
    goesForward=false;
    motor1.run(BACKWARD);      
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);  
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}  
 
void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);     
  delay(500);
  motor1.run(FORWARD);      
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);      
} 
 
void turnLeft() {
  motor1.run(BACKWARD);     
  motor2.run(BACKWARD);  
  motor3.run(FORWARD);
  motor4.run(FORWARD);   
  delay(500);
  motor1.run(FORWARD);     
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

//FALL DETECTION

void mpu_read(){
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 }
void send_event(const char *event)
{
  Serial.print("Connecting to "); 
  Serial.println(host);
    // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }
    // We now create a URI for the request
  String url = "/trigger/";
  url += event;
  url += "/with/key/";
  url += privateKey;
  Serial.print("Requesting URL: ");
  Serial.println(url);
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
  while(client.connected())
  {
    if(client.available())
    {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    } else {
      // No data yet, wait a bit
      delay(50);
    };
  }
  Serial.println();
  Serial.println("closing connection");
  client.stop();
}

//GPS TRACKER

void getloc() {
  if (Serial.available() > 0)
    if (gps.encode(Serial.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    oled.clear();
    oled.println("No GPS detected: check wiring..");
    while (true)
      ;
  }
}
void displayInfo() {

  oled.clear();
  oled.print("Lati-");
  oled.print(latitude);
  oled.println(",Long-");
  oled.print(longitude);
  oled.println(" Date");
  oled.print(gps.date.month());
  oled.print("-");
  oled.print(gps.date.day());
  oled.print("-");
  oled.print(gps.date.year());
  oled.println("time");
  oled.print(gps.time.hour());
  oled.print(":");
  oled.print(gps.date.day());
  oled.print(":");
  oled.print(gps.time.minute());

  dtostrf(gps.location.lat(), 8, 7, latitude);
  dtostrf(gps.location.lng(), 8, 7, longitude);
  delay(50);
}
