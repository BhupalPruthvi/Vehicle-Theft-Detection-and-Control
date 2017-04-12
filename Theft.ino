#define sclk 9
#define mosi 10
#define cs   13
#define rst  11
#define dc   12


// Color definitions
#define  BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include<Servo.h>
#include <SPI.h>
#include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#include <SoftwareSerial.h> //Load the Software Serial Library. This library in effect gives the arduino additional serial ports
SoftwareSerial mySerial(4,5); //Initialize SoftwareSerial, and tell it you will be connecting through pins 2 and 3
Adafruit_GPS GPS(&mySerial); //Create GPS object

Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);  

char incoming_char;
SoftwareSerial cell(8, 12);
char a[]={'S','T','O','P'};

String NMEA1;  //We will use this variable to hold our first NMEA sentence
String NMEA2;  //We will use this variable to hold our second NMEA sentence
char c;   

int inPin_1=2;
int inPin_2=A3;
int inPin_3=A4;

int outPin_1=A0;
int outPin_2=A1;
int outPin_3=A2;

int state_1=LOW;
int state_2=LOW;
int state_3=LOW;
int reading_1;
int reading_2;
int reading_3;
int previous_1;
int previous_2;
int previous_3;
int i=0;
int pos=0;
long time=0;
long debounce=200;

Servo myServo;
int servo_pin=7;


void setup() {
  // put your setup code here, to run once:
START();
  display.begin();

  Serial.println("init");
  uint16_t time = millis();
  display.fillScreen(BLACK);
  time = millis() - time;
  
  Serial.begin(9600);
pinMode(inPin_1, INPUT);
pinMode(inPin_2, INPUT);
pinMode(inPin_3, INPUT);
attachInterrupt(digitalPinToInterrupt(inPin_1), STOP, HIGH);
pinMode(outPin_1, OUTPUT);
pinMode(outPin_2, OUTPUT);
pinMode(outPin_3, OUTPUT);

myServo.attach(servo_pin);

  GPS.begin(9600);       //Turn GPS on at baud rate of 9600
  GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 H
delay(1000);
  cell.begin(9600);
  START();
  Serial.print("Starting modem communication...");
  delay(1000);
  Serial.println("OK\nIntroduce your AT commands:\n"); 
  

}

void loop()
{
  //driveMode();
  //readGPS();
  interrupts();
  
  theftMode();
  ReceiveMsg();
  
}


void driveMode()
{
  reading_1=digitalRead(inPin_1);
  if(reading_1==HIGH&&previous_1==LOW&&millis()-time>debounce)
  {
    if(state_1==HIGH)
    {
      state_1=LOW;
      Serial.println("Drive Mode-->OFF");
    }
    else
    {
      state_1=HIGH;
      Serial.println("Drive Mode-->ON");
    }
    time=millis();
  }
  digitalWrite(outPin_1, state_1);
  previous_1=reading_1;
}

void theftMode()
{
  interrupts();
  reading_2=digitalRead(inPin_2);
  if(reading_2==HIGH&&previous_2==LOW&&millis()-time>debounce)
  
  {
    interrupts();
    THEFTMODE();
    if(state_2==HIGH)
    {
      interrupts();
      state_2=LOW;
      Serial.println("Theft Mode-->OFF");
    }
    else
    {
      state_2=HIGH;
      Serial.println("Theft mode is ON");
      while(1)
      {
        interrupts();
        reading_3=digitalRead(inPin_3);
        if(reading_3==HIGH&&previous_3==LOW&&millis()-time>debounce)
        
        {
          if(state_3==LOW)
          {
            DR();
            while(1){
              interrupts();
            Serial.println("Drivemode is ON in theft mode");
            Serial.println("car theft in progress");
            
             for (pos = 0; pos <= 180; pos += 1){
              
              delay(15);
              interrupts();
              ReceiveMsg();
              myServo.write(pos);
             }
             delay(500);
              for (pos = 180; pos >= 0; pos -= 1){
               
                delay(15);
                interrupts();
                ReceiveMsg();
                myServo.write(pos); 
              }
              
            digitalWrite(outPin_3, state_3);
            state_3=HIGH;
            delay(1000);
            ReceiveMsg();
            delay(1000);
            sendMsg();
            //readGPS();
            
            Serial.println("******************");
            }
          }
          else
          {
            Serial.println("Drive mode off in theft mode");
            state_3=LOW;
          }
          digitalWrite(outPin_3, state_3);
          previous_3=reading_3;
          }
        }
       time=millis();
      }
      digitalWrite(outPin_2, state_2);
      previous_2=reading_2;
    }
}

void (*resetfunction)(void)=0;

void STOP()
{
  //Serial.println("RESET");
  resetfunction();
}

void readGPS(){  //This function will read and remember two NMEA sentences from GPS
  clearGPS();    //Serial port probably has old or corrupt data, so begin by clearing it all out
  while(!GPS.newNMEAreceived()) { //Keep reading characters in this loop until a good NMEA sentence is received
  c=GPS.read(); //read a character from the GPS
  }
GPS.parse(GPS.lastNMEA());  //Once you get a good NMEA, parse it
NMEA1=GPS.lastNMEA();      //Once parsed, save NMEA sentence into NMEA1
while(!GPS.newNMEAreceived()) {  //Go out and get the second NMEA sentence, should be different type than the first one read above.
  c=GPS.read();
  }
GPS.parse(GPS.lastNMEA());
NMEA2=GPS.lastNMEA();
  Serial.println(NMEA1);
  Serial.println(NMEA2);
  Serial.println("");
}
void clearGPS() {  //Since between GPS reads, we still have data streaming in, we need to clear the old data by reading a few sentences, and discarding these
while(!GPS.newNMEAreceived()) {
  c=GPS.read();
  }
GPS.parse(GPS.lastNMEA());
while(!GPS.newNMEAreceived()) {
  c=GPS.read();
  }
GPS.parse(GPS.lastNMEA());
}

void sendMsg()
 {
  interrupts();
    cell.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  cell.println("AT+CMGS=\"+17472461820\"\r"); // Replace x with mobile number
  delay(1000);
  Serial.println("SMS sent");
  MSGSENT();
  cell.println("Car heist in progress");// The SMS text you want to send
  cell.println("GPS COORDINATES");
  cell.println("$GPRMC,235951.800,V,,,,,0.00,0.00,050180,,,N*40");
  cell.println("$GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32");
  cell.println((char)26);
  delay(1000);
 }

void ReceiveMsg()
{
  {
  if(cell.available() > 0)
  {
    incoming_char=cell.read();
   // if((incoming_char >= ' ') && (incoming_char <= 'z'))
      Serial.print(incoming_char);
  }

  if(Serial.available() > 0)
  {
    incoming_char = Serial.read();
    cell.println(incoming_char);
  }
  if(incoming_char==a[3])
  {
    interrupts();
    SSSS();
    digitalWrite(outPin_1, HIGH);
    Serial.println("SUCCESS");
    digitalWrite(inPin_1, HIGH);
    resetfunction();
  }
  }
}

void START() {
  display.fillScreen(YELLOW);
  display.setCursor(10, 25);
  display.setTextColor(RED,GREEN);  
  display.setTextSize(1);
  display.println("VEHICLE THEFT");
  display.setCursor(20, 35);
  display.println("DETECTION");
  display.setCursor(10, 45);
  display.setTextColor(BLUE);
  display.println("-------------");
}

void THEFTMODE() {
  display.fillScreen(BLACK);
  display.setCursor(5, 10);
  display.setTextColor(GREEN);
  display.println("---------------");  
  display.setTextSize(2);
  display.setCursor(15,15);
  display.setTextColor(RED, CYAN);
  display.println("THEFT");
  display.setCursor(5, 30);
  display.println("MODE ON");
  display.setCursor(5, 45);
  display.setTextSize(1);
  display.setTextColor(GREEN);
  display.println("---------------");
}

void DR() {
  display.fillScreen(WHITE);
  display.setCursor(10, 25);
  display.setTextColor(RED, YELLOW);  
  display.setTextSize(1);
  display.println("DRIVE MODE ON");
  display.setTextColor(BLUE, YELLOW);
  display.setCursor(10, 40) ;
  display.println("IN THEFT MODE");
}

void MSGSENT() {
  display.fillScreen(CYAN);
  display.setCursor(20,5);
  display.setTextColor(BLUE, YELLOW);  
  display.setTextSize(1);
  display.println("SMS SENT");
  display.setCursor(10, 20);
  display.setTextColor(RED);
  display.println("CAR HEIST IN"); 
  display.setCursor(25, 30);
  display.println("PROGRESS");// The SMS text you want to send
  display.setCursor(3, 45);
  display.setTextColor(MAGENTA, YELLOW);
  display.println("GPS COORDINATES");
  //display.println("$GPRMC,235951.800,V,,,,,0.00,0.00,050180,,,N*40");
  //display.println("$GPVTG,0.00,T,,M,0.00,N,0.00,K,N*32");
}

void SSSS() {
  display.fillScreen(WHITE);
  display.setCursor(0, 5);
  display.setTextColor(RED);  
  display.setTextSize(1);
  display.println("STOP");
}

