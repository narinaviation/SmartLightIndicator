//========== ส่วนการ Include Libraries ต่างๆ

#include <AuthClient.h>
#include <MicroGear.h>
#include <MQTTClient.h>
#include <SHA1.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <WiFiManager.h>

//========== ส่วนการ Define เพื่อกำหนดขา (Pins) ต่างๆที่ต่ออุปกรณ์ไว้

#define redpin  15        //==== ขาควบคุม สีแดง ของ RGB LED (GPIO 15)
#define greenpin  12      //==== ขาควบคุม สีเขียว ของ RGB LED (GPIO 12)
#define bluepin 13        //==== ขาควบคุม สีน้ำเงิน ของ RGB LED (GPIO 13)

//========== ส่วนยืนยันตัวตน (Authentication)
//=== ใส่ AppID, Key, Secret ที่ได้จาก NETPIE และตั้งชื่อ Alias ให้กับอุปกรณ์ว่า smartlight1

#define APPID       "NarinSmartHome"
#define KEY         "4frexyWAZviMWjw"
#define SECRET      "rC3AJEtxhQHe2OGxY7mdGNAgl"
#define ALIAS       "smartlight1"

//========== ส่วนการประกาศตัวแปรต่างๆ

int lightlevel;           //==== ตัวแปรสำหรับเก็บค่าความสว่างของแสง ณ ปัจจุบัน

int lightlevel_high;      //==== ตัวแปรสำหรับบันทึก(จำ)ค่าความสว่างสูงสุด
int lightlevel_low;       //==== ตัวแปรสำหรับบันทึก(จำ)ค่าความสว่างต่ำสุด
int lightlevel_mean;      //==== ตัวแปรสำหรับเก็บค่ากลางของแสงที่คำนวนได้จากการเฉลี่ยแสงสูงสุด และต่ำสุด

int lightlevel_show;      //==== ตัวแปรที่ใช้เก็บค่าที่แปลงจาก 0-1023 ของ AnalogRead ไปเป็น 0-100 เพื่อแสดงค่าเป็นเปอร์เซ็นต์

//=====================================================

WiFiClient client;
AuthClient *authclient;
MicroGear microgear(client);

//=============== ฟังค์ชั่นช่วยกำหนดสี LED ==================
//    การใช้งาน = led_color(ค่าสีแดง,ค่าสีเขียว,ค่าสีน้ำเงิน)
//      โดยที่ค่าอยู่ระหว่าง 0 (ดับ) ถึง 1024 (เข้มที่สุด)
//=====================================================

void led_color(int red,int green,int blue)
{
  analogWrite(redpin, red);
  analogWrite(greenpin, green);
  analogWrite(bluepin, blue);
}

//=====================================================

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) 
{
  Serial.print("Incoming message --> ");
  Serial.print(topic);
  Serial.print(" : ");
  char strState[msglen];
  for (int i = 0; i < msglen; i++) 
  {
    strState[i] = (char)msg[i];
    Serial.print((char)msg[i]);
  }
  Serial.println();

  String stateStr = String(strState).substring(0, msglen);

//========== ส่วนการประมวลผลข้อความที่ได้รับมา

  if(stateStr == "SETHIGH")               //========== "เมื่อมีการส่งข้อความมาว่า SETHIGH"
  {
    lightlevel_high = analogRead(A0);     //========== จะเก็บค่าความสว่างในตอนที่มีข้อความเข้ามาไว้ว่าเป็นค่าความสว่างสูงสุด
    led_color(0,0,1024);                  //========== ให้ LED เป็นสีน้ำเงิน
    delay(200);                           //========== รอเวลา 0.2 วินาที
  }
  
  if(stateStr == "SETLOW")                //========== "เมื่อมีการส่งข้อความมาว่า SETLOW"
  {
    lightlevel_low = analogRead(A0);      //========== จะเก็บค่าความสว่างในตอนที่มีข้อความเข้ามาไว้ว่าเป็นค่าความสว่างต่ำสุด
    led_color(0,0,1024);                  //========== ให้ LED เป็นสีน้ำเงิน
    delay(200);                           //========== รอเวลา 0.2 วินาที
  }
}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) 
{
  Serial.println("Connected to NETPIE...");
  
  for(int i = 0; i < 5 ;i++)      //========== เมื่อต่อ NETPIE ได้แล้ว ให้ LED กระพิบสีเขียว 5 รอบ
  {
    led_color(0,0,0);
    delay(200);
    led_color(0,1024,0);
    delay(200);
  }
  
  led_color(0,0,0);              //=========== แล้วก็ให้ LED ดับไป
  delay(3000);                   //=========== เป็นเวลา 3 วินาที
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting...");
  
  pinMode(redpin, OUTPUT);    //========== กำหนดขา redpin (GPIO15) ให้เป็นแบบ OUTPUT
  pinMode(greenpin, OUTPUT);  //========== กำหนดขา greenpin (GPIO15) ให้เป็นแบบ OUTPUT
  pinMode(bluepin, OUTPUT);   //========== กำหนดขา bluepin (GPIO15) ให้เป็นแบบ OUTPUT

  lightlevel_high = 1024;     //========== ในตอนเริ่มต้น กำหนดให้ค่าแสงสว่างสูงสุด เท่ากับ 1024 (เป็นค่าสูงสุดของ AnalogRead)
  lightlevel_low  = 0;        //========== และกำหนดให้ค่าแสงสว่างต่ำสุด เท่ากับ 0 (เป็นต่ำสุดของ AnalogRead)

  //========== ส่วนการทำงานของ Wifi Manager

  led_color(0,0,1024);        //========== เริ่มต้น ให้ LED เป็นสีน้ำเงิน จนกว่าจะ Config WifiManager เสร็จ
  
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  if(!wifiManager.autoConnect("SmartLightIndicator"))   //ถ้าเชื่อมต่อไม่ได้ จะเปลี่ยนตัวเองเป็น Accesspoint ชื่อนี้
  {
    led_color(1024,0,0);      //========== หากเชื่อมต่อไม่ได้ และเกินเวลาที่กำหนดไว้ ให้ LED เป็นสีแดง
    Serial.println("Failed to connect and hit timeout");
    delay(3000);              //========== เป็นเวลา 3 วินาที
    ESP.reset();              //========== แล้วรีเซ็ตบอร์ดใหม่
    delay(5000);
  }

  led_color(0,1024,0);        //========== แต่ถ้าเชื่อมต่อ WiFi สำเร็จ LED จะเป็นสีเขียวค้าง
  
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);

  microgear.init(KEY,SECRET,ALIAS);
  microgear.connect(APPID);
}

void loop() 
{  
  if (microgear.connected()) 
  {
    lightlevel = analogRead(A0);                                    //อ่านค่าความสว่างปัจจุบันไปเก็บไว้ในตัวแปร lightlevel
    lightlevel_show = map(lightlevel,0,1023,0,100);                 //สำหรับค่าความสว่างที่จะนำไปแสดงผล จะถูก map ค่าให้อยู่ในช่วง 0 ถึง 100 (อยากให้แสดงเป็น %)
                     
    
    microgear.publish("/SmartLight1/LightLevel",lightlevel_show);     //ทำการ Publish (ส่งค่า) ของตัวแปร lightlevel_show (ที่เป็น 0 ถึง 100) ไปที่ Topic/AppID/SmartLight1/LightLevel
    
    lightlevel_mean = (lightlevel_high + lightlevel_low)/2;           //คำนวนค่ากลางของความสว่าง (โดยการเอาค่าสูงสุด + ค่าต่ำสุด) หาร 2 (เฉลี่ยค่า)
    
    if(lightlevel > lightlevel_mean)                                  //เปรียบเทียบว่า ความสว่างที่อ่านได้ ณ ปัจจุบัน มากกว่าค่ากลางหรือไม่
    {
      microgear.publish("/SmartLight1/Status","HIGH");                //หากมากกว่า จะ Publish ข้อความว่า "HIGH" ไปที่ Topic/AppID/SmartLight1/Status
      led_color(0,1024,0);                                            //และทำการเปลี่ยนสีของ LED ให้เป็นสีเขียว
    }
    if(lightlevel < lightlevel_mean)                                  //เปรียบเทียบว่า ความสว่างที่อ่านได้ ณ ปัจจุบัน น้อยกว่าค่ากลางหรือไม่
    {
      microgear.publish("/SmartLight1/Status","LOW");                 //หากมากกว่า จะ Publish ข้อความว่า "HIGH" ไปที่ Topic/AppID/SmartLight1/Status
      led_color(1024,0,0);                                            //และทำการเปลี่ยนสีของ LED ให้เป็นสีแดง
    }
    microgear.loop();
  } 
  else 
  {
    led_color(0,0,0);                                                 //ถ้าเชื่อมต่อไม่ได้ กำหนดให้ LED ดับ
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
  }
  delay(100);
}
