
#include <Arduino.h>
#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "SoftwareSerial.h"
#include <BH1750.h>
#include "Chinese_calendar.h"
#include "img.h"
#include <TridentTD_LineNotify.h>

#define PIR_PIN                   D3
#define CHANGE_DISPLAY_PIN        A0
#define BUTTON_SET_PIN            D4

#define MQTT_USER                 "my_name"               // unused
#define MQTT_PASSWORD             "my_password"           // unused
#define MQTT_RECONNECT_INTERVAL   100                     // millisecond
#define MQTT_LOOP_INTERVAL        100                     // millisecond
#define MQTT_SUBSCRIBE_ButtonR    "xxx/ButtonR"           //訂閱 智能插座控制主題        xxx為電話末3碼
#define MQTT_SUBSCRIBE_PIR_Set    "xxx/MQTTBoxPIR_Set"    //訂閱 MQTTBox PIR 啟動
#define MQTT_SUBSCRIBE_PM25_Set   "xxx/MQTTBoxPM25_Set"   //訂閱 MQTTBox PM25 啟動
#define MQTT_SUBSCRIBE_PM25_Value "xxx/MQTTBoxPM25_Value" //訂閱 MQTTBox Lux 數值
#define MQTT_SUBSCRIBE_Lux_Set    "xxx/MQTTBoxLux_Set"    //訂閱 MQTTBox Lux 啟動
#define MQTT_SUBSCRIBE_Lux_Value  "xxx/MQTTBoxLux_Value"  //訂閱 MQTTBox Lux 數值
#define MQTT_PUBLISH_ButtonT      "xxx/ButtonT"           //發佈 智能插座狀態主題
#define MQTT_PUBLISH_Alarm        "IIoT4Device4/AlarmR"   //發佈 智能智能閃燈主題
#define MQTT_PUBLISH_ButtonR      "xxx/ButtonR"           //發佈 智能插座控制主題
#define MQTT_PUBLISH_PM25ButtonR  "xxx/PM25ButtonR"       //發佈 PM25智能插座控制主題
#define MQTT_PUBLISH_MQTTBoxLux   "xxx/MQTTBoxLux"        //發佈 空氣盒子 Lux 主題
#define MQTT_PUBLISH_MQTTBoxPIR   "xxx/MQTTBoxPir"        //發佈 空氣盒子 PIR 主題
#define MQTT_PUBLISH_MQTTBoxTemp  "xxx/MQTTBoxTemp"       //發佈 空氣盒子 溫度 主題
#define MQTT_PUBLISH_MQTTBoxPIR   "xxx/MQTTBoxHumi"       //發佈 空氣盒子 濕度 主題
#define MQTT_PUBLISH_MQTTBoxPM25  "xxx/MQTTBoxPM25"       //發佈 空氣盒子 PM2.5 主題
#define MQTT_PUBLISH_MQTTBoxT     "xxx/MQTTBoxLive"       //發佈 空氣盒子 Live 主題
#define MQTT_PUBLISH_MQTTBox      "xxx/MQTTBox"           //發佈 空氣盒子 整個JSON數值

#define LINE_NOTIFY_TOKEN         "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"       

// WPA/WPA2 SSID and password
char ssid2[] = "**********";      // your network SSID (name)
char pass2[] = "**********";      // your network password
char ssid1[] = "**********";      // your network SSID (name)
char pass1[] = "***********;     // your network password
int status  = WL_IDLE_STATUS;     // the Wifi radio's status
char mqtt_server[] = "broker.emqx.io";
const unsigned int   mqtt_port = 1883;
//--------------------------------------thingspeak---------------------------------------------------
const char* apiKeyTS = "******************";                  // 請更換成 Thing Speak WRITE API KEY
const char* resource = "/update?api_key=";
const char* ThinkSpeakServer = "api.thingspeak.com";          // Thing Speak API server
//----------------------------- openweathermap----------------------------------
//open weather map api key
String apiKey = "b3a1fd4de2403e47a564135a5be59821";
//the city you want the weather for
String location = "Taipei,TW";
char server[] = "api.openweathermap.org";
String weatherFlag;
unsigned long weatherTimeoutStart;
int weatherTimeoutCounter = 0;
String weatherStr, nowTemp, maxTemp, minTemp, speedM;
int weatherKind;
char dayType = '1', dayTypeR = '1';
//-------------------------------- NTP Server setup-------------------------------------------
int showFlag = 1;
unsigned int localPort = 2390; //local port to listen for UDP packets
IPAddress timeServerIP;    //time.nist.gov NTP server address
const char* ntpServerName = "time.google.com"; //NTP Server host name
const int NTP_PACKET_SIZE = 48; //NTP timestamp resides in the first 48 bytes of packets
byte packetBuffer[ NTP_PACKET_SIZE];  //buffer to hold incoming and outgoing packets
WiFiUDP udp;  //UDP instance to let us send and receive packets over UDP

void mqtt_callback(char* topic, byte* payload, unsigned int msgLength);  // MQTT Callback function header
WiFiClient wifiClient , ThingSpeakClient;
PubSubClient mqttClient(mqtt_server, mqtt_port, mqtt_callback, wifiClient);
String mqttValue;
long lastTime = 0 ;
int alarmPIRFlag = 0 , alarmLuxFlag = 0 , alarmPM25Flag = 0; //設定值啟動
int pressPIR = 0 , pressLux = 0 , pressPM25 = 0;

unsigned long WifiConnectingTimeout; //連線逾時時間
char *weekdayStr[7] = {"日", "一", "二", "三", "四", "五", "六"};

int displayValue = 0;
int unlockKey = 0;                   // 按鈕按下狀態設定
String MQTT_Lux_Value = "10" , MQTT_PM25_Value = "10" ;

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//------------------------------json6 object-------------------------------------------------------------
char json_output[1000];

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1327_MIDAS_128X128_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  /* Uno: A4=SDA, A5=SCL, add "u8g2.setBusClock(400000);" into setup() for speedup if possible */

int errorFlag = 0 ;
long pmat10 = 0;
long pmat25 = 0;
long pmat100 = 0;
unsigned int temperature = 0;
unsigned int humandity = 0;
unsigned long moveDelayTime;
float luxValue;
int alarmMove = 0 , alarmSet = 0 , sensorTransSet = 0 , luxSet = 0 , pm25Set = 0;
String MQTTBox_Setup ;

#define SUN  0
#define SUN_CLOUD  1
#define CLOUD 2
#define RAIN 3
#define THUNDER 4

BH1750 lightMeter;
SoftwareSerial SerialSensor(D7, D8); // RX, TX

void drawWeatherSymbol(u8g2_uint_t x, u8g2_uint_t y, uint8_t symbol)
{
  // fonts used:
  // u8g2_font_open_iconic_embedded_6x_t
  // u8g2_font_open_iconic_weather_6x_t
  // encoding values, see: https://github.com/olikraus/u8g2/wiki/fntgrpiconic

  switch (symbol)
  {
    case SUN:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 69);
      break;
    case SUN_CLOUD:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 65);
      break;
    case CLOUD:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 64);
      break;
    case RAIN:
      u8g2.setFont(u8g2_font_open_iconic_weather_6x_t);
      u8g2.drawGlyph(x, y, 67);
      break;
    case THUNDER:
      u8g2.setFont(u8g2_font_open_iconic_embedded_6x_t);
      u8g2.drawGlyph(x, y, 67);
      break;
  }
}

void drawWeather(uint8_t symbol, int degree)
{
  drawWeatherSymbol(0, 48, symbol);
  u8g2.setFont(u8g2_font_logisoso24_tf);
  u8g2.setCursor(60, 52);
  u8g2.print(degree);
  u8g2.print("°C");    // requires enableUTF8Print()
}

boolean mqtt_nonblock_reconnect() {
  boolean doConn = false;

  if (! mqttClient.connected()) {
    // attempts to reconnect every [MQTT_RECONNECT_INTERVAL] milliseconds without blocking the main loop.
    long currTime = millis();
    if (currTime - lastTime > MQTT_RECONNECT_INTERVAL) {
      lastTime = currTime;
      doConn = true;
      boolean isConn = mqttClient.connect(clientId);
      //boolean isConn = mqttClient.connect(clientId, MQTT_USER, MQTT_PASSWORD);
      char logConnected[100];
      sprintf(logConnected, "[%s] Connect %s !", clientId, (isConn ? "Successful" : "Failed"));
      Serial.println(logConnected);

      if (isConn) {

        mqttClient.subscribe(MQTT_SUBSCRIBE_PIR_Set);    //訂閱 MQTTBox PIR 啟動
        mqttClient.subscribe(MQTT_SUBSCRIBE_Lux_Set);    //訂閱 MQTTBox Lux 啟動
        mqttClient.subscribe(MQTT_SUBSCRIBE_Lux_Value);  //訂閱 MQTTBox Lux 數值
        mqttClient.subscribe(MQTT_SUBSCRIBE_PM25_Set);   //訂閱 MQTTBox PM25 啟動
        mqttClient.subscribe(MQTT_SUBSCRIBE_PM25_Value); //訂閱 MQTTBox PM25 數值
      }
    }
  }

  return doConn;
}


void mqtt_callback(char* topic, byte* payload, unsigned int msgLength) {

  Serial.printf("Message arrived with Topic [%s]\n  Data Length: [%d], Payload: [", topic, msgLength);
  Serial.write(payload, msgLength);
  Serial.println("]");

  if (strcmp(MQTT_SUBSCRIBE_PIR_Set, topic) == 0)         //智能插座控制主題
  {
    MQTTBox_Setup = "";
    for (int i = 0; i < msgLength; i++) {
      MQTTBox_Setup += (char)payload[i];
    }      //  Payload: [On]
    if ( MQTTBox_Setup == "On" )
      pressPIR = 1;
    else
      pressPIR = 0;
  }
  if (strcmp(MQTT_SUBSCRIBE_Lux_Set, topic) == 0)         //智能插座控制主題
  {
    MQTTBox_Setup = "";
    for (int i = 0; i < msgLength; i++) {
      MQTTBox_Setup += (char)payload[i];
    }      //  Payload: [On]
    if ( MQTTBox_Setup == "On" )
      pressLux = 1;
    else
      pressLux = 0;
  }
  if (strcmp(MQTT_SUBSCRIBE_PM25_Set, topic) == 0)         //智能插座控制主題
  {
    MQTTBox_Setup = "";
    for (int i = 0; i < msgLength; i++) {
      MQTTBox_Setup += (char)payload[i];
    }      //  Payload: [On]
    if ( MQTTBox_Setup == "On" )
      pressPM25 = 1;
    else
      pressPM25 = 0;
  }
  if (strcmp(MQTT_SUBSCRIBE_Lux_Value, topic) == 0)         //智能插座控制主題
  {
    MQTTBox_Setup = "";
    for (int i = 0; i < msgLength; i++) {
      MQTTBox_Setup += (char)payload[i];
    }      //  Payload: 數值
    MQTT_Lux_Value = MQTTBox_Setup;

  }
  if (strcmp(MQTT_SUBSCRIBE_PM25_Value, topic) == 0)         //智能插座控制主題
  {
    MQTTBox_Setup = "";
    for (int i = 0; i < msgLength; i++) {
      MQTTBox_Setup += (char)payload[i];
    }      //  Payload: 數值
    MQTT_PM25_Value = MQTTBox_Setup;

  }
}



void setup(void) {

  int getLoop = 0 ;

  pinMode(PIR_PIN, INPUT);              //人體感測
  pinMode(CHANGE_DISPLAY_PIN, INPUT);   //切換畫面
  pinMode(BUTTON_SET_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_SET_PIN), deBounce, RISING );  //設定 Interrupt 觸發 與 觸發時執行 function

  Serial.begin(115200);
  SerialSensor.begin(9600);

  Wire.begin();
  lightMeter.begin();

  u8g2.begin();
  u8g2.enableUTF8Print();



  u8g2.setFont(u8g2_font_unifont_t_chinese1); //使用我們做好的字型
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 70);
    u8g2.drawXBMP(0, 32, 128, 64 , gImage_MQTT_C);
  } while ( u8g2.nextPage() );
  delay(1500);
  u8g2.setFont(u8g2_font_unifont_t_chinese1); //使用我們做好的字型
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 70);
    u8g2.print("連線中,請稍後...");

  } while ( u8g2.nextPage() );
  WifiConn();


  if ( WiFi.status() == WL_CONNECTED ) {

    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 70);
      u8g2.print("連線完成!!");
    } while ( u8g2.nextPage() );
    udp.begin(localPort);
    delay(1000);
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 70);
      u8g2.print("抓取網路天氣預報");
    } while ( u8g2.nextPage() );
    getNtpTime();
    getWeather();
    while ( nowTemp.toInt() == 0 ) // 若沒抓取到溫度,連續抓取5次
    {
      ++getLoop ;
      getWeather();
      if ( getLoop > 5)
      {
        Serial.println("getWeather error!!");
        break ;
      }

    }

    Serial.println("WiFi connected");
    Serial.println(WiFi.localIP());

  }
  else
  {
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 70);
      u8g2.print("連線失敗!!");
    } while ( u8g2.nextPage() );
    Serial.println("no network !!   ");
  }
  Convert2Luner(year(), month(), day()); //轉換農曆
}

void loop(void)
{
  int cut_apart = 5; //旋鈕切割幾等分顯示
  int posValue = 0;  //目前旋鈕位置
  int getLoop = 0 ;

  mqtt_nonblock_reconnect();

  mqttClient.loop();
  delay(MQTT_LOOP_INTERVAL);

  if ( second() % 3 == 0 )
  {
    luxValue = lightMeter.readLightLevel();
    mqttClient.publish(MQTT_PUBLISH_MQTTBoxT,"Live");
    
    retrievePmValue();             //抓取 PMS5003T 感測值
    while ( errorFlag )             //若抓回數值有誤,延遲2秒重新抓取
    {
      retrievePmValue();
      delay(2000);
    }
    

  }
  posValue = analogRead(A0);     //旋鈕刻度
  displayValue = map(posValue, 0, 1024, 1, cut_apart); // 從0-1024中旋鈕位置值,來對映幾等分的位置

  
  if ( second() % 29 == 0 )              //每29秒 傳送一次 MQTT JSON
  {
    Serial.println("傳輸到 ThingSpeak");
    post2ThinkSpeak(String(temperature), String(humandity), String(pmat25), String(pmat10), String(lightMeter.readLightLevel()), String(alarmMove));
    Serial.println("Publish to MQTT Broker");
    publishPMS_Value() ;
    
  }
  if ((minute() == 30 || minute() == 0 ) && second() == 0  )
  {
    getWeather();
    while ( nowTemp.toInt() == 0 ) // 若沒抓取到溫度,連續抓取5次
    {
      ++getLoop ;
      getWeather();
      if ( getLoop > 5)
      {
        Serial.println("getWeather error!!");
        break ;
      }

    }
  }

  if (digitalRead(D3)) // GPIO0 偵測有無人移動
  {
    alarmMove = 1;
    if (alarmSet)
    {
      if ( !alarmPIRFlag  )
      {
        mqttClient.publish(MQTT_PUBLISH_Alarm, "On");  //送出 On 狀態
        Serial.println("Publish Alarm On");

        LINE.setToken(LINE_NOTIFY_TOKEN);
        LINE.notifyPicture( "有人異常入侵" , "https://od.lk/s/MTVfMjU0NDEwNDVf/tgvUJZGfQB.jpg");
        alarmPIRFlag = 1 ;
      }

    }
  }
  else
  {
    alarmMove = 0;
    if ( alarmPIRFlag )
    {
      mqttClient.publish(MQTT_PUBLISH_Alarm, "Off");  //送出 Off 狀態
      Serial.println("Publish Alarm Off");
      alarmPIRFlag = 0;
    }
  }

  if ( luxValue < MQTT_Lux_Value.toInt() )
  {
    if ( luxSet )
    {
      if ( !alarmLuxFlag )
      {
        mqttClient.publish(MQTT_PUBLISH_ButtonR, "On");  // 打開智能插座送出
        Serial.println("Publish Light On");

        LINE.setToken(LINE_NOTIFY_TOKEN);
        LINE.notifyPicture( "亮度不足已開啟大燈" , "https://od.lk/s/MTVfMjU0NDExMjhf/light_On.png");
        alarmLuxFlag = 1 ;
      }
    }
  }
  else
  {
    if ( alarmLuxFlag )
    {
      mqttClient.publish(MQTT_PUBLISH_ButtonR, "Off");  //送出 Off 狀態
      Serial.println("Publish Light Off");
      alarmLuxFlag = 0;
    }
  }

  if ( pmat25 > MQTT_PM25_Value.toInt() )
  {
    if ( pm25Set )
    {
      if ( !alarmPM25Flag )
      {
        mqttClient.publish(MQTT_PUBLISH_PM25ButtonR, "On");  // 打開智能插座送出
        Serial.println("Publish PM2.5 Switch On");

        LINE.setToken(LINE_NOTIFY_TOKEN);
        LINE.notifyPicture( "空氣不佳已開清淨機" , "https://od.lk/s/MTVfMjU0NDMxODdf/Logo-1535870217.jpg");
        alarmPM25Flag = 1 ;
      }
    }
  }
  else
  {
    if ( alarmPM25Flag )
    {
      mqttClient.publish(MQTT_PUBLISH_PM25ButtonR, "Off");  //送出 Off 狀態
      Serial.println("Publish PM2.5 Switch Off");
      alarmPM25Flag = 0;
    }
  }

  switch (displayValue)
  {
    case 1 :
      displayArea_A();
      break;
    case 2 :
      displayArea_B();
      break;
    case 3 :
      displayArea_C();
      break;
    case 4 :
      displayArea_D();
      break;
    case 5 :
      displayArea_E();
      break;
  }

}

void displayArea_A()
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawFrame(0, 0, 128, 128);
    u8g2.setCursor(10, 20);
    u8g2.print(year());
    u8g2.setCursor(95, 20);
    u8g2.print(month());
    u8g2.setFont(u8g2_font_osb26_tf);
    u8g2.setCursor(45, 70);
    u8g2.print(day());
    u8g2.setFont(u8g2_font_unifont_t_chinese1);
    u8g2.setCursor(15, 90);
    u8g2.print( "星期" + String( *(weekdayStr + weekday() - 1) ) ) ;
    u8g2.setCursor(30, 118);
    u8g2.print("農曆: ");
    u8g2.setFont(u8g2_font_logisoso16_tf);
    u8g2.setCursor(65, 120);
    u8g2.print(String(lunarMon) + "/" + String(lunarDay));
    u8g2.setFont(u8g2_font_helvB12_tn);
    u8g2.setCursor(80, 90);
    u8g2.print(String(hour()) + ":" + String(minute()));
    u8g2.drawHLine(10, 30, 108);

  } while ( u8g2.nextPage() );
}

void displayArea_B()
{

  u8g2.firstPage();
  do {
    u8g2.drawFrame(0, 0, 128, 128);
    if ( sensorTransSet )
      u8g2.drawDisc(120, 120, 4);

    u8g2.setFont(u8g2_font_logisoso16_tf);
    u8g2.setCursor(5, 105);
    u8g2.print(String(temperature));
    u8g2.print("°C ");   // requires enableUTF8Print()
    u8g2.print(String(humandity));
    u8g2.print("% ");
    u8g2.print(String(int(luxValue)));
    u8g2.setCursor(5, 123);
    u8g2.print("PM2.5:  " );
    u8g2.print(String(int(pmat25)));
    u8g2.setCursor(50, 16);
    u8g2.print(String(hour()) + ":" + String(minute()));

    u8g2.setFont(u8g2_font_unifont_t_chinese1);
    u8g2.setCursor(5, 85);
    u8g2.print("室內溫濕照度:");
    u8g2.drawHLine(0, 65, 128);
    drawWeather(weatherKind, nowTemp.toInt());
    u8g2.setFont(u8g2_font_unifont_t_chinese1);


    u8g2.setCursor(108, 16);
    u8g2.print(String( *(weekdayStr + weekday() - 1) ));
  } while ( u8g2.nextPage() );
}

void displayArea_C()
{

  if (pressPIR)
    alarmSet = 1 ;
  else
    alarmSet = 0 ;


  u8g2.firstPage();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);
  do {

    if ( alarmSet )
    {
      u8g2.drawXBMP(0, 0, 128, 128 , gImage_fireWall);
      if ( alarmMove )
      {
        u8g2.setCursor(10, 45);
        u8g2.print("有人異常入侵");
      }
      else
      {
        u8g2.setCursor(30, 45);
        u8g2.print("無異狀");
      }
    }
    else
    {
      u8g2.setCursor(30, 120);
      u8g2.print("關閉警戒");
    }
  } while ( u8g2.nextPage() );
}

void displayArea_D()
{
  if (pressLux)
    luxSet = 1 ;
  else
    luxSet = 0 ;


  u8g2.firstPage();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);
  do {

    if ( luxSet )
    {

      u8g2.drawXBMP(0, 0, 128, 128 , gImage_light);
      u8g2.setCursor(10, 10);
      u8g2.print(MQTT_Lux_Value);
      if (luxValue <  MQTT_Lux_Value.toInt())
      {
        
        u8g2.setCursor(30, 50);
        u8g2.print("已開燈");
      }
      else
      {
        u8g2.setCursor(30, 50);
        u8g2.print("亮度足夠");
      }
    }
    else
    {
      u8g2.setFont(u8g2_font_t0_17_tf);
      u8g2.setCursor(5, 30);
      u8g2.print("Lux: ");
      u8g2.print(String(luxValue));
      u8g2.setCursor(5, 65);
      u8g2.print("MQTT Set> ");
      u8g2.print(MQTT_Lux_Value);
      u8g2.setFont(u8g2_font_unifont_t_chinese1);
      u8g2.setCursor(20, 120);
      u8g2.print("關閉亮度設定");
    }
  } while ( u8g2.nextPage() );
}

void displayArea_E()
{
  if (pressPM25)
    pm25Set = 1 ;
  else
    pm25Set = 0 ;

  u8g2.firstPage();
  u8g2.setFont(u8g2_font_unifont_t_chinese1);
  do {

    if ( pm25Set )
    {

      u8g2.drawXBMP(0, 58, 128, 70 , gImage_PM25);
      if ( pmat25 >  MQTT_PM25_Value.toInt())
      {
        //u8g2.drawDisc(120, 120, 4);
        u8g2.setCursor(20, 40);
        u8g2.print("已開清淨機");
      }
      else
      {
        u8g2.setCursor(30, 40);
        u8g2.print("空氣佳");
      }
    }
    else
    {
      u8g2.setFont(u8g2_font_t0_17_tf);
      u8g2.setCursor(5, 30);
      u8g2.print("PM2.5: ");
      u8g2.print(String(pmat25));
      u8g2.setCursor(5, 65);
      u8g2.print("MQTT Set< ");
      u8g2.print(MQTT_PM25_Value);
      u8g2.setFont(u8g2_font_unifont_t_chinese1);
      u8g2.setCursor(20, 120);
      u8g2.print("關閉空氣設定");
    }
  } while ( u8g2.nextPage() );
}

/*-------- NTP code ----------*/

//const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
//byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime() {

  unsigned long GMT;

  WiFi.hostByName(ntpServerName, timeServerIP);  //get a random server from the pool
  sendNTPpacket(timeServerIP);   //send an NTP packet to a time server
  delay(1000);   // wait to see if a reply is available

  int cb = udp.parsePacket();   //return bytes received
  unsigned long unix_time = 0;  //預設傳回 0, 表示未收到 NTP 回應
  if (!cb) {
    Serial.println("no packet yet");
  }
  else {  //received a packet, read the data from the buffer
    Serial.print("packet received, length=");
    Serial.println(cb);    //=48
    udp.read(packetBuffer, NTP_PACKET_SIZE);  //read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    //or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    //combine the four bytes (two words) into a long integer
    //this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900=" );
    Serial.println(secsSince1900);
    Serial.print("Unix time=");
    //Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    unix_time = secsSince1900 - 2208988800UL;  //更新 unix_time
    Serial.print(F("Unix time stamp (seconds since 1970-01-01)="));
    Serial.println(unix_time); //print Unix time
  }
  //return unix_time; //return seconds since 1970-01-01
  GMT = unix_time ;
  if (GMT != 0) {   //有得到 NTP 回應才更新 ESP8266 內建 RTC
    setTime(GMT + 28800L);  //以台灣時間更新內部時鐘
  }
  udp.begin(localPort);

}

unsigned long sendNTPpacket(IPAddress & address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  //Initialize values needed to form NTP request
  //(see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;   // Stratum, or type of clock
  packetBuffer[2] = 6;   // Polling Interval
  packetBuffer[3] = 0xEC; // Peer Clock Precision
  //8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
time_t getWeather() {

  String jsonTmp ;
  wifiClient.stop();
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (wifiClient.connect(server, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    wifiClient.print("GET /data/2.5/weather?");
    wifiClient.print("q=" + location);
    wifiClient.print("&appid=" + apiKey);
    wifiClient.print("&cnt=3");
    wifiClient.println("&units=metric");
    wifiClient.println("Host: api.openweathermap.org");
    wifiClient.println("Connection: close");
    wifiClient.println();
    wifiClient.flush();
  } else {
    Serial.println("unable to connect");
  }

  delay(500);
  while (wifiClient.available())
  {
    DynamicJsonDocument doc(5000);

    String jsonData = wifiClient.readStringUntil('\r');  //逐列讀取
    Serial.println(jsonData);
    DeserializationError error = deserializeJson(doc, jsonData);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      //return;
    }
    //JsonArray array = doc.as<JsonArray>();

    String Str1 = doc["main"]["temp"];
    String Str2 = doc["main"]["temp_min"];
    String Str3 = doc["main"]["temp_max"];
    String Str4 = doc["weather"][0]["main"];

    nowTemp = Str1;
    minTemp = Str2;
    maxTemp = Str3;
    weatherStr = Str4;

    Serial.print("目前溫度: ");
    Serial.println(nowTemp);
    Serial.print("最低溫度: ");
    Serial.println(minTemp);  //顯示溫度值
    Serial.print("最高溫度: ");
    Serial.println(maxTemp);  //顯示溫度值
    Serial.print("天氣狀況: ");
    Serial.println(weatherStr);

    if ( weatherStr == "Clear"  )
      weatherKind = 0;
    if ( weatherStr == "Clouds" )
      weatherKind = 2;
    if ( weatherStr == "Rain" )
      weatherKind = 3;
    if ( weatherStr == "Drizzle" )
      weatherKind = 3;
    if ( weatherStr == "Thunderstorm" )
      weatherKind = 4 ;
    Serial.print("weatherKind :");
    Serial.println(weatherKind);

  }
}




int WifiConn()
{
  Serial.print("Connecting to ");
  Serial.println(ssid1);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */

  WiFi.begin(ssid1, pass1);
  WifiConnectingTimeout = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid1);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    //WiFi.begin(ssid1, pass1);
    if ((millis() - WifiConnectingTimeout) > 10000) //等待 10秒
      break;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    WifiConnectingTimeout = millis();
    WiFi.begin(ssid2, pass2);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid2);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:

      if ((millis() - WifiConnectingTimeout) > 10000) //等待 10秒
        break;
      delay(500);
      Serial.print(".");
    }
  }

}

void retrievePmValue() {           //取出 PMS5003T 相關數值 函式

  int count = 0;
  unsigned char c;
  unsigned char high;
  while (SerialSensor.available()) {
    c = SerialSensor.read();
    if ((count == 0 && c != 0x42) || (count == 1 && c != 0x4d)) {
      SerialSensor.println("check failed");
      errorFlag = 1 ;
      break;
    }
    if (count > 27) {
      Serial.println("PMS Data Get complete");
      errorFlag = 0 ;
      break;
    }
    else if (count == 10 || count == 12 || count == 14 || count == 24 || count == 26) {
      high = c;
    }
    else if (count == 11) {
      pmat10 = 256 * high + c;
      //Serial.print("PM1.0=");
      //Serial.print(pmat10);
      //Serial.println(" ug/m3");
    }
    else if (count == 13) {
      pmat25 = 256 * high + c;
      //Serial.print("PM2.5=");
      //Serial.print(pmat25);
      //Serial.println(" ug/m3");
    }
    else if (count == 15) {
      pmat100 = 256 * high + c;
      //Serial.print("PM10=");
      //Serial.print(pmat100);
      //Serial.println(" ug/m3");
    }
    else if (count == 25) {
      temperature = (256 * high + c) / 10;
      //Serial.print("Temp=");
      //Serial.print(temperature);
      //Serial.println(" (C)");
    }
    else if (count == 27) {
      humandity = (256 * high + c) / 10;
      //Serial.print("Humidity=");
      //Serial.print(humandity);
      //Serial.println(" (%)");
    }
    count++;
  }
  while (SerialSensor.available()) SerialSensor.read();
}

void PMSpassiveMode()         //PMS5003T passiveMode
{
  uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x70 };
  SerialSensor.write(command, sizeof(command));
}
void PMSwake()                //打開 PMS5003T 風扇
{
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };
  SerialSensor.write(command, sizeof(command));
}
void PMSsleep()               //關閉 PMS5003T 風扇
{
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };
  SerialSensor.write(command, sizeof(command));
}

ICACHE_RAM_ATTR void deBounce()
{
  boolean state;
  boolean previousState;
  int bounceDelay = 5; // 設定連續 5 毫秒狀態相同時，判定為穩定
  String keyStr ;

  previousState = digitalRead(BUTTON_SET_PIN);  // 儲存按鈕狀態

  for (int counter = 0; counter < bounceDelay; counter++)
  {
    //delay(1);
    state = digitalRead(BUTTON_SET_PIN);        // 再讀取一次按鈕狀態
    if (state != previousState)      // 如果兩次狀態不同，把 counter 設為零，重新確認狀態
    {
      counter = 0;
      previousState = state;
    }
  }
  // for 迴圈每次都會 delay 1 毫秒，當每次讀取狀態都與第一次相同時，整個 for 跑完 5 毫秒，與之前設定的 bounceDelay 時間相同，回傳穩定的按鈕狀態
  if ( state )
  {

    switch (displayValue)
    {
      case 3 :
        pressPIR = !pressPIR;
        keyStr = "PIR";
        break;
      case 4 :
        pressLux = !pressLux;
        keyStr = "Lux";
        break;
      case 5 :
        pressPM25 = !pressPM25;
        keyStr = "PM25";
        break;

    }
    Serial.print("Press Button unlockKey= ");
    Serial.println(keyStr);
  }
}
void publishPMS_Value()          //Publish PMS5003T MQTT JSON Value
{

  String Buff   ;
  DynamicJsonDocument json_doc(1000);

  Buff += temperature ;          //數值轉為 String
  json_doc["Temp"] = Buff;

  Buff = "";
  Buff += humandity ;
  json_doc["Humi"] = Buff;

  Buff = "";
  Buff += pmat10 ;
  json_doc["PM10"] = Buff;

  Buff = "";
  Buff += pmat25 ;
  json_doc["PM25"] = Buff;

  Buff = "";
  Buff += pmat100 ;
  json_doc["PM100"] = Buff;

  Buff = "";
  Buff += lightMeter.readLightLevel();
  json_doc["luxValue"] = Buff;

  Buff = "";
  Buff += alarmMove;
  json_doc["PIR"] = Buff;

  serializeJson(json_doc, json_output);
  Serial.print("JSON Str: ");
  Serial.println(json_output);
  mqttClient.publish(MQTT_PUBLISH_MQTTBox, json_output);
}

void post2ThinkSpeak(String fieldValue1, String fieldValue2, String fieldValue3, String fieldValue4, String fieldValue5, String fieldValue6) {

  ThingSpeakClient.stop();
  if (ThingSpeakClient.connect(ThinkSpeakServer, 80)) {
    Serial.println(F("connected"));
  }
  else  {
    Serial.println(F("connection failed"));
    return;
  }

  Serial.print("Request resource: ");
  Serial.println(resource);

  ThingSpeakClient.print(String("GET ") + resource + apiKeyTS + "&field1=" + fieldValue1 + "&field2=" + fieldValue2 + "&field3=" + fieldValue3 +
                   "&field4=" + fieldValue4 + "&field5=" + fieldValue5 + "&field6=" + fieldValue6 +
                   " HTTP/1.1\r\n" +
                   "Host: " + ThinkSpeakServer + "\r\n" +
                   "Connection: close\r\n\r\n");
  int timeout = 5 * 10; // 5 seconds
  while (!ThingSpeakClient.available() && (timeout-- > 0)) {
    delay(100);
  }

  if (!ThingSpeakClient.available()) {
    Serial.println("No response, going back to sleep");
  }

  while (ThingSpeakClient.available()) {
    Serial.write(ThingSpeakClient.read());
  }

  Serial.println("\nclosing connection");
  ThingSpeakClient.stop();
}
