#include <WiFi.h>
#include <PubSubClient.h> //請先安裝PubSubClient程式庫
#include <SimpleDHT.h>
#include <ESP32Servo.h>

//wifi and password
char ssid[] = "Howhowisme";
char password[] = "howhowisme";

//pin set of temp and humi
int pinDHT11 = 23;//DHT11
SimpleDHT11 dht11(pinDHT11);

//pin set of LED
int pinBLED = 15;
int pinGLED = 2;
int pinRLED = 4;

//pin set of servo
Servo ServoHead;
Servo ServoLArm;
Servo ServoRArm;
Servo ServoFood;
int pinServoH = 32;
int pinServoL = 25;
int pinServoR = 26;
int pinServoF = 33;

int Trig = 12; //發出聲波
int Echo = 14; //接收聲波

//mqtt setting
char* MQTTServer = "mqttgo.io";//免註冊MQTT伺服器
int MQTTPort = 1883;//MQTT Port
char* MQTTUser = "";//不須帳密
char* MQTTPassword = "";//不須帳
//t h
char* MQTTPubTopic1 = "Final/minion/temp";
char* MQTTPubTopic2 = "Final/minion/humi";
//sonic
char* MQTTPubTopic3 = "Final/minion/sonic";

//led
char* MQTTSubTopicR = "Final/minion/Rled";
char* MQTTSubTopicG = "Final/minion/Gled";
char* MQTTSubTopicB = "Final/minion/Bled";

char* MQTTSubTopicServoHead = "Final/minion/ServoHead";
char* MQTTSubTopicServoLArm = "Final/minion/ServoLArm";
char* MQTTSubTopicServoRArm = "Final/minion/ServoRArm";
char* MQTTSubTopicServoFood = "Final/minion/ServoFood";
char* MQTTSubTopicServoPlay = "Final/minion/ServoPlay";

//推撥
long MQTTLastPublishTime;//此變數用來記錄推播時間
long MQTTPublishInterval = 4000;//每10秒推撥一次
WiFiClient WifiClient;
PubSubClient MQTTClient(WifiClient);

void setup() {
  Serial.begin(115200);
  
  //LED燈
  pinMode(pinBLED, OUTPUT);
  pinMode(pinGLED, OUTPUT);
  pinMode(pinRLED, OUTPUT);

  //Servo pinmode 設定
  ServoHead.attach(pinServoH, 500, 2400);
  ServoLArm.attach(pinServoL, 500, 2400);
  ServoRArm.attach(pinServoR, 500, 2400);
  ServoFood.attach(pinServoF, 500, 2400);
   //sonic
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);

  //開始WiFi MQTT連線
  WifiConnecte();
  MQTTConnecte();
}

void loop() {
  //如果WiFiMQTT連線中斷，則重啟WiFiMQTT連線
  if (WiFi.status() != WL_CONNECTED) WifiConnecte(); 
  if (!MQTTClient.connected()) MQTTConnecte(); 

  //如果距離上次傳輸已經超過10秒，則Publish溫溼度
  if ((millis() - MQTTLastPublishTime) >= MQTTPublishInterval ) {
    //讀取溫濕度
    byte temperature = 0;
    byte humidity = 0;
    byte precentage = 0;
    ReadDHT(&temperature, &humidity);
    ReadFOOD(&precentage);

    // ------ 將DHT11溫度送到MQTT主題 ------
    MQTTClient.publish(MQTTPubTopic1, String(temperature).c_str());//string to char字元陣列 
    MQTTClient.publish(MQTTPubTopic2, String(humidity).c_str());
    MQTTClient.publish(MQTTPubTopic3, String(precentage).c_str());
    Serial.println("溫溼度已推播到MQTT Broker");
    MQTTLastPublishTime = millis(); //更新最後傳輸時間
  }
  MQTTClient.loop();//更新訂閱狀態
  delay(50);
}

//讀取DHT11溫濕度
void ReadDHT(byte *temperature, byte *humidity) {
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(temperature, humidity, NULL)) !=
      SimpleDHTErrSuccess) {
    Serial.print("讀取失敗,錯誤訊息="); 
    Serial.print(SimpleDHTErrCode(err));
    Serial.print(","); 
    Serial.println(SimpleDHTErrDuration(err)); 
    delay(1000);
    return;
  }
  Serial.print("DHT讀取成功：");
  Serial.print((int)*temperature); 
  Serial.print(" *C, ");
  Serial.print((int)*humidity); 
  Serial.println(" H");
}

void ReadFOOD(byte *precentage){
  digitalWrite(Trig, LOW); //關閉
  delayMicroseconds(5);
  digitalWrite(Trig, HIGH);//啟動
  delayMicroseconds(10);
  digitalWrite(Trig, LOW); //關閉
  float EchoTime = pulseIn(Echo, HIGH); //傳回時間
  float CMValue = EchoTime / 29.4 / 2; //轉換成距離
  Serial.println(CMValue);
  float full = 8;
  float Nothing = 17;
  //float CMValue = 63;
  *precentage = ((Nothing - CMValue) / (Nothing - full) * 100);
}

//開始WiFi連線
void WifiConnecte() {
  //開始WiFi連線
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi連線成功");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());
}

//開始MQTT連線
void MQTTConnecte() {
  MQTTClient.setServer(MQTTServer, MQTTPort);
  MQTTClient.setCallback(MQTTCallback);
  while (!MQTTClient.connected()) {
    //以亂數為ClietID
    String  MQTTClientid = "esp32-" + String(random(1000000, 9999999));
    if (MQTTClient.connect(MQTTClientid.c_str(), MQTTUser, MQTTPassword)) {
      //連結成功，顯示「已連線」。
      Serial.println("MQTT已連線");
      //訂閱SubTopic1主題
      MQTTClient.subscribe(MQTTSubTopicR);
      MQTTClient.subscribe(MQTTSubTopicG);
      MQTTClient.subscribe(MQTTSubTopicB);
      MQTTClient.subscribe(MQTTSubTopicServoHead);
      MQTTClient.subscribe(MQTTSubTopicServoLArm);
      MQTTClient.subscribe(MQTTSubTopicServoRArm);
      MQTTClient.subscribe(MQTTSubTopicServoFood);
      MQTTClient.subscribe(MQTTSubTopicServoPlay);

    } else {
      //若連線不成功，則顯示錯誤訊息，並重新連線
      Serial.print("MQTT連線失敗,狀態碼=");
      Serial.println(MQTTClient.state());
      Serial.println("五秒後重新連線");
      delay(5000);
    }
  }
}

//接收到訂閱時
void MQTTCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print(topic); Serial.print("訂閱通知:");
  String payloadString;//將接收的payload轉成字串
  //顯示訂閱內容
  for (int i = 0; i < length; i++) {
    payloadString = payloadString + (char)payload[i];
  }
  Serial.println(payloadString);
  //比對主題是否為訂閱主題1
  if (strcmp(topic, MQTTSubTopicR) == 0) {
    Serial.println("改變燈號：" + payloadString);
    if (payloadString == "1") digitalWrite(pinRLED, HIGH);
    if (payloadString == "0") digitalWrite(pinRLED, LOW);
  }
  if (strcmp(topic, MQTTSubTopicG) == 0) {
    Serial.println("改變燈號：" + payloadString);
    if (payloadString == "1") digitalWrite(pinGLED, HIGH);
    if (payloadString == "0") digitalWrite(pinGLED, LOW);
  }
  if (strcmp(topic, MQTTSubTopicB) == 0) {
    Serial.println("改變燈號：" + payloadString);
    if (payloadString == "1") digitalWrite(pinBLED, HIGH);
    if (payloadString == "0") digitalWrite(pinBLED, LOW);
  }
  if (strcmp(topic, MQTTSubTopicServoHead) == 0) {
    ServoHead.write( payloadString.toInt());
    Serial.println("H馬達轉到" + payloadString);
  }
  if (strcmp(topic, MQTTSubTopicServoLArm) == 0) {
    ServoLArm.write( payloadString.toInt());
    Serial.println("L馬達轉到" + payloadString);
  }
  if (strcmp(topic, MQTTSubTopicServoRArm) == 0) {
    ServoRArm.write( payloadString.toInt());
    Serial.println("R馬達轉到" + payloadString);
  }
  if (strcmp(topic, MQTTSubTopicServoFood) == 0) {
    if(payloadString == "1"){
      ServoFood.write(45);
      delay(200);
      ServoFood.write(0);
      Serial.println("food deliver");
    }
    
  }

  if (strcmp(topic, MQTTSubTopicServoPlay) == 0) {
    //playing code
  unsigned long startTime = millis(); // 獲取當前時間
  while (millis() - startTime < 5000) { // 持續執行5秒鐘
    int LrandomAngle = random(0, 181); // 隨機生成0到180之間的角度
    ServoLArm.write(LrandomAngle);
    Serial.println(LrandomAngle); 
    int RrandomAngle = random(0, 181); 
    ServoRArm.write(RrandomAngle); 
    Serial.println(RrandomAngle); 
    delay(500);
  }
    Serial.println("playing");
  }

  


}