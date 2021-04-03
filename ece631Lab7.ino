/***************************************************
 2021-03-08
 EC631 ESP32 Hello World
 Board : "NodeMCU-32s"
 ****************************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#define LEDBLUE 2
const char* ssid     = "ece631Lab";
const char* password = "esp32IOT!";
unsigned long LEDMillis;
const int NUMELEMENTS = 5;
String distPayload = "";
int Count;
int samples;
int avgNum;
bool LEDState;
char serial[50];
bool flash;
int rate;
int PWM_FREQUENCY = 16;
int PWM_CHANNEL = 0;
int PWM_RESOLUTION = 8;
int dutyCycle = 1/256;
double sendDist = 0;
double movingArray[NUMELEMENTS];
double movingAvg = 0;
double distance = 0;
double highTime = 0;
double lowTime = 0;
double width = 0;
IPAddress server(192,168,1,159);
DynamicJsonDocument doc(1024);
DynamicJsonDocument temp(1024);

WiFiClient wificlient;
PubSubClient client(wificlient);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  deserializeJson(doc, payload);
  temp = doc;
  rate = doc["Flash"];
}

void reconnect() {
  // Loop until we're reconnected
  client.setServer(server, 1883);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("CruseTaylor")) {
      Serial.println("connected");
      client.subscribe("/ece631/Lab7/Distance/SensorID/0");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// want rise and the width
// pin22 Echo interrupt
void isr(){
 //Serial.println("test");
if(digitalRead(22) == HIGH){
  highTime = micros();
  }
else if(digitalRead(22) == LOW){
  lowTime = micros();
  }

 width = lowTime - highTime;
 
 distance =  ((width / 1000000) * 13503.9) / 2;
}

double sum(double* arr[], int n){
 double sum = 0;

 for(int i = 0; i < n; i++){
 sum = sum + *arr[i]; 
 }
}

void setup() {
 Serial.begin(115200);

 pinMode(23, INPUT_PULLUP);

 int PWM_FREQUENCY = 16;
 int PWM_CHANNEL = 0;
 int PWM_RESOLUTION = 8;
 int dutyCycle = 1;

 ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

 ledcAttachPin(23, PWM_CHANNEL);
 attachInterrupt(digitalPinToInterrupt(22), isr, CHANGE);

 ledcWrite(PWM_CHANNEL, dutyCycle);

 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

 client.setCallback(callback);
 client.setServer(server,1883);
 reconnect();
 client.loop();

 pinMode(LEDBLUE, OUTPUT);
 digitalWrite(LEDBLUE, HIGH);
 delay(100);
 digitalWrite(LEDBLUE, LOW);
 LEDState = 0;
 Serial.setTimeout(100);//0.1 second timeout
 Serial.println("ESP32 Lab 5");
 Count = 0;
 LEDMillis = millis();
}
void loop() {

 movingArray[samples] = distance;
 samples++;

 if(avgNum < NUMELEMENTS)
 avgNum++;
 
 if(samples >= NUMELEMENTS)
 samples = 0;
  // moving average filter
 for(int i = 0; i < avgNum; i++){
  sendDist = sendDist + movingArray[i];
 }

 sendDist = sendDist / avgNum;
 
 if(millis() - LEDMillis >= 1000){
 LEDMillis = millis();

 temp["Distance"] = sendDist;
 temp["Units"] = "Inches";

 serializeJson(temp, distPayload);

 distPayload.toCharArray(serial, 50);

 Serial.print("Distance: ");
 Serial.println(sendDist);
 client.publish("/ece631/Lab7/Distance/SensorID/0", serial);
 temp.clear();
 distPayload = "";
 }
 
 client.loop();
 if(doc["LED"] == "ON")
 {
  digitalWrite(LEDBLUE, HIGH);
  client.publish("/ece631/Lab5/LED/State", "{\"LED\":\"ON\"}");
 }
 else if(doc["LED"] == "OFF")
 {
  digitalWrite(LEDBLUE,LOW);
  client.publish("/ece631/Lab5/LED/State", "{\"LED\":\"OFF\"}");
 }
 if(doc["Flash"] >= 100 && doc["Flash"] <= 10000)
 {
  
  flash = true;
  rate = doc["Flash"];
  
  while(flash){
  client.loop();
  
  if(doc["LED"] == "ON" || doc["LED"] == "OFF")
  {
  flash = false;
  break;
  }
  
  if(millis() - LEDMillis >= rate)
  {
    LEDMillis = millis();
    LEDState = LEDState ^ HIGH;
    digitalWrite(LEDBLUE, LEDState);
    if(LEDState == HIGH)
    {
      client.publish("/ece631/Lab5/LED/State", "{\"LED\":\"ON\"}");
      Serial.println("{\"LED\":\"ON\"}");
    }
    else
    {
      client.publish("/ece631/Lab5/LED/State", "{\"LED\":\"OFF\"}");
      Serial.println("{\"LED\":\"OFF\"}");
    }
  }
  }
 } 
}
