#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>

const int LEFT_IR_SENSOR = 3;
const int MIDDLE_IR_SENSOR = 4;
const int RIGHT_IR_SENSOR = 5;

const int LEFT_US_TRIG_SENSOR = 13;
const int LEFT_US_ECHO_SENSOR = 36;
const int MIDDLE_US_TRIG_SENSOR = 14;
const int MIDDLE_US_ECHO_SENSOR = 35;
const int RIGHT_US_TRIG_SENSOR = 12;
const int RIGHT_US_ECHO_SENSOR = 39;

Servo esc;
const int ESC_PIN = 40;
bool motorsEnabled = false;

bool BoEnabled = false;
const int L298D_L_C_PIN = 42;
const int L298D_L_A_PIN = 41;
const int L298D_R_C_PIN = 37;
const int L298D_R_A_PIN = 38;
const int L298D_L_EN_PIN = 9;
const int L298D_R_EN_PIN = 10;

#define L_PWM_CHANNEL 0
#define R_PWM_CHANNEL 1

int PWM_FREQ = 1000;
int PWM_RES = 8;

const int L298D_B_A_PIN = 11;
const int L298D_B_C_PIN = 46;

String serialLog = "";

// WiFi Configuration
const char *ssid = "Your Wifi Ssid "; // Change to your WiFi SSID
const char *password = "Your Wifi Password"; // Change to your WiFi Password
IPAddress local_IP(192, 168, 1, 10);      // Change to your local IP for static IP.
IPAddress gateway(192, 168, 1, 254);      // Change to your gateway IP based on your router.
IPAddress subnet(255, 255, 255, 0);       //Change to your subnet mask.

// AsyncWebServer
AsyncWebServer server(80);

// Function Declarations
void moveStart();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMovement();
void brushmortoron();
void brushmortoroff();
void setSpeed(int motorSpeed);
void setupMotors();
void BrushlessSetup();
void setBrushlessMotorSpeed(int speed);
void setupSensors();
void setupbrushmortors();
long readUltrasonicDistance(int trigPin, int echoPin);
void setupRoutes();
void reconnectWiFi();
void log(const String &msg);
void logf(const char *fmt, ...);

void setupRoutes()
{
  // IR sensor endpoints
  server.on("/api/irsensor/left", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(digitalRead(LEFT_IR_SENSOR))); });

  server.on("/api/irsensor/middle", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(digitalRead(MIDDLE_IR_SENSOR))); });

  server.on("/api/irsensor/right", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", String(digitalRead(RIGHT_IR_SENSOR))); });

  // Ultrasonic sensor endpoints
  server.on("/api/ussensor/left", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    long dist = readUltrasonicDistance(LEFT_US_TRIG_SENSOR, LEFT_US_ECHO_SENSOR);
    request->send(200, "text/plain", String(dist)); });

  server.on("/api/ussensor/middle", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    long dist = readUltrasonicDistance(MIDDLE_US_TRIG_SENSOR, MIDDLE_US_ECHO_SENSOR);
    request->send(200, "text/plain", String(dist)); });

  server.on("/api/ussensor/right", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    long dist = readUltrasonicDistance(RIGHT_US_TRIG_SENSOR, RIGHT_US_ECHO_SENSOR);
    request->send(200, "text/plain", String(dist)); });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    request->send(200, "text/plain", serialLog);
    serialLog = ""; });

  // Motor Control Endpoint
  server.on("/api/sucksation/", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      if (request->hasParam("cmd", true)) {
        String cmd = request->getParam("cmd", true)->value();
        log("Command: " + cmd);
    
        if (cmd == "vaccumoff")
        {
          setBrushlessMotorSpeed(0);
          log("Vacuum motor speed set to off");
        }
        else if(cmd == "vaccumhalf"){
          setBrushlessMotorSpeed(128);
          log("Vacuum motor speed set to half speed");
        }
        else if(cmd == "vaccumfull"){
          setBrushlessMotorSpeed(255);
          log("Vacuum motor speed set to full speed");
        }
        else{
          log("Invalid command form Vaccum/Sucksation");
        }
    
        request->send(200, "text/plain", "OK");
      }

    }
  );


    // Motor Control Endpoint
  server.on("/api/brush/", HTTP_POST, [](AsyncWebServerRequest *request)
          {
            if (request->hasParam("cmd", true)) {
            String cmd = request->getParam("cmd", true)->value();
            Serial.println("Command: " + cmd);

            if (cmd == "brushoff")
            {
              brushmortoroff();
              log("Brush motor speed set to off");
            } 
            else if (cmd == "brushon") {
              brushmortoron();
              log("Brush motor speed set to on");
            }
            else {
            request->send(400, "text/plain", "Missing cmd parameter");
            } 
          }        
         });

  // Motor Control Endpoint
  server.on("/api/move/", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    if (request->hasParam("cmd", true)) {
      String cmd = request->getParam("cmd", true)->value();
      Serial.println("Command: " + cmd);
  
      if (cmd == "start" && BoEnabled == false)
      {
        moveStart();
      } 
      else if (BoEnabled) {
        if (cmd == "forward") 
        {
          moveForward();
        }
        else if (cmd == "backward"){
          moveBackward();
        }
        else if (cmd == "left") {
          turnLeft();
        }
        else if (cmd == "right") {
          turnRight();
        }
        else if (cmd == "stop") {
          stopMovement();
        } 
      }
  
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Missing cmd parameter");
    } });
}

// Brushless Motor Control
void setBrushlessMotorSpeed(int speed)
{
  speed = constrain(speed, 0, 255);
  int pwm = map(speed, 0, 255, 1000, 2000);
  esc.writeMicroseconds(pwm);
  logf("Set brushless motor speed to %d%% (PWM: %d µs)\n", speed, pwm);
}



void setupbrushmortors(){

  pinMode(L298D_B_A_PIN,OUTPUT);
  pinMode(L298D_B_C_PIN,OUTPUT);

}


// Motor Setup
void setupMotors()
{
  int motorPins[] = {L298D_L_C_PIN, L298D_L_A_PIN, L298D_R_C_PIN, L298D_R_A_PIN};
  for (int i = 0; i < 4; i++)
  {
    pinMode(motorPins[i], OUTPUT);
  }
  ledcSetup(L_PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(L298D_L_EN_PIN, L_PWM_CHANNEL);
  ledcSetup(R_PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(L298D_R_EN_PIN, R_PWM_CHANNEL);
}

void BrushlessSetup()
{
  Serial.println("Starting ESC setup...");
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);
  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1000);
  delay(2000);
  motorsEnabled = true;
}

// Sensor Setup
void setupSensors()
{
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(MIDDLE_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(LEFT_US_TRIG_SENSOR, OUTPUT);
  pinMode(LEFT_US_ECHO_SENSOR, INPUT);
  pinMode(MIDDLE_US_TRIG_SENSOR, OUTPUT);
  pinMode(MIDDLE_US_ECHO_SENSOR, INPUT);
  pinMode(RIGHT_US_TRIG_SENSOR, OUTPUT);
  pinMode(RIGHT_US_ECHO_SENSOR, INPUT);
}

// Sensor Utilities
long readUltrasonicDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);


  return duration * 0.034 / 2;
}

void setSpeed(int motorSpeed)
{
  ledcWrite(L_PWM_CHANNEL, motorSpeed);
  ledcWrite(R_PWM_CHANNEL, motorSpeed);
}

void moveStart()
{
  BoEnabled = true;
  setSpeed(0);
}

void moveForward()
{
  digitalWrite(L298D_L_C_PIN, LOW);
  digitalWrite(L298D_L_A_PIN, HIGH);
  digitalWrite(L298D_R_C_PIN, LOW);
  digitalWrite(L298D_R_A_PIN, HIGH);
  log("moveForward");
  setSpeed(250);
}

void moveBackward()
{
  digitalWrite(L298D_L_C_PIN, HIGH);
  digitalWrite(L298D_L_A_PIN, LOW);
  digitalWrite(L298D_R_C_PIN, HIGH);
  digitalWrite(L298D_R_A_PIN, LOW);
  log("moveBackward");
  setSpeed(250);
}

void turnRight()
{
  digitalWrite(L298D_L_C_PIN, HIGH);
  digitalWrite(L298D_L_A_PIN, LOW);
  digitalWrite(L298D_R_C_PIN, LOW);
  digitalWrite(L298D_R_A_PIN, HIGH);
  log("turnRight");
  setSpeed(240);
}

void turnLeft()
{
  digitalWrite(L298D_L_C_PIN, LOW);
  digitalWrite(L298D_L_A_PIN, HIGH);
  digitalWrite(L298D_R_C_PIN, HIGH);
  digitalWrite(L298D_R_A_PIN, LOW);
  log("turnLeft");
  setSpeed(240);
}

void stopMovement()
{
  digitalWrite(L298D_L_C_PIN, LOW);
  digitalWrite(L298D_L_A_PIN, LOW);
  digitalWrite(L298D_R_C_PIN, LOW);
  digitalWrite(L298D_R_A_PIN, LOW);
  log("stopMovement");
  BoEnabled = false;
}

void brushmortoron(){
  digitalWrite(L298D_B_C_PIN, HIGH);
  digitalWrite(L298D_B_A_PIN, LOW);
  Serial.println("brushmortoron");
}

void brushmortoroff(){
  digitalWrite(L298D_B_C_PIN, LOW);
  digitalWrite(L298D_B_A_PIN, LOW);
  Serial.println("brushmortoroff");
}

// WiFi Setup
void setupWiFi()
{
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    log("Connecting...");
  }
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(1000);
      log("Reconnecting to WiFi...");
    }
    log("Reconnected to WiFi. IP: ");
    log("Connected to WiFi. IP: " + WiFi.localIP().toString());
  }
}

void log(const String &msg)
{
  Serial.println(msg);
  serialLog += msg + "\n";
  if (serialLog.length() > 2000)
  {
    serialLog = serialLog.substring(serialLog.length() - 2000);
  }
}

void logf(const char *fmt, ...)
{
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  log(String(buf));
}

// Main Setup
void setup()
{
  Serial.begin(115200);
  setupWiFi();
  setupSensors();
  setupMotors();
  setupbrushmortors();
  BrushlessSetup();
  setupRoutes();
  server.begin();
}

void loop()
{
  reconnectWiFi();

  

}
