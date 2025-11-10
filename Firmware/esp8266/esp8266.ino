#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <MPU6050.h>

// ====== Hotspot credentials ======
const char* ssid = "mioths";
const char* password = "12345678";

// ====== Motor control pins ======
#define IN1 D1
#define IN2 D2
#define IN3 D5
#define IN4 D7

// ====== Define RX/TX pins if not already ======
#ifndef RX
#define RX 3
#endif
#ifndef TX
#define TX 1
#endif

// ====== Encoder pins ======
#define ENC_LEFT_A  D6
#define ENC_LEFT_B  D0
#define ENC_RIGHT_A RX
#define ENC_RIGHT_B TX

volatile long leftCount  = 0;
volatile long rightCount = 0;

ESP8266WebServer server(80);

// ====== IMU setup ======
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// ====== Motor control ======
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  Serial.println("Moving backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  Serial.println("Turning left");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  Serial.println("Turning right");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ====== Encoder ISRs ======
void IRAM_ATTR leftEncoderA_ISR() {
  bool b = digitalRead(ENC_LEFT_B);
  if (b) leftCount++;
  else   leftCount--;
}

void IRAM_ATTR rightEncoderA_ISR() {
  bool b = digitalRead(ENC_RIGHT_B);
  if (b) rightCount++;
  else   rightCount--;
}

// ====== Wi-Fi setup ======
void connectToWiFi() {
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connected to hotspot!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi. Starting AP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RobotCar", "12345678");
    Serial.print("Connect to Wi-Fi: RobotCar, then visit http://");
    Serial.println(WiFi.softAPIP());
  }
}

// ====== HTTP handlers ======
void handleRoot()        { server.send(200, "text/plain", "ESP8266 Robot Online"); }
void handleForward()     { moveForward();  server.send(200, "text/plain", "Moving Forward"); }
void handleBackward()    { moveBackward(); server.send(200, "text/plain", "Moving Backward"); }
void handleStop()        { stopMotors();   server.send(200, "text/plain", "Stopped"); }
void handlePing()        { server.send(200, "text/plain", "pong"); }

void handleCounts() {
  char buffer[80];
  snprintf(buffer, sizeof(buffer), "Left: %ld | Right: %ld", leftCount, rightCount);
  server.send(200, "text/plain", buffer);
}

// ====== New: IMU handler ======
void handleIMU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  char buffer[120];
  snprintf(buffer, sizeof(buffer),
           "{\"ax\":%d,\"ay\":%d,\"az\":%d,\"gx\":%d,\"gy\":%d,\"gz\":%d}",
           ax, ay, az, gx, gy, gz);
  server.send(200, "application/json", buffer);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  leftEncoderA_ISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderA_ISR, RISING);

  stopMotors();
  connectToWiFi();

  // ---- I2C setup (D3 = SDA, D4 = SCL) ----
  Wire.begin(D3, D4);
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connected!" : "MPU6050 connection failed!");

  // ---- Server routes ----
  server.on("/", handleRoot);
  server.on("/ping", handlePing);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/stop", handleStop);
  server.on("/counts", handleCounts);
  server.on("/imu", handleIMU);
  server.on("/left", turnLeft);
  server.on("/right", turnRight);

  server.begin();
  Serial.println("HTTP server started");
}

// ====== Loop ======
void loop() {
  server.handleClient();
}
