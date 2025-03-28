#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <DIYables_IRcontroller.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Define hardware pins
#define DHT11_PIN  17         // ESP32 GPIO17 connected to DHT11 sensor
#define IR_RECEIVER_PIN 19    // ESP32 GPIO19 connected to IR receiver
#define LED_PIN 23            // ESP32 GPIO23 for LED control

// Initialize IR controller with debounce time of 200ms
DIYables_IRcontroller_21 irController(IR_RECEIVER_PIN, 200);

// Initialize DHT11 temperature sensor
DHT dht11(DHT11_PIN, DHT11);

// Define servo motor control pins
static const int servoPan = 13;
static const int servoTilt = 14;

// Servo motor instances
Servo servoP;
Servo servoT;

// LCD configuration
int lcdColumns = 16;
int lcdRows = 2;

// Set LCD address and initialize it
// If unsure about the address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// Global variables to track system state
bool state = false;    // Stores LED on/off state
int ServoPPos;         // Tracks pan servo position
int ServoTPos;         // Tracks tilt servo position

// WiFi Credentials (Replace with actual credentials)
const char* ssid = "YOUR SSID";
const char* password = "YOUR PASSWORD";

// Create an instance of the web server on port 80
AsyncWebServer server(80);

/**
 * @brief Control center function to process commands for light and servo movements.
 * @param command String command received (e.g., "ON", "OFF", "UP", "DOWN", etc.)
 */
void controlcenter(String command) {

  // Turn light on
  if (command == "ON") {
    analogWrite(LED_PIN, 255);
  }

  // Turn light off
  if (command == "OFF") {
    analogWrite(LED_PIN, 0);
  }

  // Servo tilt up (increments position by 15 degrees)
  if (command == "UP") {
    if (ServoTPos < 180) {
      int newpos = ServoTPos;
      for (int idx = ServoTPos; idx <= ServoTPos + 15; idx++) {
        Serial.println(idx);
        servoT.write(idx);
        delay(20);
        newpos = idx;
      }
      ServoTPos = newpos;
    }
  }

  // Servo tilt down
  if (command == "DOWN") {
    if (ServoTPos > 0) {
      int newpos = ServoTPos;
      for (int idx = ServoTPos; idx >= ServoTPos - 15; idx--) {
        Serial.println(idx);
        servoT.write(idx);
        delay(20);
        newpos = idx;
      }
      ServoTPos = newpos;
    }
  }

  // Servo pan right
  if (command == "RIGHT") {
    if (ServoPPos < 180) {
      int newpos = ServoPPos;
      for (int idx = ServoPPos; idx <= ServoPPos + 15; idx++) {
        Serial.println(idx);
        servoP.write(idx);
        delay(20);
        newpos = idx;
      }
      ServoPPos = newpos;
    }
  }

  // Servo pan left
  if (command == "LEFT") {
    if (ServoPPos > 0) {
      int newpos = ServoPPos;
      for (int idx = ServoPPos; idx >= ServoPPos - 15; idx--) {
        Serial.println(idx);
        servoP.write(idx);
        delay(20);
        newpos = idx;
      }
      ServoPPos = newpos;
    }
  }
}

/**
 * @brief Displays messages on the LCD screen.
 * @param message String message to be displayed.
 */
void writelcd(String message) {

  if (message == "ON") {
    lcd.setCursor(0, 1);
    lcd.print("STATUS:ON ");
  }
  if (message == "OFF") {
    lcd.setCursor(0, 1);
    lcd.print("STATUS:OFF");
  }

  // Display main message
  lcd.setCursor(0, 0);
  lcd.print(message);
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("              "); // Clear previous message
}

/**
 * @brief Reads temperature from the DHT11 sensor and displays it on the LCD.
 */
void readtemperature() {

  float tempC = dht11.readTemperature();

  // Check if reading is successful
  if (isnan(tempC)) {
    Serial.println("Failed to read from DHT11 sensor!");
    return;
  }

  if (tempC > 0) {
    lcd.setCursor(12, 1);
    String message = String(int(round(tempC))) + (char)223 + "C"; // Format temperature with degree symbol
    Serial.println(message);
    lcd.print(message);
  }

  delay(1000); // Delay between readings
}

/**
 * @brief Setup function to initialize hardware components and WiFi.
 */
void setup() {
  Serial.begin(115200);

  // Initialize SPIFFS for file storage
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed!");
    return;
  }

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 1);
  lcd.print("STATUS:OFF");

  // Initialize IR Controller
  irController.begin();

  // Initialize servos and set default positions
  servoP.attach(servoPan);
  servoT.attach(servoTilt);
  servoP.write(90);
  ServoPPos = 90;
  servoT.write(90);
  ServoTPos = 90;

  // Initialize DHT11 sensor
  dht11.begin();

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Define Web Server Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/servo", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (request->hasParam("direction")) {
      String dir = request->getParam("direction")->value();
      controlcenter(dir);
      request->send(200, "text/plain", "OK");
    }
  });

  server.on("/led", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (request->hasParam("state")) {
      String ledState = request->getParam("state")->value();
      if (ledState == "on") {
        digitalWrite(LED_PIN, HIGH);
        writelcd("ON");
        controlcenter("ON");
      } else {
        digitalWrite(LED_PIN, LOW);
        writelcd("OFF");
        controlcenter("OFF");
      }
      request->send(200, "text/plain", "OK");
    }
  });

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    float tempC = dht11.readTemperature();
    if (isnan(tempC)) {
      request->send(500, "text/plain", "Error reading temperature");
    } else {
      request->send(200, "text/plain", String(tempC));
    }
  });

  // Start Web Server
  server.begin();
}

/**
 * @brief Main loop that checks for IR remote commands and updates temperature.
 */
void loop() {
  Key21 key = irController.getKey();
  if (key != Key21::NONE) {
    switch (key) {
      case Key21::KEY_CH_MINUS:
        state = !state;
        writelcd(state ? "ON" : "OFF");
        controlcenter(state ? "ON" : "OFF");
        break;
      case Key21::KEY_CH:
        writelcd("UP");
        controlcenter("UP");
        break;
      case Key21::KEY_VOL_PLUS:
        writelcd("DOWN");
        controlcenter("DOWN");
        break;
      case Key21::KEY_PREV:
        writelcd("LEFT");
        controlcenter("LEFT");
        break;
      case Key21::KEY_PLAY_PAUSE:
        writelcd("RIGHT");
        controlcenter("RIGHT");
        break;
    }
  }

  readtemperature();
}
