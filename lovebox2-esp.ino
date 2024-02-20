//Include Animation Files
#include "default_animation.h"

// Define the Vibrator pin
const int vib = 10;

//Include Servo Library
#include <Servo.h>
Servo myservo; 

// Define servo positions
const int originPos = 90;
const int leftPos = 0;   // 90 degrees left of origin
const int rightPos = 180; // 90 degrees right of origin

unsigned long previousMillisServi = 0; // will store last time servo was updated
const long intervalServo = 1000; // interval at which to move servo (1 second)

// Wifi & HTTP Libraries
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>


// Wifi & HTTP URL Setup
const char *ssid = "BELL049";
const char *password = "EEF34D93CEAC";
const char* serverUrl = "http://lovebox2-server-8d8e33f8d8e3.herokuapp.com";

//Global Variables
uint8_t nav = 0; // Determines Current Page

std::vector<uint16_t> imageData; // Stores recieved message RGB565 image data
String textData; // Stores recieved message text data

bool newMessageRead = false;
int currentMessageIndex = -1;
int maxMessageIndex = -1;


// Display Libraries
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// Display pins defined
#define TFT_CS         D8
#define TFT_RST        D3
#define TFT_DC         D4

// Display Initialization
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// MPU6050 Libraries (Accelorometer & Gyro)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Initialize MPU
Adafruit_MPU6050 mpu;

// Defining time intervals for message checks
unsigned long lastMessageCheckTime = 0;
const unsigned long messageCheckInterval = 10000; // 10 seconds

//Defining time intervals for animation frames
unsigned long previousMillis = 0; // will store last time the display was updated
const long interval = 40; // interval at which to refresh the display (milliseconds)

bool clearScreen = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  myservo.attach(D0, 500, 2400);  // attaches the servo on pin D0 to the servo object
  myservo.write(originPos); // start at origin position
  
  pinMode(vib, OUTPUT); // Initialize the Vibrator pin as an output
  digitalWrite(vib, LOW); //set vibrator off

  //Initialize Screen
  tft.initR(INITR_GREENTAB_WS);
  tft.setRotation(2);
  tft.fillScreen(0x0000);
  tft.setTextSize(1);
  tft.setTextColor(0xFFFF, 0x0000);
  displayGraphic(width, height, animation[0], "Connecting to WIFI"); //Display Broken Box
  Serial.println(F("Screen Initialized"));
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi.");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print('.');
  }
  Serial.println(' ');
  Serial.println("Connected to WiFi");
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Found!");

  tft.fillScreen(0x0000);

  //Short delay before Loop
  delay(100);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // Get MPU Data For Gesture Detection
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Every 10sec Check if a new message is available (Non-Blocking)
  if (millis() - lastMessageCheckTime >= messageCheckInterval) {
    if (getNewMessage()){ // if new message available
      newMessageRead = false;
      tft.fillScreen(0x0000);
      displayGraphic(width, height, animation[1], "Fetching Message");
      getImageData(); // Get and save image data
      clearScreen = true;
      moveServoSequence();
      nav = 1; //navigate to new message page
    }
    lastMessageCheckTime = millis(); // Reset the timer
  }

  //Clear Screen
  if (clearScreen){
    tft.fillScreen(0x0000);
    clearScreen = false;
  }

  // Check Page To Determine Controls and Display
  if(nav == 0){ // Home Page
    displayAnimation(1, 4, 4, width, height, animation, "LoveBox 2.0"); //Display Default Animation
      // Tilt detection
    if (sqrt((a.acceleration.x * a.acceleration.x) + (a.acceleration.z * a.acceleration.z) + (a.acceleration.y * a.acceleration.y)) > 18.0) { // Adjust this threshold to your needs
      // Left tilt detected
      Serial.println("Tilt left detected!");
      hapticPulse();
      if (getLatestMessage()){ //Check if there is a History & Fetch Index
        nav = 2; //Navigate to Message History Page
        tft.fillScreen(0x0000);
        getIndexMessage(currentMessageIndex);
        displayGraphic(width, height, animation[1], "Fetching Message");
        getImageData();
        tft.fillScreen(0x0000);
        diplayMessage();
      }
      delay(500); // Delay to debounce and avoid multiple detections
    }
  }
  
  else if (nav == 1) { // New Message Page
    //Clear Screen
    if(newMessageRead == false){
      displayAnimation(5, 16, 12, width, height, animation, "Shake To Open"); //Display Default Animation
    }
    // Shake detection
    if (sqrt((a.acceleration.x * a.acceleration.x) + (a.acceleration.z * a.acceleration.z) + (a.acceleration.y * a.acceleration.y)) > 18.0) {
      Serial.println("Shake detected!");
      hapticPulse();
      if(newMessageRead == false){ // Message hasn't been read yet
        //Clear Screen
        diplayMessage();
        Serial.println("Displaying Message");
        newMessageRead = true; // Set Message Read to True
      }else{ // Message has been read already
        setMessageReadStatus(); // Notify Server that Message is read
        clearScreen = true;
        nav = 0; // Navigate back home
      }
      // Delay to debounce and avoid multiple detections
      delay(500);
    }
  }
  
  else if (nav == 2){ // Message History Page
    // Shake detection
    if (sqrt((a.acceleration.x * a.acceleration.x) + (a.acceleration.z * a.acceleration.z) + (a.acceleration.y * a.acceleration.y)) > 18.0) {
      Serial.println("Shake detected!");
      hapticPulse();
      clearScreen = true;
      nav = 0; // Go back to home page
      delay(500); // Delay to debounce and avoid multiple detections
    }

    // Tilt detection
    if (a.acceleration.y > 6.0) { // Adjust this threshold to your needs
      // Left tilt detected
      Serial.println("Tilt left detected!");
      hapticPulse();
      currentMessageIndex -= 1; // Decrement Message Index by 1
      if(currentMessageIndex > 0){ // Check if message at index is valid
        getIndexMessage(currentMessageIndex);
        tft.fillScreen(0x0000);
        displayGraphic(width, height, animation[1], "Fetching Message");
        getImageData();
        tft.fillScreen(0x0000);
        diplayMessage();
      }else{
        currentMessageIndex += 1; // Revert Back to original current index
      }
      
      delay(500); // Delay to debounce and avoid multiple detections
    } else if (a.acceleration.y < -6.0) { // Adjust this threshold to your needs
      // Right tilt detected
      Serial.println("Tilt right detected!");
      hapticPulse();
      currentMessageIndex += 1; // Increment Message Index by 1
      if(currentMessageIndex<=maxMessageIndex){ // check if message at index is available
        getIndexMessage(currentMessageIndex);
        tft.fillScreen(0x0000);
        displayGraphic(width, height, animation[1], "Fetching Message");
        getImageData();
        tft.fillScreen(0x0000);
        diplayMessage();
      }else{
        currentMessageIndex -= 1; //Revert back to original current index
      }
      delay(500); // Delay to debounce and avoid multiple detections
    }
    
  }

  delay(50); // Simple delay to limit reading frequency
}

// End Point Request Functions

// Get latest Message Index
bool getLatestMessage() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return false;
  }
  
  WiFiClient client;
  HTTPClient http;
  
  String url = String(serverUrl) + "/get_latest_message_index";
  http.begin(client, url);

  int httpCode = http.GET();
  if (httpCode <= 0) {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("HTTP request did not return OK.");
    http.end();
    return false;
  }

  String payload = http.getString();

  // Use StaticJsonDocument with a fixed size (e.g., 1024)
  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload);

  // Access the JSON data
  currentMessageIndex = doc["data"]["index"];
  maxMessageIndex = currentMessageIndex;
  Serial.println("Latest Message Index:"+currentMessageIndex);
  bool newMessageAvailable = doc["status"];
  Serial.println("Latest Message Available: " + String(newMessageAvailable));
  return newMessageAvailable;
  
  http.end();
}

// Get Index Message Status and Save Text data
bool getIndexMessage(int indexMessage) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return false;
  }
  
  WiFiClient client;
  
  HTTPClient http;
  
  String url = String(serverUrl) + "/get_index_message/" + String(indexMessage);
  Serial.println(url);
  http.begin(client, url);

  int httpCode = http.GET();
  if (httpCode <= 0) {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("HTTP request did not return OK.");
    http.end();
    return false;
  }

  String payload = http.getString();

  // Use StaticJsonDocument with a fixed size (e.g., 1024)
  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload);

  // Access the JSON data
  textData = doc["data"]["text"].as<String>();
  bool newMessageAvailable = doc["status"];
  Serial.println("Latest Message Available: " + String(newMessageAvailable));
  return newMessageAvailable;
  
  http.end();
}

// Get New Message Status and Save Text data
bool getNewMessage() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return false;
  }
  
  WiFiClient client;
  
  HTTPClient http;
  
  String url = String(serverUrl) + "/get_new_message";
  http.begin(client, url);

  int httpCode = http.GET();
  if (httpCode <= 0) {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("HTTP request did not return OK.");
    http.end();
    return false;
  }

  String payload = http.getString();

  // Use StaticJsonDocument with a fixed size (e.g., 1024)
  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload);

  // Access the JSON data
  textData = doc["data"]["text"].as<String>();
  Serial.println("New Message:"+textData);
  bool newMessageAvailable = doc["status"];
  Serial.println("New Message Available: " + String(newMessageAvailable));
  return newMessageAvailable;
  
  http.end();
}

// Set New Message Read Status to True
void setMessageReadStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }
  
  WiFiClient client;
  
  HTTPClient http;
  
  String url = String(serverUrl) + "/set_message_read";
  http.begin(client, url);

  int httpCode = http.GET();
  if (httpCode <= 0) {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return;
  }

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("HTTP request did not return OK.");
    http.end();
    return;
  }

  Serial.println("Successfully Set Message to Read");
  
  http.end();
}

// Get Image Data From Message and Save it
void getImageData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }

  WiFiClient client;
  
  HTTPClient http;

  String url = String(serverUrl) + "/get_image_data";
  http.begin(client, url);
  
  int httpCode = http.GET();

  if (httpCode <= 0) {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return;
  }

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("HTTP request did not return OK.");
    http.end();
    return;
  }
  

  WiFiClient* stream = http.getStreamPtr();
  imageData.clear(); // Clear existing data
  imageData.reserve(128 * 135); // Pre-allocate memory

  uint8_t buffer[512];
  int len = http.getSize();

  while (http.connected() && (len > 0 || len == -1)) {
    size_t size = stream->available();
    if (size) {
      int c = stream->readBytes(buffer, sizeof(buffer));
      for (int i = 0; i < c; i += 2) {
        if ((i + 1) < c) {
          imageData.push_back((uint16_t(buffer[i + 1]) << 8) | uint16_t(buffer[i]));
        }
      }
      if (len > 0) len -= c;
    }
    yield(); // Small delay to allow WiFi and background tasks
  }

  if (!imageData.empty()) {
    Serial.println("Image data received.");
  } else {
    Serial.println("No image data received.");
  }

  http.end();
}

//Display Functions

// Display Message
void diplayMessage() {
  tft.drawRGBBitmap(0, 0, imageData.data(), 128, 135);
  drawTextMessage(textData);
}

void drawText(String text){
 int textLen = text.length();
  tft.setCursor((80 - ((textLen*8)/2)),115);
  tft.print(text);
}

void drawTextMessage(String text){
  int textLen = text.length();
  tft.setCursor(3,138);
  tft.print(text);
}

void displayAnimation(int start_frame, int end_frame, int frames, int width, int height, const unsigned short animation[][4440], String text) {
  static int frame = start_frame;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    if (frame <= end_frame) {
      drawText(text);
      tft.drawRGBBitmap((128/2) - (width/2), (160/2) - (height/2) - 15, animation[frame], width, height);
      frame++;
    } else {
      // Reset frame counter and optionally wait a bit longer after the last frame
      frame = start_frame;
    }
  }
}

void displayGraphic (int width, int height, const unsigned short graphic[4624], String text) {
    drawText(text);
    tft.drawRGBBitmap((128/2) - (width/2), (160/2) - (height/2) - 15, graphic, width, height);
}

void moveServoSequence() {
  myservo.write(rightPos);
  delay(500);
  myservo.write(leftPos);
  delay(500);
  myservo.write(originPos);
  delay(500);
}

void hapticPulse() {
  digitalWrite(vib, HIGH);
  delay(50);
  digitalWrite(vib, LOW);
}
