// ------------------ INCLUDES & LIBRARIES ------------------

#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <PWMServo.h>
#include <TFT_eSPI.h>
#include <CircularBuffer.hpp>
#include <DFRobotDFPlayerMini.h>

// --------------------------------------------------------------- DEBUG

// #define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINTLN(x, d) \
  { \
    Serial.println(F(x)); \
    delay(d); \
  }
#else
#define DEBUG_PRINTLN(x, d)
#endif

// ------------------ DISPLAY SETTINGS ------------------

#define TOP_MARGIN 35
#define VALUE_CLEAR_WIDTH 150  // Increased clearing width for EMF value
#define VALUE_CLEAR_HEIGHT 40  // Increased clearing height for EMF value

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240

#define DISPLAY_LEFT_MARGIN 5       // Fixed left margin for labels
#define DISPLAY_TOP_MARGIN  5
#define LABEL_WIDTH 10              // Width allocated for labels
#define LABEL_HEIGHT 20

#define GRAPH_X_OFFSET 30
#define GRAPH_Y_OFFSET (DISPLAY_TOP_MARGIN + LABEL_HEIGHT)
#define GRAPH_WIDTH  220  // Adjust graph width to prevent clipping
#define GRAPH_HEIGHT 200            // Graph height

#define MAX_DATA_POINTS 75
#define CURVE_TENSION 0.3

// ------------------ COLOR DEFINITIONS ------------------

#define BACKGROUND_COLOR 0x0000
#define GRID_COLOR 0x1082
#define TEXT_COLOR 0xFFFF
#define BASELINE_COLOR 0x4208

#define CRITICAL_HIGH 0xF800
#define WARNING_HIGH 0xFD20
#define NORMAL_HIGH 0x07E0
#define NORMAL_LOW 0x07FF
#define WARNING_LOW 0x001F
#define CRITICAL_LOW 0x7817

// ------------------ CONSTANTS ------------------

#define UPDATE_INTERVAL 50  // Milliseconds for smooth updates
#define CALIBRATION_TIME 13000

#define LED_COUNT 5
const int LED_PINS[LED_COUNT] = {2, 3, 4, 5, 6}; 

#define EMF_BUFFER_SIZE 128
#define MIN_EMF_VALUE 0
#define MAX_EMF_VALUE 1000

#define SERVO_PIN 7
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 80

// ------------------ DATA STRUCTURES ------------------

struct ColorThreshold {
  float value;
  uint16_t color;
};

ColorThreshold thresholds[] = {
  { 1080, CRITICAL_HIGH },
  { 900, WARNING_HIGH },
  { 720, NORMAL_HIGH },
  { 480, NORMAL_LOW },
  { 300, WARNING_LOW },
  { 0, CRITICAL_LOW }
};

struct Point {
  float x, y;
};

struct MagBaseline {
  float x = 0;
  float y = 0;
  float z = 0;
  int readings = 0;
};

// ------------------ GLOBAL VARIABLES ------------------

TFT_eSPI tft = TFT_eSPI();
Adafruit_LIS3MDL lis3mdl = Adafruit_LIS3MDL();
DFRobotDFPlayerMini player;
PWMServo emfServo;

CircularBuffer<float, MAX_DATA_POINTS> dataBuffer;
CircularBuffer<int16_t, EMF_BUFFER_SIZE> emfBuffer; // Buffer to store EMF readings

float yMin = 0;
float yMax = 1200;
uint32_t lastUpdateTime = 0;
float currentValue = 600;  // For smooth updates

bool imuInitialized = false, playerInitialized = false, displayInitialized = false;
bool isCalibrating = false;
float calibrationOffset = 0.0;
MagBaseline baseline;
volatile unsigned long calibrationStartTime = 0;
int ledSequenceIndex = 0;
uint8_t lastRangeIndex = -1;

// Mapping constants
const float EMF_MIN = 0.0;
const float EMF_MAX = 1080.0;

// ------------------ FUNCTION DECLARATIONS ------------------

void drawGrid();
void drawBaseline();
void drawThresholdIndicators();
void staticBits();
void drawCurrentValue(float value);
void calibrateEMF();
float generateNewValue();
void updateDataBuffer(float newValue);
void redrawBackground();
void drawGraphLine();
Point calculateBezierPoint(Point p0, Point p1, Point p2, Point p3, float t);
uint16_t getColorForValue(float value);
void drawCurvedLine(Point p1, Point p2, uint16_t color1, uint16_t color2);
void updateGraph();

// ------------------ INITIALIZATION ------------------

void drawGhostLogo() {
  // Clear screen
  tft.fillScreen(TFT_BLACK);
  
  // Main body - rounder shape
  tft.fillCircle(120, 90, 60, TFT_WHITE);
  tft.fillRect(60, 90, 120, 100, TFT_WHITE);
  
  // Smoother bottom waves
  for(int i = 0; i < 6; i++) {
    tft.fillCircle(60 + (i * 24), 190, 12, TFT_WHITE);
  }
  
  // Eyes - larger, more expressive
  tft.fillCircle(95, 80, 12, TFT_BLACK);
  tft.fillCircle(145, 80, 12, TFT_BLACK);
  
  // Eye highlights
  tft.fillCircle(92, 76, 4, TFT_WHITE);
  tft.fillCircle(142, 76, 4, TFT_WHITE);
  
  // Cute smile
  for(int i = 0; i < 180; i++) {
    float angle = i * PI / 180;
    int x = 120 + cos(angle) * 20;
    int y = 110 + sin(angle) * 15;
    tft.drawPixel(x, y, TFT_BLACK);
  }

  delay(2000);
  tft.fillScreen(BACKGROUND_COLOR);
}

void setupDisplay() {
  tft.init();
  tft.setRotation(0);
  drawGhostLogo();
  displayInitialized = true;
}

void setupLEDS() {
  for (int i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
}

void setupServo() {
  emfServo.attach(SERVO_PIN);
  emfServo.write(SERVO_MIN_ANGLE);
}

void setupPlayer() {
  for (int attempts = 0; attempts < 3; attempts++) {
    if (player.begin(Serial1)) {
      playerInitialized = true;
      player.setTimeOut(500);
      DEBUG_PRINTLN("DFPlayer initialized successfully.", 200);
      setVolume(10);
      playBeepSFX();
      break;
    } else {
      DEBUG_PRINTLN("DFPlayer initialization failed. Retrying...", 500);
    }

    if (!playerInitialized && attempts > 2) {
      DEBUG_PRINTLN("DFPlayer initialization failed after multiple attempts.", 500);
      break;
    }
  }
}

void setupIMU() {
  if (!lis3mdl.begin_I2C()) {
    DEBUG_PRINTLN("Failed to find LIS3MDL chip", 2000);
  } else {
    imuInitialized = true;
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    DEBUG_PRINTLN("IMU: LIS3MDL chip found!", 300);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  setupDisplay();
  setupPlayer();
  setupServo();
  setupLEDS();
  setupIMU();

  baseline = calibrateMagnetometer();

  // Draw static elements
  staticBits();               // Draw static elements like "EMF" label
  redrawBackground();         // Draw the grid and baseline
  drawThresholdIndicators();  // Draw initial threshold labels
}

// ------------------ DRAW FUNCTIONS ------------------
#define THRESHOLD_SPACING_FACTOR 0.9  // Adjust this factor to fine-tune the spacing

// Progress bar configuration
const int BAR_X = 30;
const int BAR_Y = 100;
const int BAR_WIDTH = 260;
const int BAR_HEIGHT = 20;
const int TEXT_Y = BAR_Y - 30;
const uint32_t BAR_COLOR = TFT_BLUE;
const uint32_t BG_COLOR = TFT_BLACK;
const uint32_t FRAME_COLOR = TFT_WHITE;

void drawProgressBar(int percentage) {
  const int barWidth = 100;
  const int barHeight = 20;
  const int x = (tft.width() - barWidth) / 2;
  const int y = (tft.height() - barHeight) / 2;

  tft.drawRect(x, y, barWidth, barHeight, TFT_WHITE);
  tft.fillRect(x + 2, y + 2, (barWidth - 4) * percentage / 100, barHeight - 4, TFT_GREEN);

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(percentage) + "%", tft.width() / 2, y + barHeight + 10);
}

void drawThresholdIndicators() {
  tft.setTextSize(1);
  int labelX = DISPLAY_LEFT_MARGIN;  // Fixed position on the left side

  // Number of grid divisions
  size_t numLabels = sizeof(thresholds) / sizeof(thresholds[0]);
  int divisionHeight = GRAPH_HEIGHT / (numLabels - 1); // Even spacing

  for (size_t i = 0; i < numLabels; i++) {
    // Apply spacing factor to adjust the distance dynamically
    int centeredDivisionHeight = divisionHeight * THRESHOLD_SPACING_FACTOR;
    int yPos = GRAPH_Y_OFFSET + (i * centeredDivisionHeight) + (GRAPH_HEIGHT / 2) * (1 - THRESHOLD_SPACING_FACTOR);

    // Clear and draw the threshold label
    tft.fillRect(labelX, yPos - 8, LABEL_WIDTH, 10, BACKGROUND_COLOR);
    tft.setTextColor(thresholds[i].color);
    tft.setCursor(labelX, yPos - 3);
    tft.print(int(thresholds[i].value));
  }
}

void drawGrid() {
   // Vertical grid lines
  for (int x = 0; x <= GRAPH_WIDTH; x += GRAPH_WIDTH / 10) {
    tft.drawFastVLine(GRAPH_X_OFFSET + x, GRAPH_Y_OFFSET, GRAPH_HEIGHT, GRID_COLOR);
  }

  // Horizontal grid lines
  for (int i = 0; i <= (sizeof(thresholds) / sizeof(thresholds[0])) - 1; i++) {
    int yPos = GRAPH_Y_OFFSET + (i * (GRAPH_HEIGHT / ((sizeof(thresholds) / sizeof(thresholds[0])) - 1)));
    tft.drawFastHLine(GRAPH_X_OFFSET, yPos, GRAPH_WIDTH, GRID_COLOR);
  }
}

void drawBaseline() {
  // Calculate the baseline Y position based on the calibration offset
  int baselineY = map(baseline, yMin, yMax, GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET);

  // Draw a horizontal baseline across the graph
  for (int x = GRAPH_X_OFFSET; x < GRAPH_X_OFFSET + GRAPH_WIDTH; x += 5) {
    tft.drawPixel(x, baselineY, BASELINE_COLOR);
  }

  tft.setTextSize(1);
  tft.setTextColor(BASELINE_COLOR);
  tft.setCursor(GRAPH_X_OFFSET, baselineY + 5);
  tft.print("baseline ");
  tft.print(baseline);
}

void staticBits() {
  tft.fillRect(0, 0, DISPLAY_WIDTH, TOP_MARGIN, BACKGROUND_COLOR);
  tft.setTextSize(2);
  tft.setTextColor(TEXT_COLOR);
  tft.setCursor(GRAPH_X_OFFSET, 5);
  tft.print("EMF ");
}

void drawCurrentValue(float value) {  // Definition
  static float lastValue = -1;
  if (value == lastValue) return;
  lastValue = value;

  int valueX = GRAPH_X_OFFSET + 50; // Adjust for spacing to the right of "EMF"
  tft.fillRect(valueX, 5, 100, 20, BACKGROUND_COLOR); // Clear the display area
  tft.setTextSize(2);
  tft.setTextColor(getColorForValue(value));
  tft.setCursor(valueX, 5);
  tft.print(value, 1);
}

// --------------------------------------------------------------- UPDATES

void updateServo() {
  if (isCalibrating) return;
  int lastAngle = -1;

  int16_t currentValue = emfBuffer.last();
  int servoAngle = map(currentValue, EMF_MIN, EMF_MAX, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  if (servoAngle != lastAngle){
    emfServo.write(servoAngle);
    lastAngle = servoAngle;
  }
}

void updateLEDSequence() {
  if (isCalibrating) return;

  int16_t currentValue = emfBuffer.last();
  int updateRate = map(currentValue, EMF_MIN, EMF_MAX, 800, 50);

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= updateRate) {
    digitalWrite(LED_PINS[ledSequenceIndex], LOW);
    ledSequenceIndex = (ledSequenceIndex + 1) % LED_COUNT;
    digitalWrite(LED_PINS[ledSequenceIndex], HIGH);
    lastUpdate = millis();
  }
}

// --------------------------------------------------------------- SOUND CONTROL

void setVolume(int volume) {
  if (playerInitialized) {
    player.volume(volume);
    delay(5);
  }
}

void playCalibrationSFX() {
  if (playerInitialized) {
    player.playFolder(1, 4);
    delay(2000);
  }
}

void playBeepSFX() {
  setVolume(15);

  if (playerInitialized) {
    player.playFolder(1, 1);
    delay(500);
  }
}

void PlayTrack(int TrackToPlay) {
  static int TrackPlaying = -1;

  if (TrackPlaying != TrackToPlay) {
    player.stop();
    player.playFolder(2, TrackToPlay);
    TrackPlaying = TrackToPlay;
  }
}

void soundControl(float value) {
  int currentRangeIndex = 0;
  int emfValue = static_cast<int>(value);

  if (emfValue <= 466)
    currentRangeIndex = 1;
  else if (emfValue <= 933)
    currentRangeIndex = 2;
  else
    currentRangeIndex = 3;

  if (currentRangeIndex != lastRangeIndex && playerInitialized) {
    PlayTrack(currentRangeIndex);
    lastRangeIndex = currentRangeIndex;
  }
}

// ------------------ GRAPH FUNCTIONS ------------------

// void calibrateEMF() {
//   isCalibrating = true;
//   calibrationStartTime = millis();
//   float calibrationSum = 0;
//   int readings = 0;
//   int lastLedCount = -1;
//   int lastServoPos = -1;
//   playCalibrationSFX();

//   for (int i = 0; i < LED_COUNT; i++) digitalWrite(LED_PINS[i], LOW);
//   emfServo.write(0);

//   while (millis() - calibrationStartTime < CALIBRATION_TIME) {
//     sensors_event_t event;
//     lis3mdl.getEvent(&event);
//     float magnitude = sqrt(sq(event.magnetic.x) + sq(event.magnetic.y) + sq(event.magnetic.z));
//     calibrationSum += magnitude;
//     readings++;

//     int progress = ((millis() - calibrationStartTime) * 100) / CALIBRATION_TIME;
//     int currentLedCount = map(progress, 0, 100, 0, LED_COUNT);
//     int currentServoPos = map(progress, 0, 100, 0, 80);

//     if (currentLedCount != lastLedCount) {
//       for (int i = 0; i < LED_COUNT; i++) {
//         digitalWrite(LED_PINS[i], i < currentLedCount ? HIGH : LOW);
//       }
//       lastLedCount = currentLedCount;
//     }

//     if (currentServoPos != lastServoPos) {
//       emfServo.write(currentServoPos);
//       lastServoPos = currentServoPos;
//     }

//     drawProgressBar(progress);
//     delay(10);
//   }

//   calibrationOffset = calibrationSum / readings;
//   isCalibrating = false;
//   for (int i = 0; i < LED_COUNT; i++) digitalWrite(LED_PINS[i], LOW);
//   emfServo.write(0);
//   tft.fillScreen(TFT_BLACK);
// }

float getEMFReading(const MagBaseline& baseline) {
  sensors_event_t event;
  lis3mdl.getEvent(&event);
  
  float deltaX = abs(event.magnetic.x - baseline.x);
  float deltaY = abs(event.magnetic.y - baseline.y);
  float deltaZ = abs(event.magnetic.z - baseline.z);
  
  return sqrt(sq(deltaX) + sq(deltaY) + sq(deltaZ));
}

void updateProgressIndicators(int progress) {
  int lastLedCount = -1;
  int lastServoPos = -1;
  int currentLedCount = map(progress, 0, 100, 0, LED_COUNT);
  int currentServoPos = map(progress, 0, 100, SERVO_MIN, SERVO_MAX);

  if (currentLedCount != lastLedCount) {
    for (int i = 0; i < LED_COUNT; i++) {
      digitalWrite(LED_PINS[i], i < currentLedCount ? HIGH : LOW);
    }
    lastLedCount = currentLedCount;
  }

  if (currentServoPos != lastServoPos) {
    emfServo.write(currentServoPos);
    lastServoPos = currentServoPos;
  }

  drawProgressBar(progress);
}

MagBaseline calibrateMagnetometer() {
  calibrationStartTime = millis();
  playCalibrationSFX();
  MagBaseline baseline;
  isCalibrating = true;
  
  while (millis() - calibrationStartTime < CALIBRATION_TIME) {
    sensors_event_t event;
    lis3mdl.getEvent(&event);
    
    baseline.x += event.magnetic.x;
    baseline.y += event.magnetic.y;
    baseline.z += event.magnetic.z;
    baseline.readings++;

    int progress = ((millis() - calibrationStartTime) * 100) / CALIBRATION_TIME;
    updateProgressIndicators(progress);
    delay(10);
  }

  baseline.x /= baseline.readings;
  baseline.y /= baseline.readings;
  baseline.z /= baseline.readings;
  
  return baseline;
}

void servoControl(float emfReading) {
  int servoAngle = map(emfReading, EMF_MIN, EMF_MAX, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  emfServo.write(servoAngle);
}

void ledControl(float emfReading) {
  static unsigned long lastLedUpdate = 0;
  static int currentLed = 0;
  int updateInterval = map(emfReading, EMF_MIN, EMF_MAX, 800, 50); // 500ms slowest, 50ms fastest
  
  if (millis() - lastLedUpdate > updateInterval) {
    for (int i = 0; i < LED_COUNT; i++) {
      digitalWrite(LED_PINS[i], LOW);
    }
    
    digitalWrite(LED_PINS[currentLed], HIGH);
    currentLed = (currentLed + 1) % LED_COUNT;
    lastLedUpdate = millis();
  }
}

void updateOperationalIndicators(float emfReading) {
  static float lastReading = -1;

  if (lastReading != emfReading) {
    soundControl(emfReading);
    servoControl(emfReading);
    ledControl(emfReading);
    lastReading = emfReading;
  }
}

void updateDataBuffer(float newValue) {
  if (dataBuffer.size() == MAX_DATA_POINTS) {
    dataBuffer.shift();
  }
  dataBuffer.push(newValue);
}

void redrawBackground() {
  tft.fillRect(GRAPH_X_OFFSET, GRAPH_Y_OFFSET, GRAPH_WIDTH, GRAPH_HEIGHT, BACKGROUND_COLOR);
  drawGrid();
  drawBaseline();
}

void drawGraphLine() {
  if (dataBuffer.size() > 1) {
    for (int i = 0; i < dataBuffer.size() - 1; i++) {
      Point p1 = {
        (float)map(i, 0, MAX_DATA_POINTS - 1, GRAPH_X_OFFSET, GRAPH_X_OFFSET + GRAPH_WIDTH),
        (float)map(dataBuffer[i], yMin, yMax, GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET)
      };
      Point p2 = {
        (float)map(i + 1, 0, MAX_DATA_POINTS - 1, GRAPH_X_OFFSET, GRAPH_X_OFFSET + GRAPH_WIDTH),
        (float)map(dataBuffer[i + 1], yMin, yMax, GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET)
      };

      uint16_t color1 = getColorForValue(dataBuffer[i]);
      uint16_t color2 = getColorForValue(dataBuffer[i + 1]);

      drawCurvedLine(p1, p2, color1, color2);
    }
  }
}

// ------------------ BEZIER CURVE FUNCTIONS ------------------

Point calculateBezierPoint(Point p0, Point p1, Point p2, Point p3, float t) {
  float tt = t * t;
  float ttt = tt * t;
  float u = 1.0f - t;
  float uu = u * u;
  float uuu = uu * u;

  Point pt;
  pt.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
  pt.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
  return pt;
}

uint16_t getColorForValue(float value) {
  for (size_t i = 0; i < sizeof(thresholds) / sizeof(thresholds[0]); i++) {
    if (value >= thresholds[i].value) {
      return thresholds[i].color;
    }
  }
  return CRITICAL_LOW;
}

void drawCurvedLine(Point p1, Point p2, uint16_t color1, uint16_t color2) {
  Point ctrl1 = { (float)(p1.x + (p2.x - p1.x) * CURVE_TENSION), p1.y };
  Point ctrl2 = { (float)(p2.x + (p2.x - p1.x) * CURVE_TENSION), p2.y };

  const int segments = 20;
  for (int i = 0; i < segments; i++) {
    float t1 = (float)i / segments;
    float t2 = (float)(i + 1) / segments;

    Point pt1 = calculateBezierPoint(p1, ctrl1, ctrl2, p2, t1);
    Point pt2 = calculateBezierPoint(p1, ctrl1, ctrl2, p2, t2);

    tft.drawLine(pt1.x, pt1.y, pt2.x, pt2.y, color1);
  }
}

// ------------------ UPDATE FUNCTION ------------------

void updateGraph() {
  if (millis() - lastUpdateTime < UPDATE_INTERVAL) return;
  lastUpdateTime = millis();

  float newValue = getEMFReading(baseline);
  updateOperationalIndicators(newValue);
  currentValue = newValue;

  redrawBackground();
  drawGraphLine();
  drawThresholdIndicators();
  drawCurrentValue(newValue);
  updateDataBuffer(newValue);
}

// ------------------ CORE LOOP ------------------

void loop() {
  updateGraph();
}