// ------------------ INCLUDES & LIBRARIES ------------------

#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <PWMServo.h>
#include <TFT_eSPI.h>
#include <CircularBuffer.hpp>
#include <DFRobotDFPlayerMini.h>

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
#define LED_COUNT 7
#define EMF_BUFFER_SIZE 128
#define MIN_EMF_VALUE 0
#define MAX_EMF_VALUE 1000
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 80

// ------------------ GLOBAL VARIABLES ------------------

TFT_eSPI tft = TFT_eSPI();
CircularBuffer<float, MAX_DATA_POINTS> dataBuffer;
float yMin = 0;
float yMax = 1200;
uint32_t lastUpdateTime = 0;
float currentValue = 600;  // For smooth updates

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

// ------------------ FUNCTION DECLARATIONS ------------------

void drawGrid();
void drawBaseline();
void drawThresholdIndicators();
void staticBits();
void drawCurrentValue(float value);
float generateNewValue();
void updateDataBuffer(float newValue);
void redrawBackground();
void drawGraphLine();
Point calculateBezierPoint(Point p0, Point p1, Point p2, Point p3, float t);
uint16_t getColorForValue(float value);
void drawCurvedLine(Point p1, Point p2, uint16_t color1, uint16_t color2);
void updateGraph();

// ------------------ INITIALIZATION ------------------

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(BACKGROUND_COLOR);

  staticBits();               // Draw static elements like "EMF" label
  redrawBackground();         // Draw the grid and baseline
  drawThresholdIndicators();  // Draw initial threshold labels
}

// ------------------ DRAW FUNCTIONS ------------------

#define NUM_GRID_DIVISIONS 8  // Increase this value to tighten the gap between labels
#define THRESHOLD_SPACING_FACTOR 0.9  // Adjust this factor to fine-tune the spacing

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
  int baselineY = GRAPH_Y_OFFSET + GRAPH_HEIGHT / 2;
  for (int x = GRAPH_X_OFFSET; x < GRAPH_X_OFFSET + GRAPH_WIDTH; x += 5) {
    tft.drawPixel(x, baselineY, BASELINE_COLOR);
  }

  tft.setTextSize(1);
  tft.setTextColor(BASELINE_COLOR);
  tft.setCursor(GRAPH_X_OFFSET, baselineY + 5);
  tft.print("baseline");
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

// ------------------ GRAPH FUNCTIONS ------------------

float generateNewValue() {
  float t = millis() / 100.0;
  return 500 + 40 * sin(t) * cos(t / 2) + random(-2, 2);
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

  float newValue = generateNewValue();
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