#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <CircularBuffer.hpp>


#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define GRAPH_HEIGHT 200
#define GRAPH_WIDTH 220
#define GRAPH_X_OFFSET 10
#define GRAPH_Y_OFFSET 20
#define MAX_DATA_POINTS 75
#define CURVE_TENSION 0.3

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

#define BASELINE_VALUE 600
#define UPDATE_INTERVAL 20  // Back to 50ms for smooth updates

TFT_eSPI tft = TFT_eSPI();
CircularBuffer<float, MAX_DATA_POINTS> dataBuffer;
float yMin = 0;
float yMax = 1200;
uint32_t lastUpdateTime = 0;
float currentValue = 600;  // Track current value for smoother updates
const uint16_t updateInterval = 50;


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

void drawGrid() {
  for (int x = 0; x <= GRAPH_WIDTH; x += GRAPH_WIDTH / 10) {
    tft.drawFastVLine(GRAPH_X_OFFSET + x, GRAPH_Y_OFFSET, GRAPH_HEIGHT, GRID_COLOR);
  }
  for (int y = 0; y <= GRAPH_HEIGHT; y += GRAPH_HEIGHT / 8) {
    tft.drawFastHLine(GRAPH_X_OFFSET, GRAPH_Y_OFFSET + y, GRAPH_WIDTH, GRID_COLOR);
  }
}


// Keep all the existing functions (calculateBezierPoint, getColorForValue, drawCurvedLine) the same
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
  Point ctrl1 = {
    p1.x + (p2.x - p1.x) * static_cast<float>(CURVE_TENSION),
    p1.y
  };

  Point ctrl2 = {
    p2.x + (p2.x - p1.x) * static_cast<float>(CURVE_TENSION),
    p2.y
  };

  // Draw the curved line with color interpolation
  const int segments = 20;

  for (int i = 0; i < segments; i++) {
    float t1 = (float)i / segments;
    float t2 = (float)(i + 1) / segments;

    Point pt1 = calculateBezierPoint(p1, ctrl1, ctrl2, p2, t1);
    Point pt2 = calculateBezierPoint(p1, ctrl1, ctrl2, p2, t2);

    // Interpolate colors
    float colorRatio = t1;
    uint8_t r1 = (color1 >> 11) & 0x1F;
    uint8_t g1 = (color1 >> 5) & 0x3F;
    uint8_t b1 = color1 & 0x1F;
    uint8_t r2 = (color2 >> 11) & 0x1F;
    uint8_t g2 = (color2 >> 5) & 0x3F;
    uint8_t b2 = color2 & 0x1F;
    uint8_t r = r1 + (r2 - r1) * colorRatio;
    uint8_t g = g1 + (g2 - g1) * colorRatio;
    uint8_t b = b1 + (b2 - b1) * colorRatio;
    uint16_t interpolatedColor = (r << 11) | (g << 5) | b;

    tft.drawLine(pt1.x, pt1.y, pt2.x, pt2.y, interpolatedColor);
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



void drawThresholdIndicators() {
  tft.setTextSize(1);

  int yMinPos = map(0, yMin, yMax, GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET);
  tft.setTextColor(WARNING_LOW);
  tft.setCursor(0, yMinPos - 3);
  tft.print(0);

  int halfValue = (yMin + yMax) / 2;
  int yHalfPos = map(halfValue, yMin, yMax, GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET);
  tft.setTextColor(NORMAL_HIGH);
  tft.setCursor(0, yHalfPos - 3);
  tft.print(halfValue);

  int yMaxPos = map(yMax, yMin, yMax, GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET);
  tft.setCursor(0, yMaxPos - 3);
  tft.setTextColor(CRITICAL_HIGH);
  tft.print((int)yMax);
}

void drawStaticText(const char* label) {
  tft.setTextSize(2);
  tft.setTextColor(TEXT_COLOR);
  tft.fillRect(5, 0, DISPLAY_WIDTH, 20, BACKGROUND_COLOR);  // Clear text area
  int16_t xOffset = (DISPLAY_WIDTH - tft.textWidth(label)) / 2;
  tft.setCursor(xOffset, 5);
  tft.print(label);
}

void drawCurrentValue(float value) {
  tft.setTextSize(2);
  tft.setTextColor(getColorForValue(value));
  tft.fillRect(5, 0, DISPLAY_WIDTH, 20, BACKGROUND_COLOR);  // Clear dynamic text area
  int16_t valueOffset = (DISPLAY_WIDTH + 50) / 2;           // Adjust dynamic value position
  tft.setCursor(valueOffset, 5);
  tft.print(value, 1);  // Display value as an integer
}

void clearGraph() {
  tft.fillRect(20, 40, DISPLAY_WIDTH, DISPLAY_HEIGHT, BACKGROUND_COLOR);
}


void staticBits() {
  tft.setTextSize(2);
  tft.setTextColor(TEXT_COLOR);
  tft.setCursor(10, 2);
  tft.print("EMF: ");
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(BACKGROUND_COLOR);
  tft.setTextColor(TEXT_COLOR);

  drawStaticText("Starting ...");
  delay(2000);
  redrawBackground();
  drawBaseline();


  // Initialize with baseline value
  for (int i = 0; i < MAX_DATA_POINTS; i++) {
    dataBuffer.push(50);
  }

  staticBits();
}

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
  tft.fillScreen(BACKGROUND_COLOR);
  drawGrid();
  drawBaseline();
  delay(10);
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


void updateGraph() {
  if (millis() - lastUpdateTime < UPDATE_INTERVAL) return;
  lastUpdateTime = millis();

  float t = millis() / 1000.0;
  float newValue = generateNewValue();
  currentValue = newValue;

  redrawBackground();
  drawCurrentValue(newValue);




  // Draw curved line graph with color gradients
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

  if (dataBuffer.size() == MAX_DATA_POINTS) {
    dataBuffer.shift();
  }
  dataBuffer.push(newValue);

  // Draw Y-axis labels with threshold indicators
  tft.setTextSize(1);
  for (int i = 0; i < sizeof(thresholds) / sizeof(thresholds[0]); i++) {
    int yPos = map(thresholds[i].value, yMin, yMax, GRAPH_Y_OFFSET + GRAPH_HEIGHT, GRAPH_Y_OFFSET);
    tft.setTextColor(thresholds[i].color);
    tft.setCursor(0, yPos - 3);
    tft.print(int(thresholds[i].value));
  }
}

void loop() {
  updateGraph();
}