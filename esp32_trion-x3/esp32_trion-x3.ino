#include <Wire.h>
#include <Adafruit_GFX.h>      // Core graphics library
#include <Adafruit_ST7735.h>   // For ST7735 display
#include "MPU6050_6Axis_MotionApps20.h" // MPU6050 DMP library from jrowberg/i2cdevlib

/*

MPU6050:-
VCC       ->  3.3V
GND       ->  GND
SDA       ->  GPIO21
SCL       ->  GPIO22

ST7735:-
VCC       ->  3.3V
GND       ->  GND
CS        ->  GPIO5
RST/RES   ->  GPIO4
DC/RS/A0  ->  GPIO2
MOSI/SDA  ->  GPIO23
SCK/SCL   ->  GPIO18
LED       ->  3.3V
*/

// Define ST7735 pins
#define ST7735_CS     5  // Chip select
#define ST7735_RST    4  // Reset
#define ST7735_DC     2  // Data/Command
Adafruit_ST7735 tft = Adafruit_ST7735(ST7735_CS, ST7735_DC, ST7735_RST);

// MPU6050 instance
MPU6050 mpu;

// MPU6050 variables
bool dmpReady = false; // DMP initialization status
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Number of bytes in FIFO buffer
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation data
Quaternion q;           // [w, x, y, z] quaternion container
VectorFloat gravity;    // Gravity vector
float ypr[3];           // [yaw, pitch, roll] in radians

int padX = 22; // X-padding for text
int padY = 5; // Y-padding for text

// Center coordinates for the cube
int xOrigin = 1/2;
int yOrigin = 3/6;

// Cube size and projection scale
float height = 20;
float length = 10;
float breadth = 15;
float ratio = 3;
float fov = breadth * ratio;

// Cube cubeVertices
float cubeVertices[8][3] = {
  {-breadth, -height, -length}, {breadth, -height, -length},
  {breadth, height, -length}, {-breadth, height, -length},
  {-breadth, -height, length}, {breadth, -height, length},
  {breadth, height, length}, {-breadth, height, length}
};
float cubeProjection[8][2] = {0};

// Axes cubeVertices
float axesVertices[3][3] = {
  {-breadth - 10, 0, 0},  // X-axis
  {0, -height - 10, 0},   // Y-axis
  {0, 0, -length - 10}    // Z-axis
};
float axesProjection[3][2] = {0};

float yaw = 0;
float roll = 0;
float pitch = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C

  // Initialize ST7735 display
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0); // Set orientation to portrait
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);

  xOrigin = tft.width() * 1/2;
  yOrigin = tft.height() * 3/6;

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not found, check wiring!");
    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    tft.setCursor(44, tft.height() * 4/9);
    tft.println(" ERROR ");
    tft.setTextColor(ST7735_RED);
    tft.setCursor(10, tft.height() * 5/9);
    tft.println("MPU6050 not found!");
    while (1);
  }

  tft.setTextColor(ST7735_WHITE, 0x08bc);
  tft.setCursor(20, tft.height() * 3/10);
  tft.println(" Startup Info ");
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(13, tft.height() * 4/10);
  tft.println("MPU6050 Detected!");

  // Load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // Check DMP initialization
  if (devStatus != 0) {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    tft.setCursor(44, tft.height() * 4/9);
    tft.println(" ERROR ");
    tft.setTextColor(ST7735_RED);
    tft.setCursor(15, tft.height() * 5/9);
    tft.println("DMP INIT Failed!");
    while (1);
  }

  tft.setTextColor(ST7735_ORANGE);
  tft.setCursor(6, tft.height() * 5/10);
  tft.println("Calibrating MPU6050");

  mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();

  // Enable DMP
  mpu.setDMPEnabled(true);
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

  Serial.println("MPU6050 initialized successfully with DMP.");
  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(12, tft.height() * 6/10);
  tft.println("MPU6050 DMP Ready");
  delay(1500);

  initInfo();
}

void loop() {
  if (!dmpReady) return;

  // Get DMP data
  fifoCount = mpu.getFIFOCount();

  // Check for FIFO overflow
  if ((mpuIntStatus = mpu.getIntStatus()) & 0x10 || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  // Wait for enough data in FIFO
  if (fifoCount < packetSize) return;

  // Read data from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // Get orientation data from DMP
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  

  // Check for motion detection 
  // if (mpu.getIntStatus() & 0x02) { // DMP interrupt bit for motion detection
  //   motionDetected = true;
  // } else {
  //   motionDetected = false;
  // }

  if (detectMotion()) {
    drawCube();
    drawAxes();
    updateRotationInfo();
  }
}

bool detectMotion() {
  bool motion = false;
  // Check and update if any value changed
  if (yaw > ypr[0] + 0.05 || yaw < ypr[0] - 0.05) {
    motion = true;
    yaw = ypr[0];
  }
  if (pitch > ypr[1] + 0.05 || pitch < ypr[1] - 0.05) {
    motion = true;
    pitch = ypr[1];
  }
  if (roll < -ypr[2] + 0.05 || roll > -ypr[2] - 0.05) {
    motion = true;
    roll = -ypr[2];
  }
  
  return motion;
}

void drawCube() {
  // Remove previous cube from screen by drawing black lines over it
  for (int i = 0; i < 4; i++) {
    tft.drawLine(cubeProjection[i][0], cubeProjection[i][1], cubeProjection[(i + 1) % 4][0], cubeProjection[(i + 1) % 4][1], ST7735_BLACK);
    tft.drawLine(cubeProjection[i + 4][0], cubeProjection[i + 4][1], cubeProjection[((i + 1) % 4) + 4][0], cubeProjection[((i + 1) % 4) + 4][1], ST7735_BLACK);
    tft.drawLine(cubeProjection[i][0], cubeProjection[i][1], cubeProjection[i + 4][0], cubeProjection[i + 4][1], ST7735_BLACK);
  }

  // Rotate the cubeVertices based on roll, pitch, and yaw
  for (int i = 0; i < 8; i++) {
    float x = cubeVertices[i][0];
    float y = cubeVertices[i][1];
    float z = cubeVertices[i][2];

    // Rotate around X axis (pitch)
    float tempY = y * cos(pitch) - z * sin(pitch);
    z = y * sin(pitch) + z * cos(pitch);
    y = tempY;

    // Rotate around Y axis (roll)
    float tempX = x * cos(roll) + z * sin(roll);
    z = -x * sin(roll) + z * cos(roll);
    x = tempX;

    // Rotate around Z axis (yaw)
    tempX = x * cos(yaw) - y * sin(yaw);
    y = x * sin(yaw) + y * cos(yaw);
    x = tempX;

    // Apply weak perspective cubeProjection
    cubeProjection[i][0] = xOrigin + (fov * x) / (fov + z);
    cubeProjection[i][1] = yOrigin + (fov * y) / (fov + z);
  }

  // Draw cube edges
  for (int i = 0; i < 4; i++) {
    tft.drawLine(cubeProjection[i][0], cubeProjection[i][1], cubeProjection[(i + 1) % 4][0], cubeProjection[(i + 1) % 4][1], ST7735_WHITE);
    tft.drawLine(cubeProjection[i + 4][0], cubeProjection[i + 4][1], cubeProjection[((i + 1) % 4) + 4][0], cubeProjection[((i + 1) % 4) + 4][1], ST7735_WHITE);
    tft.drawLine(cubeProjection[i][0], cubeProjection[i][1], cubeProjection[i + 4][0], cubeProjection[i + 4][1], ST7735_WHITE);
  }
}

void drawAxes() {
  // Remove previous axes from screen by drawing black lines over it
  tft.drawLine(xOrigin, yOrigin, axesProjection[0][0], axesProjection[0][1], ST7735_BLACK);
  tft.drawChar(axesProjection[0][0] - 8, axesProjection[0][1] - 3, 'X', ST7735_BLACK, ST7735_BLACK, 1);
  tft.drawLine(xOrigin, yOrigin, axesProjection[1][0], axesProjection[1][1], ST7735_BLACK);
  tft.drawChar(axesProjection[1][0] - 2, axesProjection[1][1] - 10, 'Y', ST7735_BLACK, ST7735_BLACK, 1);
  tft.drawLine(xOrigin, yOrigin, axesProjection[2][0], axesProjection[2][1], ST7735_BLACK);
  tft.drawChar(axesProjection[2][0] + 3, axesProjection[2][1] + 3, 'Z', ST7735_BLACK, ST7735_BLACK, 1);

  // Rotate the axes vertices based on roll, pitch, and yaw
  for (int i = 0; i < 3; i++) {
    float x = axesVertices[i][0];
    float y = axesVertices[i][1];
    float z = axesVertices[i][2];

    // Rotate around X axis (pitch)
    float tempY = y * cos(pitch) - z * sin(pitch);
    z = y * sin(pitch) + z * cos(pitch);
    y = tempY;

    // Rotate around Y axis (roll)
    float tempX = x * cos(roll) + z * sin(roll);
    z = -x * sin(roll) + z * cos(roll);
    x = tempX;

    // Rotate around Z axis (yaw)
    tempX = x * cos(yaw) - y * sin(yaw);
    y = x * sin(yaw) + y * cos(yaw);
    x = tempX;

    // Apply weak perspective cubeProjection
    axesProjection[i][0] = xOrigin + (fov * x) / (fov + z);
    axesProjection[i][1] = yOrigin + (fov * y) / (fov + z);
  }

  // Draw X-axis
  tft.drawLine(xOrigin, yOrigin, axesProjection[0][0], axesProjection[0][1], ST7735_RED);
  tft.drawChar(axesProjection[0][0] - 8, axesProjection[0][1] - 3, 'X', ST7735_RED, ST7735_BLACK, 1);
  // Draw Y-axis
  tft.drawLine(xOrigin, yOrigin, axesProjection[1][0], axesProjection[1][1], ST7735_GREEN);
  tft.drawChar(axesProjection[1][0] - 2, axesProjection[1][1] - 10, 'Y', ST7735_GREEN, ST7735_BLACK, 1);
  // Draw Z-axis
  tft.drawLine(xOrigin, yOrigin, axesProjection[2][0], axesProjection[2][1], ST7735_BLUE);
  tft.drawChar(axesProjection[2][0] + 3, axesProjection[2][1] + 3, 'Z', ST7735_BLUE, ST7735_BLACK, 1);
}

void updateRotationInfo() {
  int yPos = tft.height() - 35;
  int xPos = padX + 42;
  tft.setTextSize(1);
  tft.setCursor(xPos, yPos);
  tft.fillRect(xPos, yPos, 100, 10, ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.print(radianToDegree(yaw), 1);
  tft.print(" deg");

  yPos += 10;
  tft.setCursor(xPos, yPos);
  tft.fillRect(xPos, yPos, 100, 10, ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.print(radianToDegree(roll), 1);
  tft.print(" deg");

  yPos += 10;
  tft.setCursor(xPos, yPos);
  tft.fillRect(xPos, yPos, 100, 10, ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.print(radianToDegree(pitch), 1);
  tft.print(" deg");
}

void initInfo() {
  tft.fillScreen(ST7735_BLACK);

  // Display Title
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_RED);
  tft.setCursor(8, padY);
  tft.println(" 3-Axis Orientaion ");
  tft.setCursor(32, padY + 10); 
  tft.println(" Visualizer ");

  int yPos = tft.height() - 35;
  tft.setCursor(padX, yPos);
  tft.setTextColor(ST7735_BLUE);
  tft.print("Yaw:   ");
  tft.setTextColor(ST7735_WHITE);
  tft.print("0.0 deg");

  yPos += 10;
  tft.setCursor(padX, yPos);
  tft.setTextColor(ST7735_GREEN);
  tft.print("Roll:  ");
  tft.setTextColor(ST7735_WHITE);
  tft.print("0.0 deg");

  yPos += 10;
  tft.setCursor(padX, yPos);
  tft.setTextColor(ST7735_RED);
  tft.print("Pitch: ");
  tft.setTextColor(ST7735_WHITE);
  tft.print("0.0 deg");
}

float radianToDegree(float rad) {
  return rad * 180 / M_PI;
}