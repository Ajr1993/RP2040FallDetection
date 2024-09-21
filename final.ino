/*
This software detects a fall using a Threshold based algorithm incoprating a 
Euclidean norm to detect the velocity of a fall. This software incoporates a GUI
for better usbaility. This code is designed to detect falls in those
working in high-risk profession
*/
#include "LCD_Test.h"
#include <math.h>  // For squaring

UWORD *BlackImage;
const float conversion_factor = 3.3f / (1 << 12) * 2;

// Variables for fall detection algorithm
const float HIGHACCTHRESHOLD = 500.0;                // High acceleration threshold
const float LOWACCTHRESHOLD = 100.0;                 // Low acceleration threshold
const float GYROTHRESHOLD = 5;                       // Gyroscope threshold
const unsigned long TIMEWINDOWFALLCONFIRMED = 1000;  // Time window to confirm fall in milliseconds

bool potentialFall = false;
unsigned long startFallTime = 0;
unsigned long fallLength;
unsigned long fallStartTimestamp = 0;
unsigned long fallDetectedTimestamp = 0;
unsigned long StartOfProgramme;

enum Screen {
  SENSOR_SCREEN,
  BATTERY_SCREEN,
  HOME_SCREEN,
};

Screen currentScreen = SENSOR_SCREEN;
const unsigned long changeScreenInterval = 10000;  // Interval to switch screens
unsigned long switchTime = 0;

// Displays the sensor screen
void display_SensorScreen(float acc[], float gyro[]) {
  Paint_Clear(WHITE);
  Paint_DrawString_EN(30, 50, "ACC_X = ", &Font16, WHITE, BLACK);
  Paint_DrawString_EN(30, 75, "ACC_Y = ", &Font16, WHITE, BLACK);
  Paint_DrawString_EN(30, 100, "ACC_Z = ", &Font16, WHITE, BLACK);
  Paint_DrawString_EN(30, 125, "GYR_X = ", &Font16, WHITE, BLACK);
  Paint_DrawString_EN(30, 150, "GYR_Y = ", &Font16, WHITE, BLACK);
  Paint_DrawString_EN(30, 175, "GYR_Z = ", &Font16, WHITE, BLACK);
  Paint_DrawNum(120, 50, acc[0], &Font16, 2, BLACK, WHITE);
  Paint_DrawNum(120, 75, acc[1], &Font16, 2, BLACK, WHITE);
  Paint_DrawNum(120, 100, acc[2], &Font16, 2, BLACK, WHITE);
  Paint_DrawNum(120, 125, gyro[0], &Font16, 2, BLACK, WHITE);
  Paint_DrawNum(120, 150, gyro[1], &Font16, 2, BLACK, WHITE);
  Paint_DrawNum(120, 175, gyro[2], &Font16, 2, BLACK, WHITE);
  LCD_1IN28_Display(BlackImage);
}

// Displays battery screen
void display_BatteryScreen(uint16_t result) {
  Paint_Clear(WHITE);
  Paint_DrawString_EN(50, 110, "BAT(V)=", &Font16, WHITE, BLACK);
  Paint_DrawNum(130, 110, result * conversion_factor, &Font16, 2, BLACK, WHITE);
  LCD_1IN28_Display(BlackImage);
}

// Displays Home Screen
void display_HomeScreen() {
  Paint_Clear(WHITE);
  Paint_DrawString_EN(25, 100, "No Falls Detected :)", &Font20, BLACK, WHITE);
  LCD_1IN28_Display(BlackImage);
}

// Displays image if a fall has been detected
void display_fallDetectedSensorScreen(float acc[], float gyro[]) {
  Paint_Clear(BLUE);  // Changes background colour to blue
  Paint_DrawString_EN(10, 10, "Fall Detected!", &Font20, BLACK, BLUE);
  Paint_DrawString_EN(10, 40, "ACC_X = ", &Font16, WHITE, BLUE);
  Paint_DrawString_EN(10, 60, "ACC_Y = ", &Font16, WHITE, BLUE);
  Paint_DrawString_EN(10, 80, "ACC_Z = ", &Font16, WHITE, BLUE);
  Paint_DrawString_EN(10, 100, "GYR_X = ", &Font16, WHITE, BLUE);
  Paint_DrawString_EN(10, 120, "GYR_Y = ", &Font16, WHITE, BLUE);
  Paint_DrawString_EN(10, 140, "GYR_Z = ", &Font16, WHITE, BLUE);
  Paint_DrawNum(100, 40, acc[0], &Font16, 2, BLACK, BLUE);
  Paint_DrawNum(100, 60, acc[1], &Font16, 2, BLACK, BLUE);
  Paint_DrawNum(100, 80, acc[2], &Font16, 2, BLACK, BLUE);
  Paint_DrawNum(100, 100, gyro[0], &Font16, 2, BLACK, BLUE);
  Paint_DrawNum(100, 120, gyro[1], &Font16, 2, BLACK, BLUE);
  Paint_DrawNum(100, 140, gyro[2], &Font16, 2, BLACK, BLUE);
  Paint_DrawString_EN(10, 160, "Fall Start:", &Font16, WHITE, BLUE);
  Paint_DrawNum(100, 160, (fallStartTimestamp - StartOfProgramme) / 1000, &Font16, 2, WHITE, BLUE);  // Displays start of fall
  Paint_DrawString_EN(10, 180, "Fall Detected:", &Font16, WHITE, BLUE);
  Paint_DrawNum(100, 180, (fallDetectedTimestamp - StartOfProgramme) / 1000, &Font16, 2, WHITE, BLUE);  // Display the time fall is detected
  LCD_1IN28_Display(BlackImage);
}

void display_Algorithm(float acc[], float gyro[]) {
  // Algorithm to detect falls by checking  if total velocity and orientation are greater than threshold values
  // Idea for detecting a potential fall https://github.com/MKShakir/GNG-2101-InflataCare/blob/main/Prototype_3.ino
  float total = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (total > HIGHACCTHRESHOLD && (abs(gyro[0]) > GYROTHRESHOLD || total > HIGHACCTHRESHOLD && abs(gyro[1]) > GYROTHRESHOLD || total > HIGHACCTHRESHOLD && abs(gyro[2]) > GYROTHRESHOLD)) {
    potentialFall = true;
    startFallTime = millis();  // records start time of potential fall
    fallStartTimestamp = startFallTime;
  }

  if (potentialFall && total <= LOWACCTHRESHOLD) {  // Low acceleration threshold
    fallLength = millis() - startFallTime;
    if (fallLength <= TIMEWINDOWFALLCONFIRMED) {
      Serial.print("Fall Detected!");
      fallDetectedTimestamp = millis();
      display_fallDetectedSensorScreen(acc, gyro);
      DEV_Delay_ms(20000);
      potentialFall = false;
      Paint_Clear(WHITE);
      Paint_DrawString_EN(25, 100, "System okay!", &Font20, BLACK, WHITE);
      LCD_1IN28_Display(BlackImage);
      DEV_Delay_ms(5000);
    }
  }

  if (potentialFall && fallLength > TIMEWINDOWFALLCONFIRMED) {
    potentialFall = false;
  }
}
// Idea for my display_ScreenUpdate function https://www.instructables.com/Design-a-Fancy-GUI-for-Your-Project/
void display_ScreenUpdate() {
  if (millis() - switchTime >= changeScreenInterval) {
    // Toggle the screen
    if (currentScreen == SENSOR_SCREEN) {
      currentScreen = BATTERY_SCREEN;
    } else if (currentScreen == BATTERY_SCREEN) {
      currentScreen = HOME_SCREEN;
    } else if (currentScreen == HOME_SCREEN) {
      currentScreen = SENSOR_SCREEN;
    }
    switchTime = millis();
  }
}

// Gyro readings
void display_gyroReadings(float gyro[]) {
  Serial.print("Gyro X: ");
  Serial.println(gyro[0]);
  Serial.println("Gyro Y: ");
  Serial.println(gyro[1]);
  Serial.println("Gyro Z: ");
  Serial.println(gyro[2]);
}
void display_screenReadings(float acc[], float gyro[], uint16_t result) {
  switch (currentScreen) {
    case SENSOR_SCREEN:
      display_SensorScreen(acc, gyro);
      break;

    case BATTERY_SCREEN:
      display_BatteryScreen(result);
      break;

    case HOME_SCREEN:
      display_HomeScreen();
      break;
  }
}

void setup() {
  // Initialise the  LCD screen
  if (DEV_Module_Init() != 0) {
    Serial.println("GPIO Init Fail!");
    return;
  }
  Serial.println("GPIO Init successful!");

  LCD_1IN28_Init(HORIZONTAL);
  DEV_SET_PWM(50);
  LCD_1IN28_Clear(WHITE);

  // Allocate memory to buffer image
  UDOUBLE Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
  BlackImage = (UWORD *)malloc(Imagesize);
  if (BlackImage == NULL) {
    Serial.println("Failed to apply for black memory...");
    exit(0);
  }

  // Create new buffer image on LCD
  Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
  Paint_SetScale(65);
  Paint_SetRotate(ROTATE_0);

  // initialise programme start time to detect falls
  StartOfProgramme = millis();

  // Initial start up image on LCD screen
  Paint_Clear(WHITE);
  Paint_DrawPoint(50, 41, BLACK, DOT_PIXEL_1X1, DOT_FILL_RIGHTUP);
  Paint_DrawPoint(50, 46, BLACK, DOT_PIXEL_2X2, DOT_FILL_RIGHTUP);
  Paint_DrawPoint(50, 51, BLACK, DOT_PIXEL_3X3, DOT_FILL_RIGHTUP);
  Paint_DrawPoint(50, 56, BLACK, DOT_PIXEL_4X4, DOT_FILL_RIGHTUP);
  Paint_DrawPoint(50, 61, BLACK, DOT_PIXEL_5X5, DOT_FILL_RIGHTUP);
  Paint_DrawLine(60, 40, 90, 70, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Paint_DrawLine(60, 70, 90, 40, MAGENTA, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Paint_DrawRectangle(60, 40, 90, 70, RED, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
  Paint_DrawRectangle(100, 40, 130, 70, BLUE, DOT_PIXEL_2X2, DRAW_FILL_FULL);
  Paint_DrawLine(135, 55, 165, 55, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  Paint_DrawLine(150, 40, 150, 70, CYAN, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  Paint_DrawCircle(150, 55, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  Paint_DrawCircle(185, 55, 15, GREEN, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawNum(50, 80, 9.87654321, &Font20, 3, WHITE, BLACK);
  Paint_DrawString_EN(50, 100, "ABC", &Font20, 0x000f, 0xfff0);
  Paint_DrawString_CN(50, 120, "微雪电子", &Font24CN, WHITE, BLUE);
  Paint_DrawString_EN(50, 161, "WaveShare", &Font16, RED, WHITE);
  LCD_1IN28_Display(BlackImage);
  DEV_Delay_ms(2000);

  Serial.println("drawing...\r\n");

  QMI8658_init();
  Serial.println("QMI8658_init\r\n");
  DEV_SET_PWM(100);
}

void loop() {
  float acc[3], gyro[3];
  unsigned int tim_count = 0;
  uint16_t result;
  result = DEC_ADC_Read();
  QMI8658_read_xyz(acc, gyro, &tim_count);
  display_gyroReadings(gyro);
  display_Algorithm(acc, gyro);
  display_ScreenUpdate();
  display_screenReadings(acc, gyro, result);

  DEV_Delay_ms(100);
}