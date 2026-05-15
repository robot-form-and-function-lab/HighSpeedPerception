#include "esp_camera.h"
#include "FS.h"
#include "SPI.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "img_converters.h" // see https://github.com/espressif/esp32-camera/blob/master/conversions/include/img_converters.h

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

const int IMAGE_WIDTH = 96; // set the camera properties to this size in the configure file
const int IMAGE_HEIGHT = 96; // set the camera properties to this size

// Two-dimensional array to hold the pixel values
uint8_t image2D[IMAGE_HEIGHT][IMAGE_WIDTH];
#define THRESHOLD 150 //decimal threshold for white pixel

//for pid
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include <utility/imumaths.h>
#include "driver/ledc.h"

// ---------- Motor PWM pins (SET THESE TO YOUR REAL GPIOs) ----------

const int motor1Pin1 = 1;  // e.g., GPIO01 for Motor 1 AIN1 -> D0
const int motor1Pin2 = 2;  // e.g., GPIO02 for Motor 1 AIN2 -> D1 //ON for forward
const int motor2Pin1 = 3;  // e.g., GPIO03 for Motor 2 BIN1 -> D2
const int motor2Pin2 = 4;  // e.g., GPIO04 for Motor 2 BIN2 -> D3 //ON for forward

// ---------- LEDC PWM config ----------
const int pwmFreq = 1000;      // 1 kHz is fine for DC motor drivers
const int pwmResolution = 10;  // 10-bit → 0..1023
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;

// IMU sample period in microseconds (40 ms = 25 Hz)
const uint32_t BNO_PERIOD_US = 4000;

// --------- Define these for the PID functions -----------
float prevTimeForward = 0;

// ---------- Globals ----------
bool imuOK = false;

// ---------- IMU (BNO055) ----------
#define I2C_SDA 5
#define I2C_SCL 6
#define BNO_ADDRESS_0x28 (0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS_0x28, &Wire);

//----PID CONST ----
volatile float kp = 1023;
volatile float kd = 1000.0; //600 best results so far
volatile float ki = 200.0; //200

//---Yaw Degree ----
volatile float yawDeg;
volatile float initialYaw;

//---- Cumulative Errors terms ----
volatile float error = 0;
volatile float cumError = 0;
volatile float prevError = 0;
volatile float prevPID = 0;

// ---------- Motor helpers ----------
void setupMotors() {
  //condense into ledcAttach
  ledcAttachChannel(motor1Pin1, pwmFreq, pwmResolution, pwmChannel1);
  ledcAttachChannel(motor1Pin2, pwmFreq, pwmResolution, pwmChannel2);
  ledcAttachChannel(motor2Pin1, pwmFreq, pwmResolution, pwmChannel3);
  ledcAttachChannel(motor2Pin2, pwmFreq, pwmResolution, pwmChannel4);
}

// initial variables for optical controller - assumes object is moving center; next is undetermined
 struct Direction {
  int current; 

  int previous; 

  int next; 
} dir;

// PWM parameters
const uint32_t PWM_FREQ    = 5000;  // 5 kHz
const uint8_t  PWM_RES     = 8;     // 0–255 duty resolution

inline void stopMotors() {
  ledcWriteChannel(pwmChannel1, 0);
  ledcWriteChannel(pwmChannel2, 0);
  ledcWriteChannel(pwmChannel3, 0);
  ledcWriteChannel(pwmChannel4, 0);
}

void setup() {
  Serial.begin(115200);
  delay(50);  

  while(!Serial); // When the serial monitor is turned on, the program starts to execute

  camera_config_t config; //setting up configuration 
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_96X96; // be sure to update the WIDTH and HEIGHT of the image! 
  config.pixel_format = PIXFORMAT_GRAYSCALE; // changed to grayscale
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12; //this can be adjusted to create lower or higher quality images
  config.fb_count = 1;
  // camera initialize, will need to remove some of these things for the robot itself
  esp_err_t err = esp_camera_init(&config); 
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Motors
  setupMotors();
  stopMotors();
  struct Direction dir;

  // IMU
  imuOK = initIMU();
}

//make one central turning function
inline void turn(float ref){
  //turn right
  if(ref<0){
    float dt = (micros() - prevTimeForward) / 1000000.0;  //convert this to second
    prevTimeForward = micros();
    float targetAngle = ref;
    error = abs(ref);
    float P = kp * error;
        cumError = cumError + error * dt;
        float I = ki * cumError;
        float D = kd * (error - prevError) / dt;
        float PID = P + I + D;
        prevError = error;
        float motorSpeed = constrain(PID, 800, 1023);
        ledcWriteChannel(pwmChannel1, motorSpeed*.25); // Right forward
        ledcWriteChannel(pwmChannel2, 0);
        ledcWriteChannel(pwmChannel3, motorSpeed);   //left      // Left backward
        ledcWriteChannel(pwmChannel4, 0);
  }
  //turn left
  if(ref > 0){
    float dt = (micros() - prevTimeForward) / 1000000.0;  //convert this to second
    prevTimeForward = micros();
    float targetAngle = ref;
    error = abs(ref);

    float P = kp * error;
        cumError = cumError + error * dt;
        float I = ki * cumError;
        float D = kd * (error - prevError) / dt;
        float PID = P + I + D;
        prevError = error;
        float motorSpeed = constrain(PID, 800, 1023);
        ledcWriteChannel(pwmChannel1, motorSpeed); // Right forward
        ledcWriteChannel(pwmChannel2, 0);
        ledcWriteChannel(pwmChannel3, motorSpeed*.25);         // Left backward
        ledcWriteChannel(pwmChannel4, 0);
  }
}

void loop(){
  //setting up a pointer to the frame buffer
  camera_fb_t * fb = NULL;
  // // Take Picture with camera and put in buffer
  fb = cameraCapture(fb);

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  img_processing_dir(fb);
    sensors_event_t event;
    bno.getEvent(&event);
    float deg = event.orientation.x;
    yawDeg = deg;
  turn(calcRefAngle());
}

camera_fb_t * cameraCapture(camera_fb_t * fb){  
  fb = esp_camera_fb_get();
  return fb;
}

// will update the Direction struct with current and previous directions, reutrns the struct
// it is important to understand that the Direction struct will ONLY contain correct values in here, outside of this function 
// the values could be outdated bc of call by value//call by reference nonsense C partakes in :-[
void img_processing_dir(camera_fb_t * fb) {
  int sec_1_sum = 0;
  int sec_2_sum = 0;
  int sec_3_sum = 0;
  int sec_4_sum = 0;
  int sec_5_sum = 0;
  int sec_6_sum = 0;
  if (fb) {
    // Transfer pixel data from the image buffer to the 2D array
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
      for (int col = 0; col < IMAGE_WIDTH; col++) {
        int index = (row * IMAGE_WIDTH) + col; // Calculate the index in the 1D buffer
        image2D[row][col] = (fb->buf[index] > THRESHOLD) ? 0 : 1;;    // Copy the pixel value to the 2D array and put a 1 if above threshold, otherwise 0
        //dividing image up into thirds, left, center, and right
        if (col < IMAGE_WIDTH/6){
          sec_1_sum += image2D[row][col];
        }
        if (col > IMAGE_WIDTH/6 && col < 2*IMAGE_WIDTH/6){
          sec_2_sum += image2D[row][col];
        }
        if (col > 2*IMAGE_WIDTH/6 && col < 3*IMAGE_WIDTH/6){
          sec_3_sum += image2D[row][col];
        }
        if (col > 3*IMAGE_WIDTH/6 && col < 4*IMAGE_WIDTH/6){
          sec_4_sum += image2D[row][col];
        }
        if (col > 4*IMAGE_WIDTH/6 && col < 5*IMAGE_WIDTH/6){
          sec_5_sum += image2D[row][col];
        }
        if (col > 5*IMAGE_WIDTH/6 && col < IMAGE_WIDTH){
          sec_6_sum += image2D[row][col]; 
        }
      }
    }
  // Release the image buffer
  esp_camera_fb_return(fb);
  }
    int arr[] = {sec_1_sum,sec_2_sum,sec_3_sum,sec_4_sum,sec_5_sum,sec_6_sum}; 
    int max = arr[0];
    int max_idx = 0; 
    for (int i = 0; i < 6; i++) { 
        if (max < arr[i]){
          max = arr[i];
          max_idx = i;
        }
    }
    // now that the section with the highest pixels is identified, assign deirection object us moving 
    dir.current = max_idx;
    chooseNext(dir.current,dir.previous);
    Serial.println(dir.next);
    //use pixel to real world measurement to estimate output reference angle 
    float refAngle = calcRefAngle();
    Serial.println(refAngle);
   // at the end of the loop, the current result becomes the previous 
  dir.previous = dir.current;
}

// the following uses the current and previous values in the Direction struct to determine the next value. The logic can be described in the following: 
void chooseNext( int current, int previous){
    //moving right
    if(previous < current && current+1 <= 5){
      dir.next = current +2;
    }
    //staying in place 
    if(previous == current){
      dir.next = current; 
    }
    //moving left
    if(previous > current && current -1 >= 0){
      dir.next = current -2; 
    }
}

inline float readYawDegrees() {
  sensors_event_t event;
  bno.getEvent(&event);
  float deg = event.orientation.x;
  return deg;
}

// uses the section the object of interest is in a real to image length measurements to calculate a reference angle 
float calcRefAngle(){
  // establish section length in image plane (derrived from pixel meaurement) and focal length. Both in centimeters
  float len_arr[] = {-1.02,-0.68,-0.34,0.34,0.68,1.02}; 
  float focal_length = 13; //in cm

  float quot = len_arr[dir.next]/focal_length; 

  float refAngle_rad = atan(quot);

  // convert to degrees
  const float pi = 3.14159267;
  float refAngle = refAngle_rad / 2 / pi * 360;
  Serial.println("Ref: ");
  Serial.print(refAngle);

  return refAngle;

}


// this function will actually use the predicted directions to assign motor directions to tigerBot


inline float angDiff(float target, float current) {
  float diff = target - current;  // in [−360, +360]
  if (diff > 180.0f) diff -= 360.0f;
  else if (diff <= -180.0f) diff += 360.0f;
  return diff;
}

// ---------- IMU helpers ----------
bool initIMU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected. Check wiring & address (0x28/0x29).");
    return false;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 ready.");
  return true;
}
