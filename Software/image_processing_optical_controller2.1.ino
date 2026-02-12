#include "esp_camera.h"
#include "FS.h"
//#include "SD.h"
#include "SPI.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "img_converters.h" // see https://github.com/espressif/esp32-camera/blob/master/conversions/include/img_converters.h
//#include "driver/ledc.h"

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
//#include "camera_pins.h"

const int IMAGE_WIDTH = 96; // set the camera properties to this size in the configure file
const int IMAGE_HEIGHT = 96; // set the camera properties to this size

// Two-dimensional array to hold the pixel values
uint8_t image2D[IMAGE_HEIGHT][IMAGE_WIDTH];
//uint8_t image2dr[IMAGE_HEIGHT][IMAGE_WIDTH]; //two array to make a binary image file
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


float ref=30;
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

int motorSpeed1 = 500;  // duty (0..1023). Tune as needed.
int motorSpeed2 = 500;  // duty (0..1023). Tune as needed.

//------------bounds for the pwm---------------//
int lowerbound1 = 300;
int upperbound1 = 400;
int lowerbound2 = 123;
int upperbound2 = 500;

//---------------Throttle----------------------//
int throttle1 = 1000;
int throttle2 = 1000;

// IMU sample period in microseconds (40 ms = 25 Hz)
const uint32_t BNO_PERIOD_US = 4000;

typedef struct pid_struct {
  float kp;
  float ki;
  float kd;
  bool remote_enable;
} pid_struct;

// ---------- Shared state (callback -> control loop) ----------
volatile int g_cmd = 0;  // latest command (atomic-sized)
volatile bool g_pidUpdated = false;
volatile unsigned long g_lastCmdUs = 0;  // time (us) we last saw a command
pid_struct g_pid;                        // last PID packet

// ---------- Control loop config (micros) ----------
const uint32_t CONTROL_HZ = 100;                            // 100 Hz motor update
const uint32_t CONTROL_PERIOD_US = 1000000UL / CONTROL_HZ;  // 1 million microsecond is equal to 1 second
unsigned long lastControlTickUs = 0;                        // Keep track of the total micro-second past for motors
unsigned long lastIMUSampleUs = 0;                          // Keep track of the total micro-second past for IMU
int lastAppliedCmd = -1;                                    // track to avoid redundant writes


// --------- Define these for the PID functions -----------

float prevTimeForward = 0;
float prevTimeBackward = 0;

int lastCmd = -1;

// ---------- Globals ----------
bool imuOK = false;

// ---------- IMU (BNO055) ----------
#define I2C_SDA 5
#define I2C_SCL 6
#define BNO_ADDRESS_0x28 (0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS_0x28, &Wire);
//imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);


//----PID CONST ----
volatile float kp;
volatile float kd;
volatile float ki;

//---Yaw Degree ----
volatile float yawDeg;
volatile float initialYaw;

//---- Cumulative Errors terms ----
volatile float error = 0;
volatile float cumError = 0;
volatile float prevError = 0;

// ---------- ESP-NOW command / PID packets ----------
typedef struct command_struct {
  int command_type;  // 0=STOP, 1=FORWARD, 2=BACKWARD, 3=LEFT, 4=RIGHT
} command_struct;

// ---------- ESP-NOW telemetry (to peer) ----------

//YOU CAN ADD MORE IF YOU WANT TO RECIEVE MORE INFORMATION
typedef struct send_struct {
  float yaw;  // yaw in degrees
} send_struct;
send_struct sendData;

// ---------- ESP-NOW peer MAC (REPLACE) ----------
uint8_t receiverMac[6] = { 0xD8, 0x3B, 0xDA, 0x46, 0x6A, 0xB4 };  // PUT REAL MAC

// ---------- Motor helpers ----------
void setupMotors() {
  // ledcAttachPin configures the GPIO mux; pinMode() is not needed for PWM.
  /*ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel3, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel4, pwmFreq, pwmResolution);

  ledcAttachPin(motor1Pin1, pwmChannel1);
  ledcAttachPin(motor1Pin2, pwmChannel2);
  ledcAttachPin(motor2Pin1, pwmChannel3);
  ledcAttachPin(motor2Pin2, pwmChannel4);
  */

  //condense into ledcAttach
  ledcAttachChannel(motor1Pin1, pwmFreq, pwmResolution, pwmChannel1);
  ledcAttachChannel(motor1Pin2, pwmFreq, pwmResolution, pwmChannel2);
  ledcAttachChannel(motor2Pin1, pwmFreq, pwmResolution, pwmChannel3);
  ledcAttachChannel(motor2Pin2, pwmFreq, pwmResolution, pwmChannel4);
}



/**
// our call back to dump whatever we got in binary format, this is used with CoolTerm on my machine to capture an image
size_t jpgCallBack(void * arg, size_t index, const void* data, size_t len)
{
  uint8_t* basePtr = (uint8_t*) data;
  for (size_t i = 0; i < len; i++) {
    Serial.write(basePtr[i]);
  }
  return 0;
}
***/




// initial variables for optical controller - assumes object is moving center; next is undetermined
 struct Direction {
  char current; //= 'C'; 

  char previous; // = 'C';

  char next; // = '?'; // need to add logic for ? behavior.. what should it be? 
} dir;






// --- pin definitions ---
const uint8_t A_IN1 = 1;  // Motor A forward
const uint8_t A_IN2 = 2;  // Motor A reverse
const uint8_t B_IN1 = 3;  // Motor B forward
const uint8_t B_IN2 = 4;  // Motor B reverse



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
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  delay(50);

  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

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


  //pinMode(D0, OUTPUT);
  //pinMode(D1, OUTPUT);
  //pinMode(D2, OUTPUT);
  //pinMode(D3, OUTPUT);




 // configure all four DRV8833 inputs in one step each:
  // ledcAttach(motor1Pin1, pwmFreq, pwmResolution);  // automatically allocates a channel+timer
  // ledcAttach(motor1Pin2, pwmFreq, pwmResolution);
  // ledcAttach(motor2Pin1, pwmFreq, pwmResolution);
  // ledcAttach(motor2Pin2, pwmFreq, pwmResolution);
  //ledcAttachChannel(motor1Pin1, pwmFreq, pwmResolution, pwmChannel1);



 
  // camera initialize, will need to remove some of these things for the robot itself
  // esp_err_t err = esp_camera_init(&config); 
  // if (err != ESP_OK) {
  //   Serial.printf("Camera init failed with error 0x%x", err);
  //   return;
  // }

  //gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

  // Motors
  setupMotors();
  //stopMotors();

  struct Direction dir;

  // IMU
  //imuOK = initIMU();
  dir.current = 'C';
  dir.previous = 'C'; 
  dir.next = '?'; 


}

//make one central turning function
inline void turn(float ref){
  //printf("Yaw angle to see if it is updating: %.2f\n\n", yawDeg);
  float diff = angDiff(ref, yawDeg);
  
  //turn right
  if(diff<0){
    float dt = (micros() - prevTimeForward) / 1000000.0;  //convert this to second
    prevTimeForward = micros();
    float targetAngle = ref;
    // float targetAngle = initialYaw + 90;
    diff = angDiff(targetAngle, yawDeg);
    printf("Diff: %.2f\n", diff);
    //error = diff;
    //printf("Error: %.2f\n", error);
    float P = kp * error;
        cumError = cumError + error * dt;
        cumError = constrain(cumError, -50, 50);  //This is the error for the Intergral term
        float I = ki * cumError;
        float D = kd * (error - prevError) / dt;
        float PID = P + I + D;
        float motorSpeed = map(error, 0, 179, 500, 1023);
        printf("MotorSpeed: %.2f", motorSpeed);
        //motorSpeed1 = constrain(throttle1 - PID, lowerbound1, upperbound1);
        //motorSpeed2 = constrain(throttle2 + PID, lowerbound2, upperbound2);
        // analogWrite(D0,motorSpeed);
        // analogWrite(D1,0);
        // analogWrite(D2,0);
        // analogWrite(D3,motorSpeed);
        
        //take reference angle and add it to current yaw angle- that is the new angle we need to get to 


        ledcWriteChannel(pwmChannel1, 0); // Right forward
        ledcWriteChannel(pwmChannel2, motorSpeed);
        ledcWriteChannel(pwmChannel3, motorSpeed);         // Left backward
        ledcWriteChannel(pwmChannel4, 0);
  }
  //turn left
  if(diff>0){
    //printf("Yaw angle to see if it is updating: %.2f\n\n", yawDeg);
    //diff = angDiff(ref, yawDeg);
    float dt = (micros() - prevTimeForward) / 1000000.0;  //convert this to second
    prevTimeForward = micros();
    float targetAngle = ref;
    // float targetAngle = initialYaw + 90;
    diff = angDiff(targetAngle, yawDeg);
    printf("Diff: %.2f\n", diff);

    //error = diff;
    printf("Error: %.2f\n", error);

    float P = kp * error;
        cumError = cumError + error * dt;
        cumError = constrain(cumError, -50, 50);  //This is the error for the Intergral term
        float I = ki * cumError;
        float D = kd * (error - prevError) / dt;
        float PID = P + I + D;
        float motorSpeed = map(error, 0, 179, 500, 1023);
        printf("MotorSpeed: %.2f", motorSpeed);

        //motorSpeed1 = constrain(throttle1 - PID, lowerbound1, upperbound1);
        //motorSpeed2 = constrain(throttle2 + PID, lowerbound2, upperbound2);
        // analogWrite(D0,0);
        // analogWrite(D1,motorSpeed);
        // analogWrite(D2,motorSpeed);
        // analogWrite(D3,0);
        ledcWriteChannel(pwmChannel1, motorSpeed); // Right forward
        ledcWriteChannel(pwmChannel2, 0);
        ledcWriteChannel(pwmChannel3, 0);         // Left backward
        ledcWriteChannel(pwmChannel4, motorSpeed);
  }
  //go straight
  else if(error<10 && error>-1){
    stopMotors();
    float dt = (micros() - prevTimeForward) / 1000000.0;  //convert this to second
    prevTimeForward = micros();
    float targetAngle = ref;
    // float targetAngle = initialYaw + 90;
    diff = angDiff(yawDeg, yawDeg);

    //error = diff;
    //printf("Error: %.2f\n", error);

    float P = kp * error;
        cumError = cumError + error * dt;
        cumError = constrain(cumError, -50, 50);  //This is the error for the Intergral term
        float I = ki * cumError;
        float D = kd * (error - prevError) / dt;
        float PID = P + I + D;
        float motorSpeed = map(error, 0, 179, 0, 1023);
        //motorSpeed1 = constrain(throttle1 - PID, lowerbound1, upperbound1);
        //motorSpeed2 = constrain(throttle2 + PID, lowerbound2, upperbound2);
        analogWrite(D0,0);
        analogWrite(D1, motorSpeed);
        analogWrite(D2, motorSpeed);
        analogWrite(D3,0);
        // ledcWrite(pwmChannel1, motorSpeed);  //right motor (battery connector indiates the head of robot)
        // ledcWrite(pwmChannel2, 0);
        // ledcWrite(pwmChannel3, motorSpeed);  //left motor
        // ledcWrite(pwmChannel4, 0);
  }
}

float getErr(){
  //error = ref-yawDeg;
  /*if(((yawDeg-error)*-1) > ref-yawDeg){
    error = 360-ref-yawDeg;
    return error;
  }
  else if(ref-yawDeg>180.0){
    error = ref-yawDeg-360.0f;
  }
  else{
    error = ref-yawDeg;
    return error;
  }
  */
  error = angDiff(ref, yawDeg);
  if(error<0){
    return error*=-1;
    //return error;
  }
  return error;
}

inline void turnLeft() {
  // Left wheel backward, right wheel forward
  //   ledcWrite(pwmChannel1, 1000);
  //   ledcWrite(pwmChannel2, 0);
  //   ledcWrite(pwmChannel3, 200);
  //   ledcWrite(pwmChannel4, 0);

  printf("Yaw angle to see if it is updating: %.2f\n\n", yawDeg);
  printf("Initial angle to see if it is updated: %.2f\n", initialYaw);

  float diff = angDiff(30, yawDeg); // Returns positive for left turns
  //float diff = angDiff(initialYaw-30, yawDeg); // Returns positive for left turns

  // Target is +90 degrees from start
  if (diff < 30.0) {
    //analogWrite()
      // ledcWrite(pwmChannel1, 1023); // Right forward
      // ledcWrite(pwmChannel2, 0);
      // ledcWrite(pwmChannel3, 0);         // Left backward
      // ledcWrite(pwmChannel4, 1023);
  } else {
      stopMotors();
      //g_cmd = 0; // FORCE STATE TO STOP
  }

  /*
  if(yawDeg!=target){
    *go
  }
  else{
    stopMotors();
  }
  */
}

inline void turnRight() {

  //float diff = angDiff(targetAngle, yawDeg);
  float diff = angDiff(initialYaw, yawDeg); // Returns negative for right turns

  printf("Yaw angle to see if it is updating: %.2f\n\n", yawDeg);
  printf("Initial angle to see if it is updated: %.2f\n", initialYaw);
  //motorSpeed1 = constrain(throttle1 + PID, lowerbound1, upperbound1);
  //motorSpeed2 = constrain(throttle2 - PID, lowerbound2, upperbound2);

  // motorSpeed1 = constrain(throttle1 + PID, 500, 1023);
  // motorSpeed2 = constrain(throttle2 - PID, 500, 1023);
  //prevError = error;

    // Target is -90 degrees from start
    if (diff > -30.0) {
      //if(diff!=0){
        float dt = (micros() - prevTimeForward) / 1000000.0;  //convert this to second
        prevTimeForward = micros();
        float targetAngle = initialYaw + 90;
        float diff = angDiff(targetAngle, yawDeg);

        error = yawDeg-targetAngle;
        float P = kp * error;
        cumError = cumError + error * dt;
        cumError = constrain(cumError, -50, 50);  //This is the error for the Intergral term
        float I = ki * cumError;
        float D = kd * (error - prevError) / dt;
        float PID = P + I + D;
        motorSpeed1 = constrain(throttle1 - PID, lowerbound1, upperbound1);
        motorSpeed2 = constrain(throttle2 + PID, lowerbound2, upperbound2);
        ledcWrite(pwmChannel1, 0);         // Right backward
        //ledcWrite(pwmChannel2, speed1);
        //ledcWrite(pwmChannel3, speed2);
        ledcWrite(pwmChannel2, motorSpeed1);
        ledcWrite(pwmChannel3, motorSpeed2);
        //ledcWrite(pwmChannel2, 1023);
        //ledcWrite(pwmChannel3, 1023); // Left forward
        ledcWrite(pwmChannel4, 0);

    } else {
        stopMotors();
        //g_cmd = 0; // FORCE STATE TO STOP
    }
}

void loop(){
  error = getErr();
  //ledcWrite(pwmChannel1, 0); // Right forward
  // ledcWrite(pwmChannel2, 500);
  // ledcWrite(pwmChannel3, 500);         // Left backward
  // //ledcWrite(pwmChannel4, 0);
  // delay(200);

  // Serial.println("Loop gets here");
  // analogWrite(D0, 255);
  // analogWrite(D2, 255);
  // delay(200);

  //delay(100);
  //error = ref-yawDeg;
  //printf("Error: %.2f\n", error);
  turn(ref); //ref = 30
  // setting up a pointer to the frame buffer
  //camera_fb_t * fb = NULL;
  // Take Picture with camera and put in buffer


  //fb = esp_camera_fb_get(); 

 
  

  //gpio_set_level(GPIO_NUM_4, 1);
  //delay(1000);
  //gpio_set_level(GPIO_NUM_4, 0);

  //fb = esp_camera_fb_get(); // when placed in a function, it does not work :( 
  //gpio_set_level(GPIO_NUM_4, 0);
  //fb = cameraCapture(fb);
  //gpio_set_level(GPIO_NUM_4, 0);
  

  /*if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }*/


  

  //gpio_set_level(GPIO_NUM_4, 0);
  //img_processing_dir(fb, dir);
  //gpio_set_level(GPIO_NUM_4, 0); 

  //delay(3000);
  //backwards();
  //left();
  sensors_event_t event;
  bno.getEvent(&event);
//   //Serial.print(event.orientation.x-180);
  float deg = event.orientation.x;
  yawDeg = deg-180;
//  //yawDeg = readYawDegrees();
  Serial.print("Yaw: ");
//  Serial.println(event.orientation.x, 2);
  Serial.println(yawDeg);
//  //printf("Yaw: %0.2f\n", yawDeg);
  delay(100);
//turnRight();
//moveForward();
//turnLeft();
//turn(30.0);
}

camera_fb_t * cameraCapture(camera_fb_t * fb){
  
  //gpio_set_level(GPIO_NUM_4, 1);
  fb = esp_camera_fb_get();
  //gpio_set_level(GPIO_NUM_4, 1);
  esp_camera_fb_return(fb);
  return fb;
  
}

// will update the Direction struct with current and previous directions, reutrns the struct
// it is important to understand that the Direction struct will ONLY contain correct values in here, outside of this function 
// the values could be outdated bc of call by value//call by reference nonsense C partakes in :-[
void img_processing_dir(camera_fb_t * fb) {
  int leftSum = 0;
  int centerSum = 0;
  int rightSum = 0;

  //gpio_set_level(GPIO_NUM_4, 1); // for timing measurement 

  if (fb) {
    // Transfer pixel data from the image buffer to the 2D array
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
      for (int col = 0; col < IMAGE_WIDTH; col++) {
        
        int index = (row * IMAGE_WIDTH) + col; // Calculate the index in the 1D buffer
        image2D[row][col] = (fb->buf[index] > THRESHOLD) ? 1 : 0;;    // Copy the pixel value to the 2D array and put a 1 if above threshold, otherwise 0
        
        // the following prints the binary image values 
        //image2dr[row][col] = fb->buf[index]; 
        //Serial.print(image2D[row][col]);
        //Serial.print(" ");
        
        //dividing image up into thirds, left, center, and right
        if (col < IMAGE_WIDTH/3){
          leftSum += image2D[row][col];
        }
        if (col > IMAGE_WIDTH/3 && col < 2*IMAGE_WIDTH/3){
          centerSum += image2D[row][col];
        }
        if (col > 2*IMAGE_WIDTH/3 && col < IMAGE_WIDTH){
          rightSum += image2D[row][col];
        }
      }
      //Serial.println(" ");
      
    }

  // Release the image buffer
  esp_camera_fb_return(fb);

  
  }
  // the following is the logic for determining the current direction. based on separating the image into thirds 

  if(centerSum >= leftSum && centerSum >= rightSum){
    //Serial.println("both motors on");
    dir.current = 'C';
    }
  if(centerSum <= leftSum && leftSum >= rightSum){
    //Serial.println("left motor on");
    dir.current = 'L';
    }
  if(rightSum >= leftSum && leftSum <= rightSum){
    //Serial.println("right motor on");
    dir.current = 'R';
    }
  //else {Serial.println("both motors on");} // this is where the (cant find it) behavior will lie 

  // at this point, the next direction should be calculated

  chooseNext();

  // actually drive tigerBot
  //gpio_set_level(GPIO_NUM_4, 0);
  //pilot(dir);
  //right();
  //gpio_set_level(GPIO_NUM_4, 0);


  

  
  
    dir.previous = dir.current;
    
    Serial.println("currently moving: " );
    Serial.println(dir.current);  

    Serial.println("previously moving: " );
    Serial.println(dir.previous);  

    Serial.println("most likely moving: " );
    Serial.println(dir.next);  

    
  
  // at the end of the loop, the current result becomes the previous 
 

  

}




// the following uses the current and previous values in the Direction struct to determine the next value. The logic can be described in the following: 
// <insert table here> 
void chooseNext(){

  
  //column 1
    if(dir.current == 'L' && dir.previous == 'L'){
      dir.next = 'L';
    }
          
    if(dir.current == 'L' && dir.previous == 'C'){
      dir.next = 'L'; 
    }
        
     
    if(dir.current == 'L' && dir.previous == 'R'){
       dir.next = 'L'; 
    }
       
    //column 2
    if(dir.current == 'C' && dir.previous == 'L'){ 
      dir.next = 'R';
    }
        
    

    if(dir.current == 'C' && dir.previous == 'C'){
        dir.next = 'C'; 
    }

    if(dir.current == 'C' && dir.previous == 'R'){
        dir.next = 'L'; 
    }


    //column 3
    if(dir.current == 'R' && dir.previous == 'L'){
        dir.next = 'R'; 
    } 

    if(dir.current == 'R' && dir.previous == 'C'){
        dir.next = 'R'; 
    } 

    if(dir.current == 'R' && dir.previous == 'R'){
        dir.next = 'R'; 
    } 

    
  
    
    

}

void printCalStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  // Serial.printf("Calib(sys,gyro,accel,mag)=(%u,%u,%u,%u)\n", sys, gyro, accel, mag);
}

inline float readYawDegrees() {
  //imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x-180;

  //return e.x() - 180;  // 0..360 -> -180 to 180
  // return e.x(); // 0..360 -> -180 to 180
}



void pilot(){

  //gpio_set_level(GPIO_NUM_4, 1);
  /**

  if(dir.next == 'R'){
      right();
      //stop();
  }
  
  
  
  if(dir.next == 'C'){
    //forward();
    //Serial.println("going forward");
    stopMotors();
  }


  if(dir.next == 'L'){
    //left();
    Serial.println("moving left");
    //stop();
  }

  
  if(dir.next == 'L'){
    left();
    //stop();
  }
  else{
    //stop();
    Serial.println("stationary");
  }
  */
  
  // ---- IMU sample & send (25 Hz by default) ----
  if (imuOK && (micros() - lastIMUSampleUs) >= BNO_PERIOD_US) {
    lastIMUSampleUs = micros();
  
    yawDeg = readYawDegrees();
    sendData.yaw = yawDeg;

    esp_err_t rc = esp_now_send(receiverMac, (uint8_t *)&sendData, sizeof(sendData));
    if (rc != ESP_OK) {
      // Keep prints modest
      // Serial.printf("[ESP-NOW] send err=%d\n", rc);
    }

    static uint8_t calDiv = 0;
    if ((calDiv++ % 10) == 0) {  // print occasionally
      // Serial.printf("Yaw(deg)=%.2f\n", yawDeg);
      printCalStatus();
    }
  }

  // ---- Motor control ticker (100 Hz by default) ----
  if ((micros() - lastControlTickUs) >= CONTROL_PERIOD_US) {
    lastControlTickUs = micros();

    if (g_cmd != lastCmd) {
      initialYaw = yawDeg;  // CAPTURE STARTING POINT ONCE
      resetErrors();
      lastCmd = g_cmd;
    }

    switch (g_cmd) {
      case 0: stopMotors(); break;
      case 1: moveForward(); break;
      case 2: moveBackward(); break;
      case 3: turnLeft(); break;
      case 4: turnRight(); break;
      default: stopMotors(); break;
    }

    // Apply any pending PID updates here (not in callback)
    if (g_pidUpdated) {

      pid_struct loc = g_pid;
      g_pidUpdated = false;

      if (loc.remote_enable) {
        kp = loc.kp;
        ki = loc.ki;
        kd = loc.kd;
      }
      Serial.printf("[PID] kp=%.3f ki=%.3f kd=%.3f remote=%d\n",
                    loc.kp, loc.ki, loc.kd, loc.remote_enable ? 1 : 0);

      //proportional determines which direction to go into
      //derivative corrects overshoot by overcompensating in the opposite direction
      //integral winds up error for precision
    }
  }




}

// this function will actually use the predicted directions to assign motor directions to tigerBot


inline float angDiff(float target, float current) {
  float diff = target - current;  // in [−360, +360]
  if (diff > 180.0f) diff -= 360.0f;
  else if (diff <= -180.0f) diff += 360.0f;
  return diff;
}

inline void moveForward() {
  //need to have the yaw angle updated before this is called

  printf("Yaw angle to see if it is updating: %.2f\n\n", yawDeg);
  printf("Initial angle to see if it is updated: %.2f\n", initialYaw);

  //PID calculation
  float dt = (micros() - prevTimeForward) / 1000000.0;  //convert this to second
  prevTimeForward = micros();
  error = angDiff(yawDeg, yawDeg);
  float P = kp * error;
  cumError = cumError + error * dt;
  cumError = constrain(cumError, -50, 50);  //This is the error for the Intergral term
  float I = ki * cumError;
  float D = kd * (error - prevError) / dt;
  float PID = P + I + D;
  motorSpeed1 = constrain(throttle1 + PID, lowerbound1, upperbound1);
  motorSpeed2 = constrain(throttle2 - PID, lowerbound2, upperbound2);
  prevError = error;

  //left 700, and right 350 the one on the right dominates -> goes pretty straight
  //left 750, and right 350 the one on the left dominates -> goes pretty straight

  // AIN1=PWM, AIN2=0; BIN1=PWM, BIN2=0  (adjust if your H-bridge expects opposite polarity)
  //keep motorSpeed2 at 750
  //find motorSpeed1 such that it is slower than the left going at 350
  ledcWrite(pwmChannel1, motorSpeed2);  //right motor (battery connector indiates the head of robot)
  ledcWrite(pwmChannel2, 0);
  ledcWrite(pwmChannel3, motorSpeed1);  //left motor
  ledcWrite(pwmChannel4, 0);
}

inline void moveBackward() {

  //implement the PID control for this one

  //PID calculation
  float dt = (micros() - prevTimeBackward) / 1000000.0;  //convert this to second
  prevTimeBackward = micros();
  error = angDiff(initialYaw, yawDeg);
  float P = kp * error;
  cumError = cumError + error * dt;
  cumError = constrain(cumError, -50, 50);  //This is the error for the Intergral term
  float I = ki * cumError;
  float D = kd * (error - prevError) / dt;
  float PID = P + I + D;

  //inverse the -+ because it is now driving backwards
  motorSpeed1 = constrain(throttle1 - PID, lowerbound1, upperbound1);
  motorSpeed2 = constrain(throttle2 + PID, lowerbound2, upperbound2);

  prevError = error;

  //right and left 500 pretty straight

  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, motorSpeed1);
  ledcWrite(pwmChannel3, 0);
  ledcWrite(pwmChannel4, motorSpeed2);
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

inline void resetErrors() {

  error = 0;
  cumError = 0;
  prevError = 0;
}

/*void forward(){
  //digitalWrite(D1, HIGH);
  setMotorA(-170);
  setMotorB(-170);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, HIGH);
  //digitalWrite(D2, LOW);
}
void backwards(){
  //digitalWrite(D1, LOW);
  //digitalWrite(D0, HIGH);
  //digitalWrite(D3, LOW);
  //digitalWrite(D2, HIGH);
  setMotorA(120);
  setMotorB(120);
}
void left(){
  //digitalWrite(D1, LOW);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, HIGH);
  //digitalWrite(D2, LOW);
  setMotorA(140);
  setMotorB(180);
}
void right(){
  //digitalWrite(D1, HIGH);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, LOW);
  //digitalWrite(D2, LOW);
  setMotorA(90);
  setMotorB(180);
}
void stop(){
  //digitalWrite(D1, LOW);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, LOW);
  //digitalWrite(D2, LOW);
  setMotorA(0);
  setMotorB(0);
}
*/

//void back_left(){
  //digitalWrite(D0, HIGH);
  //digitalWrite(D1, LOW);
  //digitalWrite(D2, LOW);
  //digitalWrite(D3, LOW);
//}




void setMotorA(int16_t speed) {
  if (speed >= 0) {
    ledcWrite(A_IN1, speed);
    ledcWrite(A_IN2, 0);
  } else {
    ledcWrite(A_IN1, 0);
    ledcWrite(A_IN2, -speed);
  }
}

void setMotorB(int16_t speed) {
  if (speed >= 0) {
    ledcWrite(B_IN1, speed);
    ledcWrite(B_IN2, 0);
  } else {
    ledcWrite(B_IN1, 0);
    ledcWrite(B_IN2, -speed);
  }
}








