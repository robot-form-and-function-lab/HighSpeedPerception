#include "esp_camera.h"
#include "FS.h"
//#include "SD.h"
#include "SPI.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "img_converters.h" // see https://github.com/espressif/esp32-camera/blob/master/conversions/include/img_converters.h

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

const int IMAGE_WIDTH = 96; // set the camera properties to this size in the configure file
const int IMAGE_HEIGHT = 96; // set the camera properties to this size

// Two-dimensional array to hold the pixel values
uint8_t image2D[IMAGE_HEIGHT][IMAGE_WIDTH];
//uint8_t image2dr[IMAGE_HEIGHT][IMAGE_WIDTH]; //two array to make a binary image file
#define THRESHOLD 150 //decimal threshold for white pixel




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



void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  
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
  ledcAttach(A_IN1, PWM_FREQ, PWM_RES);  // automatically allocates a channel+timer
  ledcAttach(A_IN2, PWM_FREQ, PWM_RES);
  ledcAttach(B_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(B_IN2, PWM_FREQ, PWM_RES);



 
  // camera initialize, will need to remove some of these things for the robot itself
  esp_err_t err = esp_camera_init(&config); 
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

  struct Direction dir;
  dir.current = 'C';
  dir.previous = 'C'; 
  dir.next = '?'; 

  


  
 


}

void loop(){
  // setting up a pointer to the frame buffer
  camera_fb_t * fb = NULL;
  
  // Take Picture with camera and put in buffer


  //fb = esp_camera_fb_get(); 

 
  

  //gpio_set_level(GPIO_NUM_4, 1);
  //delay(1000);
  //gpio_set_level(GPIO_NUM_4, 0);

  //fb = esp_camera_fb_get(); // when placed in a function, it does not work :( 
  //gpio_set_level(GPIO_NUM_4, 0);
  fb = cameraCapture(fb);
  //gpio_set_level(GPIO_NUM_4, 0);
  

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }


  

  //gpio_set_level(GPIO_NUM_4, 0);
  img_processing_dir(fb);
  //gpio_set_level(GPIO_NUM_4, 0); 

  //delay(3000);
  //backwards();
  //left();
 


 

}

camera_fb_t * cameraCapture(camera_fb_t * fb){
  
  //gpio_set_level(GPIO_NUM_4, 1);
  fb = esp_camera_fb_get();
  //gpio_set_level(GPIO_NUM_4, 1);
  //esp_camera_fb_return(fb);
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
  pilot();
  //right();
  //gpio_set_level(GPIO_NUM_4, 0);


  


  // at the end of the loop, the current result becomes the previous 
  dir.previous = dir.current;
  

  /**

  Serial.println("currently moving: " );
  Serial.println(dir.current);  

  Serial.println("previously moving: " );
  Serial.println(dir.previous);  

  Serial.println("most likely moving: " );
  Serial.println(dir.next);  
  */
  

  

  

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

// this function will actually use the predicted directions to assign motor directions to tigerBot
void pilot( ){

  //gpio_set_level(GPIO_NUM_4, 1);
  

  if(dir.next == 'R'){
      //right();
      Serial.println("going right");
      //stop();
  }
  
  
  
  if(dir.next == 'C'){
    //forward();
    Serial.println("going forward");
    //stop();
  }


  if(dir.next == 'L'){
    //left();
    Serial.println("moving left");
    //stop();
  }

  /**
  if(dir.next == 'L'){
    left();
    //stop();
  }
  */
  else{
    //stop();
    Serial.println("stationary");
  }
  
  




}

void forward(){
  //digitalWrite(D1, HIGH);
  //setMotorA(-170);
  //setMotorB(-170);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, HIGH);
  //digitalWrite(D2, LOW);
}
void backwards(){
  //digitalWrite(D1, LOW);
  //digitalWrite(D0, HIGH);
  //digitalWrite(D3, LOW);
  //digitalWrite(D2, HIGH);
  //setMotorA(120);
  //setMotorB(120);
}
void left(){
  //digitalWrite(D1, LOW);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, HIGH);
  //digitalWrite(D2, LOW);
  //setMotorA(140);
  //setMotorB(180);
}
void right(){
  //digitalWrite(D1, HIGH);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, LOW);
  //digitalWrite(D2, LOW);
  //setMotorA(90);
  //setMotorB(180);
}
void stop(){
  //digitalWrite(D1, LOW);
  //digitalWrite(D0, LOW);
  //digitalWrite(D3, LOW);
  //digitalWrite(D2, LOW);
  //setMotorA(0);
  //setMotorB(0);
}
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








