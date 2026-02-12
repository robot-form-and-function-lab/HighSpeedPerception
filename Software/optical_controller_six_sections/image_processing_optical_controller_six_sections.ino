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




// initial variables for optical controller - assumes object is moving center; next is undetermined

// initial variables for optical controller - assumes object is moving center; next is undetermined
 struct Direction {
  int current; 

  int previous; 

  int next; 
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

 

  struct Direction dir;
  

}

void loop(){
  // setting up a pointer to the frame buffer

  
  
  
  
  camera_fb_t * fb = NULL;
  
  // Take Picture with camera and put in buffer


  
  fb = cameraCapture(fb);
 

  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  
  
  


  img_processing_dir(fb);

  
  
  

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


  //gpio_set_level(GPIO_NUM_4, 1); // for timing measurement 

  if (fb) {
    // Transfer pixel data from the image buffer to the 2D array
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
      for (int col = 0; col < IMAGE_WIDTH; col++) {
        
        int index = (row * IMAGE_WIDTH) + col; // Calculate the index in the 1D buffer
        image2D[row][col] = (fb->buf[index] > THRESHOLD) ? 1 : 0;;    // Copy the pixel value to the 2D array and put a 1 if above threshold, otherwise 0
       
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

    //delay(1000);
    Serial.println("calculating angle reference");
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
        dir.next = current +1;
      
      //Serial.println("moving right, next section is: ");
      //Serial.print(next);

      
    }
    //staying in place 
    if(previous == current){
      dir.next = current; 
      //Serial.println("stationary, remaining in section: ");
      //Serial.print(next); 
      
    }
    //moving left
    if(previous > current && current -1 >= 0){
      dir.next = current -1; 
      //Serial.println("moving left, moving into section: ");
      //Serial.print(next);
      
    }

    //Serial.println(dir.next);
    

  

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


  return refAngle;

}












