#include <visual_wake_word.h>

// ArduCAM Mini demo (C)2017 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Mini camera, and can run on any Arduino platform.
// This demo was made for ArduCAM_Mini_2MP_Plus.
// It needs to be used in combination with PC software.
// It can test OV2640 functions
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_2MP_Plus
// and use Arduino IDE 1.6.8 compiler or above
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"
#include <JPEGDecoder.h>
//This demo can only work on OV2640_MINI_2MP_PLUS platform.
#if !(defined OV2640_MINI_2MP_PLUS)
  #error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif

#define MAX_JPEG_BYTES 16384
#define RED 22     
#define BLUE 24     
#define GREEN 23
#define LED_PWR 25
 
// set pin 7 as the slave select for the digital pot:
const int CS = 7;

uint8_t bmp_buf[MAX_JPEG_BYTES] = {0};

uint32_t jpg_buf_length = 0;

#if defined (OV2640_MINI_2MP_PLUS)
  ArduCAM myCAM( OV2640, CS );
#else
  ArduCAM myCAM( OV5642, CS );
#endif

uint8_t read_fifo_burst(ArduCAM myCAM);

float frame[45*45*3];
unsigned short dnnc_conv2d_input_shape[] = {1, 45, 45, 3}; // model input shape.

// Decode the JPEG image, crop it, and convert it to greyscale
void DecodeImage(int image_width, int image_height)
{
  
  // Parse the JPEG headers. The image will be decoded as a sequence of Minimum
  // Coded Units (MCUs), which are 16x8 blocks of pixels.
  if (!(JpegDec.decodeArray(bmp_buf, jpg_buf_length) ==1))
  {
    Serial.println("decodeArray failed");
  }

  // Crop the image by keeping a certain number of MCUs in each dimension
  const int keep_x_mcus = image_width / JpegDec.MCUWidth;
  const int keep_y_mcus = image_height / JpegDec.MCUHeight;

  // Calculate how many MCUs we will throw away on the x axis
  const int skip_x_mcus = JpegDec.MCUSPerRow - keep_x_mcus;
  // Roughly center the crop by skipping half the throwaway MCUs at the
  // beginning of each row
  const int skip_start_x_mcus = skip_x_mcus / 2;
  // Index where we will start throwing away MCUs after the data
  const int skip_end_x_mcu_index = skip_start_x_mcus + keep_x_mcus;
  // Same approach for the columns
  const int skip_y_mcus = JpegDec.MCUSPerCol - keep_y_mcus;
  const int skip_start_y_mcus = skip_y_mcus / 2;
  const int skip_end_y_mcu_index = skip_start_y_mcus + keep_y_mcus;

  // Pointer to the current pixel
  uint16_t* pImg;
  // Color of the current pixel
  uint16_t color;

  // Loop over the MCUs
  while (JpegDec.read()) {
    
    //Serial.println("Looping over MCUs");
    
    // Skip over the initial set of rows
    if (JpegDec.MCUy < skip_start_y_mcus) {
      continue;
    }
    // Skip if we're on a column that we don't want
    if (JpegDec.MCUx < skip_start_x_mcus ||
        JpegDec.MCUx >= skip_end_x_mcu_index) {
      continue;
    }
    // Skip if we've got all the rows we want
    if (JpegDec.MCUy >= skip_end_y_mcu_index) {
      continue;
    }

    // Pointer to the current pixel
    pImg = JpegDec.pImage;

    // The x and y indexes of the current MCU, ignoring the MCUs we skip
    int relative_mcu_x = JpegDec.MCUx - skip_start_x_mcus;
    int relative_mcu_y = JpegDec.MCUy - skip_start_y_mcus;

    // The coordinates of the top left of this MCU when applied to the output
    // image
    int x_origin = relative_mcu_x * JpegDec.MCUWidth;
    int y_origin = relative_mcu_y * JpegDec.MCUHeight;

    float temp_buf[16*8*3] = {0};

    // Loop through the MCU's rows and columns
    for (int mcu_row = 0; mcu_row < JpegDec.MCUHeight; mcu_row++) {
      // The y coordinate of this pixel in the output index
      int current_y = y_origin + mcu_row;
      for (int mcu_col = 0; mcu_col < JpegDec.MCUWidth; mcu_col++) {
        // Read the color of the pixel as 16-bit integer
        color = *pImg++;

        temp_buf[mcu_row*16 + mcu_col] = (float)((color & 0xF800) >> 8);
        temp_buf[mcu_row*16 + mcu_col+128] = (float)((color & 0x07E0) >> 3);
        temp_buf[mcu_row*16 + mcu_col+256] = (float)((color & 0x001F) << 3);
      }
    }
    for (int j = 0; j < 1; j++)
    {
      for (int i = 0; i < 2; i++)
      {
        frame[(relative_mcu_y)*45 + (relative_mcu_x*2 + i)] = (temp_buf[i*8] + temp_buf[i*8 + 1] + temp_buf[i*8 + 2] +
          temp_buf[i*8 + 3] + temp_buf[i*8 + 4] + temp_buf[i*8 + 5] + temp_buf[i*8 + 6] + temp_buf[i*8 + 7] +
          
          temp_buf[i*8 + 16] + temp_buf[i*8 + 1 + 16] + temp_buf[i*8 + 2 + 16] + temp_buf[i*8 + 3 + 16] +
          temp_buf[i*8 + 4 + 16] + temp_buf[i*8 + 5 + 16] + temp_buf[i*8 + 6 + 16] + temp_buf[i*8 + 7 + 16] +

          temp_buf[i*8 + 32] + temp_buf[i*8 + 1 + 32] + temp_buf[i*8 + 2 + 32] + temp_buf[i*8 + 3 + 32] +
          temp_buf[i*8 + 4 + 32] + temp_buf[i*8 + 5 + 32] + temp_buf[i*8 + 6 + 32] + temp_buf[i*8 + 7 + 32] +

          temp_buf[i*8 + 48] + temp_buf[i*8 + 1 + 48] + temp_buf[i*8 + 2 + 48] + temp_buf[i*8 + 3 + 48] + 
          temp_buf[i*8 + 4 + 48] + temp_buf[i*8 + 5 + 48] + temp_buf[i*8 + 6 + 48] + temp_buf[i*8 + 7 + 48] +

          temp_buf[i*8 + 64] + temp_buf[i*8 + 1 + 64] + temp_buf[i*8 + 2 + 64] + temp_buf[i*8 + 3 + 64] + 
          temp_buf[i*8 + 4 + 64] + temp_buf[i*8 + 5 + 64] + temp_buf[i*8 + 6 + 64] + temp_buf[i*8 + 7 + 64] +

          temp_buf[i*8 + 80] + temp_buf[i*8 + 1 + 80] + temp_buf[i*8 + 2 + 80] + temp_buf[i*8 + 3 + 80] + 
          temp_buf[i*8 + 4 + 80] + temp_buf[i*8 + 5 + 80] + temp_buf[i*8 + 6 + 80] + temp_buf[i*8 + 7 + 80]) / (48.0 * 255);

        frame[(relative_mcu_y)*45 + (relative_mcu_x*2 + i) + 2025] = (temp_buf[i*8 + 128] + temp_buf[i*8 + 1 + 128] + temp_buf[i*8 + 2 + 128] +
          temp_buf[i*8 + 3 + 128] + temp_buf[i*8 + 4 + 128] + temp_buf[i*8 + 5 + 128] + temp_buf[i*8 + 6 + 128] + temp_buf[i*8 + 7 + 128] +
          
          temp_buf[i*8 + 16 + 128] + temp_buf[i*8 + 1 + 16 + 128] + temp_buf[i*8 + 2 + 16 + 128] + temp_buf[i*8 + 3 + 16 + 128] +
          temp_buf[i*8 + 4 + 16 + 128] + temp_buf[i*8 + 5 + 16 + 128] + temp_buf[i*8 + 6 + 16 + 128] + temp_buf[i*8 + 7 + 16 + 128] +

          temp_buf[i*8 + 32 + 128] + temp_buf[i*8 + 1 + 32 + 128] + temp_buf[i*8 + 2 + 32 + 128] + temp_buf[i*8 + 3 + 32 + 128] +
          temp_buf[i*8 + 4 + 32 + 128] + temp_buf[i*8 + 5 + 32 + 128] + temp_buf[i*8 + 6 + 32 + 128] + temp_buf[i*8 + 7 + 32 + 128] +

          temp_buf[i*8 + 48 + 128] + temp_buf[i*8 + 1 + 48 + 128] + temp_buf[i*8 + 2 + 48 + 128] + temp_buf[i*8 + 3 + 48 + 128] + 
          temp_buf[i*8 + 4 + 48 + 128] + temp_buf[i*8 + 5 + 48 + 128] + temp_buf[i*8 + 6 + 48 + 128] + temp_buf[i*8 + 7 + 48 + 128] +

          temp_buf[i*8 + 64 + 128] + temp_buf[i*8 + 1 + 64 + 128] + temp_buf[i*8 + 2 + 64 + 128] + temp_buf[i*8 + 3 + 64 + 128] + 
          temp_buf[i*8 + 4 + 64 + 128] + temp_buf[i*8 + 5 + 64 + 128] + temp_buf[i*8 + 6 + 64 + 128] + temp_buf[i*8 + 7 + 64 + 128] +

          temp_buf[i*8 + 80 + 128] + temp_buf[i*8 + 1 + 80 + 128] + temp_buf[i*8 + 2 + 80 + 128] + temp_buf[i*8 + 3 + 80 + 128] + 
          temp_buf[i*8 + 4 + 80 + 128] + temp_buf[i*8 + 5 + 80 + 128] + temp_buf[i*8 + 6 + 80 + 128] + temp_buf[i*8 + 7 + 80 + 128]) / (48.0 * 255);

        frame[(relative_mcu_y)*45 + (relative_mcu_x*2 + i) + 4050] = (temp_buf[i*8 + 256] + temp_buf[i*8 + 1 + 256] + temp_buf[i*8 + 2 + 256] +
          temp_buf[i*8 + 3 + 256] + temp_buf[i*8 + 4 + 256] + temp_buf[i*8 + 5 + 256] + temp_buf[i*8 + 6 + 256] + temp_buf[i*8 + 7 + 256] +
          
          temp_buf[i*8 + 16 + 256] + temp_buf[i*8 + 1 + 16 + 256] + temp_buf[i*8 + 2 + 16 + 256] + temp_buf[i*8 + 3 + 16 + 256] +
          temp_buf[i*8 + 4 + 16 + 256] + temp_buf[i*8 + 5 + 16 + 256] + temp_buf[i*8 + 6 + 16 + 256] + temp_buf[i*8 + 7 + 16 + 256] +

          temp_buf[i*8 + 32 + 256] + temp_buf[i*8 + 1 + 32 + 256] + temp_buf[i*8 + 2 + 32 + 256] + temp_buf[i*8 + 3 + 32 + 256] +
          temp_buf[i*8 + 4 + 32 + 256] + temp_buf[i*8 + 5 + 32 + 256] + temp_buf[i*8 + 6 + 32 + 256] + temp_buf[i*8 + 7 + 32 + 256] +

          temp_buf[i*8 + 48 + 256] + temp_buf[i*8 + 1 + 48 + 256] + temp_buf[i*8 + 2 + 48 + 256] + temp_buf[i*8 + 3 + 48 + 256] + 
          temp_buf[i*8 + 4 + 48 + 256] + temp_buf[i*8 + 5 + 48 + 256] + temp_buf[i*8 + 6 + 48 + 256] + temp_buf[i*8 + 7 + 48 + 256] +

          temp_buf[i*8 + 64 + 256] + temp_buf[i*8 + 1 + 64 + 256] + temp_buf[i*8 + 2 + 64 + 256] + temp_buf[i*8 + 3 + 64 + 256] + 
          temp_buf[i*8 + 4 + 64 + 256] + temp_buf[i*8 + 5 + 64 + 256] + temp_buf[i*8 + 6 + 64 + 256] + temp_buf[i*8 + 7 + 64 + 256] +

          temp_buf[i*8 + 80 + 256] + temp_buf[i*8 + 1 + 80 + 256] + temp_buf[i*8 + 2 + 80 + 256] + temp_buf[i*8 + 3 + 80 + 256] + 
          temp_buf[i*8 + 4 + 80 + 256] + temp_buf[i*8 + 5 + 80 + 256] + temp_buf[i*8 + 6 + 80 + 256] + temp_buf[i*8 + 7 + 80 + 256]) / (48.0 * 255);

        if ((relative_mcu_x == 19) && (i == 1))
        { 
          frame[(relative_mcu_y)*45 + 40] = frame[(relative_mcu_y)*45 + 39];
          frame[(relative_mcu_y)*45 + 41] = frame[(relative_mcu_y)*45 + 39];
          frame[(relative_mcu_y)*45 + 42] = frame[(relative_mcu_y)*45 + 39];
          frame[(relative_mcu_y)*45 + 43] = frame[(relative_mcu_y)*45 + 39];
          frame[(relative_mcu_y)*45 + 44] = frame[(relative_mcu_y)*45 + 39];
          frame[(relative_mcu_y)*45 + 40 + 2025] = frame[(relative_mcu_y)*45 + 39 + 2025];
          frame[(relative_mcu_y)*45 + 41 + 2025] = frame[(relative_mcu_y)*45 + 39 + 2025];
          frame[(relative_mcu_y)*45 + 42 + 2025] = frame[(relative_mcu_y)*45 + 39 + 2025];
          frame[(relative_mcu_y)*45 + 43 + 2025] = frame[(relative_mcu_y)*45 + 39 + 2025];
          frame[(relative_mcu_y)*45 + 44 + 2025] = frame[(relative_mcu_y)*45 + 39 + 2025];
          frame[(relative_mcu_y)*45 + 40 + 4050] = frame[(relative_mcu_y)*45 + 39 + 4050];
          frame[(relative_mcu_y)*45 + 41 + 4050] = frame[(relative_mcu_y)*45 + 39 + 4050];
          frame[(relative_mcu_y)*45 + 42 + 4050] = frame[(relative_mcu_y)*45 + 39 + 4050];
          frame[(relative_mcu_y)*45 + 43 + 4050] = frame[(relative_mcu_y)*45 + 39 + 4050];
          frame[(relative_mcu_y)*45 + 44 + 4050] = frame[(relative_mcu_y)*45 + 39 + 4050];
        }

        if ((relative_mcu_y == 29) && (relative_mcu_x == 19) && (i == 1))
        {
          for (int p=0; p < 45; p++)
          {
            frame[30*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[31*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[32*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[33*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[34*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[35*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[36*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[37*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[38*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[39*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[40*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[41*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[42*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[43*45 + p] = frame[(relative_mcu_y)*45 + p];
            frame[44*45 + p] = frame[(relative_mcu_y)*45 + p];
            
            frame[30*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[31*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[32*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[33*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[34*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[35*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[36*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[37*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[38*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[39*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[40*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[41*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[42*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[43*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];
            frame[44*45 + p + 2025] = frame[(relative_mcu_y)*45 + p + 2025];

            frame[30*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[31*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[32*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[33*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[34*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[35*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[36*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[37*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[38*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[39*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[40*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[41*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[42*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[43*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
            frame[44*45 + p + 4050] = frame[(relative_mcu_y)*45 + p + 4050];
          }
        }
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  uint8_t vid, pid;
  uint8_t temp;
  #if defined(__SAM3X8E__)
    Wire1.begin();
    Serial.begin(115200);
  #else
    Wire.begin();
    Serial.begin(921600);
  #endif

  delay(5000);
  Serial.println(F("ACK CMD ArduCAM Start! END"));
  // set the CS as an output:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  // initialize SPI:
  SPI.begin();
    //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);
  while(1){
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55){
      Serial.println(F("ACK CMD SPI interface Error! END"));
      delay(1000);continue;
    }else{
      Serial.println(F("ACK CMD SPI interface OK. END"));break;
    }
  }

  Serial.println("Checking for Camera Module type");

  #if defined (OV2640_MINI_2MP_PLUS)
    while(1){
      //Check if the camera module type is OV2640
      myCAM.wrSensorReg8_8(0xff, 0x01);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
      if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
        Serial.println(F("ACK CMD Can't find OV2640 module! END"));
        delay(1000);continue;
      }
      else{
        Serial.println(F("ACK CMD OV2640 detected. END"));break;
      } 
    }
  #endif

  //Change to JPEG capture mode and initialize the OV5642 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  Serial.println(F("ACK CMD switch to OV2640_320x240 END"));
  delay(1000);
  myCAM.clear_fifo_flag();

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, HIGH);

}

void loop()
{

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  //Start capture
  myCAM.start_capture();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
  {
    delay(50);
  }
  Serial.println(F("ACK CMD CAM Capture Done. END"));
  delay(50);
  //Clear the capture done flag
  myCAM.clear_fifo_flag();

  read_fifo_burst(myCAM);
  DecodeImage(320,240);

  deepSea::ndarray<float> dnnc_conv2d_input(frame, dnnc_conv2d_input_shape, 4); 
  std::vector<deepSea::ndarray<float>> deepSea_result = deepSea_model(dnnc_conv2d_input);

  Serial.println();
  Serial.print("deepSea output is ");
  Serial.println(deepSea_result[0]._data[0]);
  unsigned int result = (unsigned int)(deepSea_result[0]._data[0] > 0.5);

  if (result) {
    Serial.println("PREDICTION is PERSON DETECTED");
    digitalWrite(BLUE, LOW);
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    delay(1000);
  }
  else {
    Serial.println("PREDICTION is NO PERSON DETECTED");
    digitalWrite(BLUE, LOW);
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, HIGH);
    delay(1000);
  }

  Serial.println();
  free(deepSea_result[0]._data);
}

uint8_t read_fifo_burst(ArduCAM myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t jpg_buf_counter = 0;
  uint32_t jpg_length;
  bool is_header = false;

  jpg_length = myCAM.read_fifo_length();
  jpg_length = 16000;
  if (jpg_length >= MAX_JPEG_BYTES)
  {
    Serial.println(F("ACK CMD Over size. END"));
    return 0;
  }
  if (jpg_length == 0 ) //0 kb
  {
    Serial.println(F("ACK CMD Size is 0. END"));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  temp =  SPI.transfer(0x00);
  jpg_length--;
  while ( jpg_length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    if (is_header == true)
    {
      bmp_buf[jpg_buf_counter] = temp;
      jpg_buf_counter++;
    }
    else if ((temp == 0xD8) && (temp_last == 0xFF))
    {
      is_header = true;
      bmp_buf[jpg_buf_counter] = temp_last;
      jpg_buf_counter++;
      bmp_buf[jpg_buf_counter] = temp;
      jpg_buf_counter++;
    }
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      break;
    }
    delayMicroseconds(15);
  }
  jpg_buf_length = jpg_buf_counter;
  myCAM.CS_HIGH();
  is_header = false;

  return 1;
}
