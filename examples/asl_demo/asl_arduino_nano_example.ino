
#include <Arduino_LSM9DS1.h>
#include "asl_imu.h"

unsigned short dnnc_conv2d_input_shape[] = {1, 1, 1380}; // model input shape.
float frame[1380];
int g_loop = -1;
int g_loop_count_start = 0;
int g_loop_count_end = 230;
unsigned int frame_index = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  while (!IMU.begin());
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Start Gesture in 1 second");
  //delay(1000);
  unsigned int a_wait_time = 0;
  unsigned int g_wait_time = 0;
  unsigned int inf_start = 0;
  unsigned int inf_end = 0;
  float temp = -10000;
  unsigned int index = 0;
  float ax, ay, az;

  if (g_loop == 0)
  {
    for (int i = 0; i < frame_index; i++)
    {
      frame[i] = frame[i+690];
    }
  }

  for (int i=g_loop_count_start; i < g_loop_count_end; i++)
  {
    while (!IMU.accelerationAvailable())
    {
      a_wait_time++;
    }
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
  
/*      Serial.print("\"ax\":");
      Serial.print(ax);
      Serial.print(',');
      Serial.print("\"ay\":");
      Serial.print(ay);
      Serial.print(',');
      Serial.print("\"az\":");
      Serial.print(az);
      Serial.println(',');*/
    }
    float gx, gy, gz;
    while (!IMU.gyroscopeAvailable())
    {
      g_wait_time++;
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
  
/*      Serial.print("\"gx\":");
      Serial.print(gx);
      Serial.print(',');
      Serial.print("\"gy\":");
      Serial.print(gy);
      Serial.print(',');
      Serial.print("\"gz\":");
      Serial.print(gz);
      Serial.println(',');*/
    }
  
    frame[frame_index+i*6]   = (ax+4)/8;
    frame[frame_index+i*6+1] = (ay+4)/8;
    frame[frame_index+i*6+2] = (az+4)/8;
    frame[frame_index+i*6+3] = (gx+2000)/4000;
    frame[frame_index+i*6+4] = (gy+2000)/4000;
    frame[frame_index+i*6+5] = (gz+2000)/4000;
  }
  
  deepSea::ndarray<float> dnnc_conv2d_input(frame, dnnc_conv2d_input_shape, 3); 
  inf_start = millis();
  std::vector<deepSea::ndarray<float>> deepSea_result = deepSea_model(dnnc_conv2d_input);
  inf_end = millis();

  for (int i = 0; i < 5; i++)
  {
    //Serial.println(deepSea_result[0]._data[i]);
  
    if (deepSea_result[0]._data[i] > temp)
    {
      temp = deepSea_result[0]._data[i];
      index = i;
    }
  }

  Serial.print("[Inferred ");
  Serial.print(inf_end - inf_start);
  Serial.print("ms] - ");
  switch(index)
  {
    case 0:
      Serial.println("Thank You");
      break;

    case 1:
      Serial.println("Help");
      break;

    case 2:
      Serial.println("More");
      break;

    case 3:
      Serial.println("No Gesture");
      break;

    case 4:
      Serial.println("Today or Now");
      break;
      
    default:
      Serial.println("Incorrect Index");
      break;

  }

  g_loop = 0;
  g_loop_count_end = 115;
  frame_index = 690;
  Serial.println();
}
