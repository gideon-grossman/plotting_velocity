import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] accelPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int aligned = 0;
int interval = 0;
int window_width = 500;
int window_height = window_width;
float velocity[] = {0,0,0};
float accel_m_s_s[] = {0,0,0};
float velocity_m_s[] = {0,0,0};
float position_m[] = {window_width / 2, window_height / 2,0};
float velocity_change[] = {0,0,0};
float position[] = {window_width/2, window_height/2, 0};
float position_change[] = {0,0,0};
int settling_time = 10;
int measurements_in_moving_average = 10;
int[][] moving_average_elements = new int[3][measurements_in_moving_average];
int measurements_amount = 0;
float LSB_PER_G = 16384.0;
float M_S_S_PER_G = 9.8;
float looping_interval = 0.026;
float ACCEL_OFFSET = 0;
boolean is_accel_awake = false;
  float horizontal = 0.0;
  int direction = 1;
  int accel[] = {0, 0, 0, 0};
ACCEL_STATE accel_timer_state = ACCEL_STATE.ACCEL_TIMER_START_STATE;
long accel_timer = 3000;
long accel_timer_start;
float[] accel_offset = new float [3];
boolean calibration_complete = false;
int offset_array_counter = 1;
int[][] offset_average_array = new int[3][500];
//int[] offset_x_average_array = new int [500];
//int[] offset_y_average_array = new int[500];
long OFFSET_AVERAGING_TIME = 5000;

void setup() {
    // 300px square viewport using OpenGL rendering
    size(1000, 1000, P3D);
    gfx = new ToxiclibsSupport(this);

    // setup lights and antialiasing
    lights();
    smooth();
    frameRate(30);
    translate(width / 2, height / 2, 3);
    print(width);
    
    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)
    String portName = "/dev/cu.usbmodem411";
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    //String portName = "COM4";
    
    // open the serial port
    port = new Serial(this, portName, 38400);
    
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)
    port.write('r');
}

void draw() {
    print("time: ", millis(), "\r\n");
      if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        port.write('r');
        interval = millis();
    }
    if (!calibration_complete){GetOffset();}
    else if (calibration_complete)
    {
    background(255, 205, 0);
    print("accel[0]: ", accel[0], "\r\n");
    accel_m_s_s[0] = accel[0] / LSB_PER_G * M_S_S_PER_G + accel_offset[0];
    accel_m_s_s[1] = accel[1] / LSB_PER_G * M_S_S_PER_G + accel_offset[1]; 
    print("accel_x: ", accel_m_s_s[0], "m/s^2\r\n");
    print("accel_y: ", accel_m_s_s[1], "m/s^2\r\n");
    print ("accel offset: ", accel_offset[0], "\r\n");
    print ("accel offset: ", accel_offset[1], "\r\n");

    
    if ((abs(accel_m_s_s[0]) > 0.15) | (abs(accel_m_s_s[1]) > 0.15)| (abs(accel_m_s_s[2]) > 0.15))
    {
    StartAccelAwakeTimer();
    }
    else 
    {
    accel_m_s_s[0] = 0.0;
    accel_m_s_s[1] = 0.0;
  }
    
    if (InitialSettlingTimeElapsed() == true)
    {
      if(AccelAwake())
      {
        velocity_m_s[0] = velocity_m_s[0] + accel_m_s_s[0] * (float)looping_interval;
        velocity_m_s[1] = velocity_m_s[1] + accel_m_s_s[1] * (float)looping_interval;
      }
      else 
     {
    velocity_m_s[0] = 0.0;
    velocity_m_s[1] = 0.0;
    }
   }
   //multiply  by 150 to display significant motion on animator.
    position_m[0] = position_m[0] + (velocity_m_s[0] * looping_interval * 150); //should replace looping interval constant with a dynamic measurement.
    position_m[1] = (position_m[1] - (velocity_m_s[1] * looping_interval * 150)); //should replace looping interval constant with a dynamic measurement.
    translate(position_m[0], position_m[1]);
    print("velocity: ");
    print(velocity_m_s[0], "m/s\t", velocity_m_s[1], "m/s\r\n");
    print("position: ");
    print(position_m[0], "m\t", position_m[1], "m/s\r\n");
    rotate(100, 100, 100, 100);
    fill(255, 0, 0, 200);
    box(10, 10, 10);
  }
}

void serialEvent(Serial port) {
    interval = millis();
    while (port.available() > 0) {
        int ch = port.read();
        //print((char)ch);
        if (ch == '$') {serialCount = 0;} // this will help with alignment
        if (aligned < 4) {
            // make sure we are properly aligned on a 14-byte packet
            if (serialCount == 0) {
                if (ch == '$') aligned++; else aligned = 0;
            } else if (serialCount == 1) {
                if (ch == 2) aligned++; else aligned = 0;
            } else if (serialCount == 12) {
                if (ch == '\r') aligned++; else aligned = 0;
            } else if (serialCount == 13) {
                if (ch == '\n') aligned++; else aligned = 0;
            }
            //println(ch + " " + aligned + " " + serialCount);
            serialCount++;
            if (serialCount == 14) serialCount = 0;
        } else {
            if (serialCount > 0 || ch == '$') {
                accelPacket[serialCount++] = (char)ch;
                if (serialCount == 14) {
                    serialCount = 0; // restart packet byte position
                    int accel_x_raw = (accelPacket[2] << 24) | (accelPacket[3] << 16) | (accelPacket[4] << 8) | (accelPacket[5]);
                    int accel_y_raw = (accelPacket[6] << 24) | (accelPacket[7] << 16) | (accelPacket[8] << 8) | (accelPacket[9]);                    
                    int[] accel_averaged = GetMovingAverage(accel_x_raw, accel_y_raw);
                    accel[0] = accel_averaged[0];
                    accel[1] = accel_averaged[1];                    
                    
                    //print("accel[0] & raw"); print(accel[0], "\t", accel_x_raw, "\r\n");
                    //accel[0] = GetMovingAverage(accel_x_raw);
                    //if (InitialSettlingTimeElapsed())
                    //{
                    //  if (abs(accel_x_raw) > 100) {accel[0] = accel_x_raw;}
                    //}
                    //accel[1] = accelPacket[3];
                    //accel[2] = accelPacket[4];
                    //print("accel: ", accel[0], "\r", accel[1], "\t", accel[2]);
                    // get quaternion from data packet
                    //q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
                    //q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
                    //q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
                    //q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
                    //for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
                    
                    // set our toxilibs quaternion to new data
                    //quat.set(q[0], q[1], q[2], q[3]);

                    /*
                    // below calculations unnecessary for orientation only using toxilibs
                    
                    // calculate gravity vector
                    gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
                    gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
                    gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
        
                    // calculate Euler angles
                    euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
                    euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
                    euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
        
                    // calculate yaw/pitch/roll angles
                    ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
                    ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
                    ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
        
                    // output various components for debugging
                    //println("q:\t" + round(q[0]*100.0f)/100.0f + "\t" + round(q[1]*100.0f)/100.0f + "\t" + round(q[2]*100.0f)/100.0f + "\t" + round(q[3]*100.0f)/100.0f);
                    //println("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);
                    //println("ypr:\t" + ypr[0]*180.0f/PI + "\t" + ypr[1]*180.0f/PI + "\t" + ypr[2]*180.0f/PI);
                    */
                }
            }
        }
    }
}

boolean InitialSettlingTimeElapsed()
{
  settling_time--;
  //print("settling time: ", settling_time);
  if (settling_time <= 0)
  {
    return true;
  }
  else return false;
}

int[] GetMovingAverage(int newXValue, int newYValue)
{
  int[] average = new int[3];
  average[0] = 0;
  average[1] = 0;
  //print("measurements_amount: ", measurements_amount, "\r\n");
  //while the moving average buffer fills for the first time
  if (measurements_amount < measurements_in_moving_average)
  {
    moving_average_elements[0][measurements_in_moving_average - 1 - measurements_amount] = newXValue;
    moving_average_elements[1][measurements_in_moving_average - 1 - measurements_amount] = newYValue;

    measurements_amount++;
  }
  
  //when there is a full moving average buffer,
  //remove the oldest element and add the newest.
  else
  {
    for (int k = measurements_in_moving_average; k > 1; k--)
    {
      moving_average_elements[0][k-1] = moving_average_elements[0][k-2];
      moving_average_elements[1][k-1] = moving_average_elements[1][k-2];
    }
    moving_average_elements[0][0] = newXValue;
    moving_average_elements[1][0] = newYValue;
    
    //average
    int[] sum = new int[3];
    sum[0] = 0;
    sum[1] = 0;
    for (int j = 0; j < measurements_in_moving_average; j++)
    {
      sum[0] += moving_average_elements[0][j];
      sum[1] += moving_average_elements[1][j];

    }
    average[0] = sum[0] / measurements_in_moving_average;
    average[1] = sum[1] / measurements_in_moving_average;
  }
  return average;
}

void StartAccelAwakeTimer()
{
  accel_timer_state = ACCEL_STATE.ACCEL_TIMER_START_STATE;
}

boolean AccelAwake()
{
  switch (accel_timer_state) {
    case ACCEL_TIMER_START_STATE:
      accel_timer_start = millis();
      accel_timer_state = ACCEL_STATE.ACCEL_TIMER_ELAPSING_STATE;
      is_accel_awake = true;
      print ("woke up accelerometer#########################\r\n");
      break;
      
    case ACCEL_TIMER_ELAPSING_STATE:
     if (millis() - accel_timer_start > accel_timer)
     {
       accel_timer_state = ACCEL_STATE.ACCEL_TIMER_ELAPSED_STATE;
     }
     is_accel_awake = true;
     break;
     
     case ACCEL_TIMER_ELAPSED_STATE:
       is_accel_awake = false;
       print("timer elapsedd #########################\r\n");
       break;
  }
    
  return is_accel_awake;
}

void GetOffset()
{
  print("in GetOffset\r\n");
  if (millis()>OFFSET_AVERAGING_TIME)
  {
    print("milis: ", millis(), "\r\n");
    
    /*consider averaging first so many values instead of just taking latest.
    for (int i = 1; i < offset_array_counter + 1; i++)
    {
      sum_x += offset_average_array[0][i];
      sum_y += offset_average_array[1][i];
    }
    float average = sum_x / offset_array_counter;
    accel_offset[0] = -1.0 * (average / LSB_PER_G * M_S_S_PER_G); 
    */
    accel_offset[0] = accel[0] / LSB_PER_G * M_S_S_PER_G * -1.0;
    accel_offset[1] = accel[1] / LSB_PER_G * M_S_S_PER_G * -1.0;
    calibration_complete = true;
  }
  else
  {
    offset_average_array[0][offset_array_counter] = accel[0];
    offset_average_array[1][offset_array_counter] = accel[1];
    print ("offset array counter: ", offset_array_counter, "\r\n");
    offset_array_counter++;
  }
}
