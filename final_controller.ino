#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>

//First code for recievers and esc
#define MAX_SIGNAL 1900
#define MIN_SIGNAL 1000
#define MOTOR_PINC1 11
#define MOTOR_PINCC1 10
#define MOTOR_PINC2 6
#define MOTOR_PINCC2 5
int DELAY = 1000;

Servo motor1, motor2, motor3, motor4;

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
float throttle;
void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}
void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() {
  calc_input(RC_CH1, RC_CH1_INPUT);
}
void calc_ch2() {
  calc_input(RC_CH2, RC_CH2_INPUT);
}
void calc_ch3() {
  calc_input(RC_CH3, RC_CH3_INPUT);
}
void calc_ch4() {
  calc_input(RC_CH4, RC_CH4_INPUT);
}






//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error = 0;                       //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;    //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;
int error_x, error_y;
float desired_angle = 0;

float Kp = 1.56;
float Ki = 0.03;
float Kd = 0.005;
         
float p_errx;
float i_errx;
float d_errx;

float p_erry;
float i_erry;
float d_erry;

float previous_errorx;
float previous_errory;

float PID_x;
float PID_y;


void setup() {
  Wire.begin();                           //begin the wire comunication
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor
  time = millis();
  //Start counting time in milliseconds
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  motor1.attach(11);
  motor2.attach(10);
  motor3.attach(6);
  motor4.attach(5);// attaching the motors

  //Serial.print("On position:");
    motor1.writeMicroseconds(1000);  ///This needs to change? the loop will make it difficult to work.
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
 /*Here we calculate the acc data error before we start the loop
     I make the mean of 200 values, that should be enough*/
  if (acc_error == 0)
  {
    for (int a = 0; a < 200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                       //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);

      Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ; //each value needs two registres
      Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;


      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));

      if (a == 199)
      {
        Acc_angle_error_x = Acc_angle_error_x / 200;
        Acc_angle_error_y = Acc_angle_error_y / 200;
        acc_error = 1;
      }
    }
  }//end of acc error calculation


  /*Here we calculate the gyro data error before we start the loop
     I make the mean of 200 values, that should be enough*/
  if (gyro_error == 0)
  {
    for (int i = 0; i < 200; i++)
    {
      Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 4, true);         //We ask for just 4 registers

      Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
      Gyr_rawY = Wire.read() << 8 | Wire.read();

      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX / 32.8);
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY / 32.8);
      if (i == 199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x / 200;
        Gyro_raw_error_y = Gyro_raw_error_y / 200;
        gyro_error = 1;
      }
    }
  }//end of gyro error calculation

  delay(7000);
}//end of setup void






void loop() {
  timePrev = time;                        // the previous time is stored before the actual time read
  time = millis();                        // actual time read
  elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds
  rc_read_values();  // RC read values 

  
  //////////////////////////////////////Gyro read/////////////////////////////////////

  Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
  Wire.write(0x43);                        //First adress of the Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);         //We ask for just 4 registers

  Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read() << 8 | Wire.read();
  /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
  /*---X---*/
  Gyr_rawX = (Gyr_rawX / 32.8) - Gyro_raw_error_x;
  /*---Y---*/
  Gyr_rawY = (Gyr_rawY / 32.8) - Gyro_raw_error_y;

  /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    If you multiply degrees/seconds by seconds you obtain degrees */
  /*---X---*/
  Gyro_angle_x = Gyr_rawX * elapsedTime;
  /*---X---*/
  Gyro_angle_y = Gyr_rawY * elapsedTime;




  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68)
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68, 6, true);  //We ask for next 6 registers starting withj the 3B
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
    The amount of register to read is specify in the requestFrom function.
    In this case we request 6 registers. Each value of acceleration is made out of
    two 8bits registers, low values and high values. For that we request the 6 of them
    and just make then sum of each pair. For that we shift to the left the high values
    register (<<) and make an or (|) operation to add the low values.
    If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/
  Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ; //each value needs two registres
  Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
  Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
  /*Now in order to obtain the Acc angles we use euler formula with acceleration values
    after that we substract the error value found before*/
  /*---X---*/
  Acc_angle_x = (atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_x;
  /*---Y---*/
  Acc_angle_y = (atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_y;


  //////////////////////////////////////Total angle and filter/////////////////////////////////////
  /*---X axis angle---*/
  Total_angle_x = 0.98 * (Total_angle_x + Gyro_angle_x) + 0.02 * Acc_angle_x;
  /*---Y axis angle---*/
  Total_angle_y = 0.98 * (Total_angle_y + Gyro_angle_y) + 0.02 * Acc_angle_y;

  error_x = Total_angle_x - desired_angle;
  error_y = Total_angle_y - desired_angle;
  
  
  ///////////////////////////////
  
  Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]);
  Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t"); 
  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);


//PID code
  p_errx = Kp * error_x;

 if(-3<error_x<3)
 {
  i_errx = i_errx + (Ki * error_x);
 }
  d_errx = Kd * ((error_x - previous_errorx) / elapsedTime);

  PID_x = p_errx + i_errx + d_errx;
  
if(PID_x>400)
{
  PID_x = 400;
}
if(PID_x<-400)
{
  PID_x=-400;
  }

  float xaxisleft = throttle + PID_x; 
  float xaxisright = throttle - PID_x;//Right
 
  //left
if(xaxisleft < 1000)
{
  xaxisleft= 1100;
}
if(xaxisleft > 1800)
{
  xaxisleft=1800;
}
//right
if(xaxisright < 1000)
{
  xaxisright= 1000;
}
if(xaxisright > 1800)
{
  xaxisright=1800;
}


    

    //motors are initialised

    if (1188 < rc_values[RC_CH3] < 1832)
    {
      throttle = map(rc_values[RC_CH3], 1188, 1832, 1000, 1800);
      motor1.writeMicroseconds(throttle );
      motor2.writeMicroseconds(throttle);
      motor3.writeMicroseconds(throttle);
      motor4.writeMicroseconds(throttle);
    

    if (rc_values[RC_CH1] < 1588) //left or right movement
    {
      int map_ch1left  = map(rc_values[RC_CH1], 1244, 1588, 10, 0);
      float left_mov = throttle * map_ch1left / 100 + throttle ;
      motor2.writeMicroseconds(left_mov );
      motor3.writeMicroseconds(left_mov);
      motor2.writeMicroseconds(xaxisleft );
      motor3.writeMicroseconds(xaxisleft);
    }
    if (rc_values[RC_CH1] > 1588)
    {
      int map_ch1right  = map(rc_values[RC_CH1], 1588, 1984, 0, 10);
      float right_mov = throttle * map_ch1right / 100 + throttle ;
      motor1.writeMicroseconds(right_mov );
      motor4.writeMicroseconds(right_mov);
       motor1.writeMicroseconds(xaxisright );
      motor4.writeMicroseconds(xaxisright);
    }
  
    if (rc_values[RC_CH2] < 1512) //forward or backward movement
    {
      int map_ch2back  = map(rc_values[RC_CH2], 1168, 1512, 10, 0);
      float back_mov = throttle * map_ch2back / 100 + throttle ;
      motor1.writeMicroseconds(back_mov );
      motor2.writeMicroseconds(back_mov);
    }

    if (rc_values[RC_CH1] > 1512)
    {
      int map_ch2front  = map(rc_values[RC_CH2], 1512, 1812, 0, 10);
      float front_mov = throttle * map_ch2front / 100 + throttle ;
      motor4.writeMicroseconds(front_mov );
      motor3.writeMicroseconds(front_mov);
    }



    if (rc_values[RC_CH4] < 1492) //YAW
    {
      int yaw_left  = map(rc_values[RC_CH4], 1140, 1492, 10, 0);
      float rotate_acw = throttle * yaw_left / 100 + throttle ;
    }
    if (rc_values[RC_CH1] > 1588)
    {
      int yaw_right  = map(rc_values[RC_CH4], 1492, 1900, 10, 0);
      float rotate_acw = throttle * yaw_right / 100 + throttle ;
    }
   



  }

 

  Serial.print("Error:      "); Serial.println(error_x); Serial.print("     ");  //error 
 // Serial.print("PID value: "); Serial.print(PID_x); Serial.print("      ");
  //Serial.print("Total X angle value  "); Serial.print(Total_angle_x); Serial.print("      ");//output 
//  Serial.print("Pwm value that is passed to the motors "); Serial.print(X_axcontrol); Serial.print("      ");

 



  
  previous_errorx = error_x;

}
