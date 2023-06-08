


float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};


void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}



#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// MPU-6050 sensor address
#define MPU6050_ADDRESS 0x68
#include <EEPROM.h>
// MPU-6050 register addresses
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

char command='5';
// MPU-6050 sensitivity settings
#define MPU6050_ACCEL_SENSITIVITY_2G 16384
//#define MPU6050_ACCEL_SENSITIVITY_2G 4096
#define MPU6050_GYRO_SENSITIVITY_500DPS 131

// Radio RF24 settings
RF24 radio(PB0, PA4); // CE, CSN on Blue Pill
const uint64_t address = 0xF0F0F0F0E1LL;
boolean button_state = 0;

// Global variables for sensor data and angles
int16_t accel_x, accel_y, accel_z,temp;
int16_t gyro_x, gyro_y, gyro_z;
float angle_pitch, angle_roll, angle_yaw;
float gyro_x_cal, gyro_y_cal, gyro_z_cal,acc_x_cal,acc_y_cal,acc_z_cal;
// Time variables for angle calculation
// Time variables for angle calculation
unsigned long timer = 0;

float dt = 0;
float angle_roll_acc, angle_pitch_acc,angle_yaw_acc ;
float acc_x, acc_y, acc_z;
uint32_t loop_timer;
char text[32]="";

float RatePitch, RateRoll, RateYaw;
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;




void setup() {
Wire.setClock(400000);
Serial.begin(9600);

// Radio Setup
if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
radio.disableDynamicPayloads();
radio.setAutoAck(false);
Serial.print("ADDRESS :");
radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
radio.startListening();              //This sets the module as receiver

// for channel input
// 
pinMode(PA0,PWM);
pinMode(PA1,PWM);
pinMode(PA2,PWM);
pinMode(PA3,PWM);

pinMode(PC13,OUTPUT);
delay(5);

Wire.begin();

setup_mpu6050();
delay(5);
read_mpu6050_data();
calibrate_gyro();

// for TIMER  3 settings




TIMER2_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
TIMER2_BASE->CR2 = 0;
TIMER2_BASE->SMCR = 0;
TIMER2_BASE->DIER = 0;
TIMER2_BASE->EGR = 0;
TIMER2_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
TIMER2_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
TIMER2_BASE->PSC = 71;
TIMER2_BASE->ARR = 5000;
TIMER2_BASE->DCR = 0;
TIMER2_BASE->CCR1 = 0;
TIMER2_BASE->CCR2=0;
TIMER2_BASE->CCR3=0;
TIMER2_BASE->CCR4=0;

loop_timer = micros();  

}
void loop() {

read_mpu6050_data();
calculate_angles();

kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, angle_roll,angle_roll_acc );
KalmanAngleRoll=Kalman1DOutput[0]; 
KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, angle_pitch,angle_pitch_acc );
KalmanAnglePitch=Kalman1DOutput[0]; 
KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

RatePitch=KalmanAnglePitch;
RateRoll=KalmanAngleRoll;
RateYaw=angle_yaw;



read_receiver();
DesiredRateRoll=0.15*(ReceiverValue[0]-1500);
DesiredRatePitch=0.15*(ReceiverValue[1]-1500);
InputThrottle=ReceiverValue[2];
DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
ErrorRateRoll=DesiredRateRoll-RateRoll;
ErrorRatePitch=DesiredRatePitch-RatePitch;
ErrorRateYaw=DesiredRateYaw-RateYaw;
pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];
pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
pid_equation(ErrorRateYaw, PRateYaw,
       IRateYaw, DRateYaw, PrevErrorRateYaw,
       PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];
if (InputThrottle > 1800) InputThrottle = 1800;
//1.024*
  MotorInput1= (InputThrottle-InputRoll+InputPitch+InputYaw);
  MotorInput2= (InputThrottle+InputRoll+InputPitch-InputYaw);
  MotorInput3= (InputThrottle+InputRoll-InputPitch+InputYaw);
  MotorInput4= (InputThrottle-InputRoll-InputPitch-InputYaw);
if (MotorInput1 > 1550)MotorInput1 = 1550;
  if (MotorInput2 > 1550)MotorInput2 = 1550; 
  if (MotorInput3 > 1550)MotorInput3 = 1550; 
  if (MotorInput4 > 1550)MotorInput4 = 1550;
  int ThrottleIdle=1100;
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;
  int ThrottleCutOff=1000;
  if (ReceiverValue[2]<1050) {
    MotorInput1=ThrottleCutOff; 
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff; 
    MotorInput4=ThrottleCutOff;
  //  reset_pid(); 
  }


TIMER2_BASE->CCR1 = MotorInput1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
TIMER2_BASE->CCR2 = MotorInput2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
TIMER2_BASE->CCR3 = MotorInput3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
TIMER2_BASE->CCR4 = MotorInput4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.
TIMER2_BASE->CNT = 5000;                                                         //This will reset timer 4 and the ESC pulses are directly created.

while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
/*

*/
reset_pid();

Serial.print(" pitch : ");

Serial.print(KalmanAnglePitch);


Serial.print(" | roll : ");
Serial.print(KalmanAngleRoll);

Serial.print(" | yaw : ");
Serial.print(angle_yaw);


/*
Serial.print(" |M1 : ");
Serial.print(MotorInput1);
Serial.print(" | M2 : ");
Serial.print(MotorInput2);
Serial.print(" | M3 : ");
Serial.print(MotorInput3);
Serial.print(" | M4 : ");
Serial.println(MotorInput4);
/*
 Serial.print(" acceleration angle pitch : ");
Serial.print( angle_pitch_acc);


Serial.print(" | acceleration angle roll : ");
Serial.println( angle_roll_acc);


//Serial.print(" | acceleration angle yaw : ");
// Serial.println(acc_z);*/

loop_timer = micros();  

                                                         //Set the timer for the next loop.

}


// MPU6050 MEASERUEMENTS


void read_mpu6050_data() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);
  accel_x = Wire.read()<<8 | Wire.read();
  accel_y = Wire.read()<<8 | Wire.read();
  accel_z = Wire.read()<<8 | Wire.read();
  temp = Wire.read()<<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
  gyro_z*=-1;
  gyro_y*=-1;
}

void setup_mpu6050() {
  // Wake up the MPU-6050 sensor
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();
  
  // Check the WHO_AM_I register to verify the connection
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 1, true);
  byte whoami = Wire.read();
  while(whoami != 0x68) {
    Serial.println("MPU-6050 connection failed!");
  
  }
}

void calculate_angles() {
  // Calculate the time difference between readings
  dt = (float)(micros() - timer) / 1000000.0;
  timer = micros();

 acc_x=(float)accel_x/MPU6050_ACCEL_SENSITIVITY_2G ;
 acc_y=(float)accel_y/MPU6050_ACCEL_SENSITIVITY_2G ;
 acc_z=(float)accel_z/MPU6050_ACCEL_SENSITIVITY_2G ;

 acc_x-=acc_x_cal;
 acc_y-=acc_y_cal;
 acc_z-=acc_z_cal;

  // Calculate the roll and pitch angles from accelerometer data
  angle_pitch_acc = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI  ;
  angle_roll_acc = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI  ;
//  angle_yaw_acc = atan2(accel_x, accel_y) * (180.0 / PI);

  // Calculate the gyro angles
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  //angle_pitch += gyro_x / MPU6050_GYRO_SENSITIVITY_500DPS * dt;
  //angle_roll += gyro_y / MPU6050_GYRO_SENSITIVITY_500DPS * dt;
   angle_yaw += gyro_z / MPU6050_GYRO_SENSITIVITY_500DPS * dt;
   
   angle_pitch = gyro_x / MPU6050_GYRO_SENSITIVITY_500DPS ;
   angle_roll  = gyro_y / MPU6050_GYRO_SENSITIVITY_500DPS ;
 

// angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
//angle_roll += angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  // Complementary filter to combine accelerometer and gyro data
  
//   angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;
//   angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02  ;
 //  angle_yaw= angle_yaw * 0.98 + angle_roll_acc * 0.02;s
  // Calculate the yaw angle using magnetometer data (not implemented in this example code)


}

void calibrate_gyro() {
  Serial.println("Calibrating gyro...");
  for (int i=0; i<2000; i++) {
    read_mpu6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

   acc_x_cal +=(float)accel_x/MPU6050_ACCEL_SENSITIVITY_2G;
   acc_y_cal +=(float)accel_y/MPU6050_ACCEL_SENSITIVITY_2G;
   acc_z_cal +=(float)accel_z/MPU6050_ACCEL_SENSITIVITY_2G;

    delay(6);
  }

  
  gyro_x_cal /= 2000.0;
  gyro_y_cal /= 2000.0;
  gyro_z_cal /= 2000.0;

   acc_x_cal /= 2000.0;
   acc_y_cal /= 2000.0;
   acc_z_cal /= 2000.0;
   acc_z_cal -= 1.0;
  
  
  Serial.println("Gyro calibration complete.");
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
  }

void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}

void read_receiver(void){
  
if (radio.available())              //Looking for the data.
{
Serial.println("Radio is sniffing");
  

radio.read(&command, sizeof(command));    //Reading the data

}
//Serial.println(radio.available());
if(command=='0'){
               ReceiverValue[0]=1000;
              ReceiverValue[1]=1000;
              ReceiverValue[2]=1000;
              ReceiverValue[3]=1000;

              }

if(command=='5'){
               ReceiverValue[0]=1500;
              ReceiverValue[1]=1500;
              ReceiverValue[2]=1550;
              ReceiverValue[3]=1500;
         }
           
 
if(command=='2'){
              ReceiverValue[0]=1500;
              ReceiverValue[1]=1500;
              ReceiverValue[2]=1400;
              ReceiverValue[3]=1500;
              }
 if(command=='8'){
              ReceiverValue[0]=1500;
              ReceiverValue[1]=1500;
              ReceiverValue[2]=1600;
              ReceiverValue[3]=1500;
              }
if(command=='9'){
              ReceiverValue[0]=1500;
              ReceiverValue[1]=1500;
              ReceiverValue[2]=1500;
              ReceiverValue[3]=1550;
              }
 if(command=='7'){
              ReceiverValue[0]=1500;
              ReceiverValue[1]=1500;
              ReceiverValue[2]=1500;
              ReceiverValue[3]=1450;
              }
 if(command=='4'){
              ReceiverValue[0]=1450;
              ReceiverValue[1]=1500;
              ReceiverValue[2]=1500;
              ReceiverValue[3]=1500;
              }
  if(command=='6'){
              ReceiverValue[0]=1550;
              ReceiverValue[1]=1500;
              ReceiverValue[2]=1500;
              ReceiverValue[3]=1500;
              }
             
 if(command=='1'){
              ReceiverValue[0]=1500;
              ReceiverValue[1]=1450;
              ReceiverValue[2]=1500;
              ReceiverValue[3]=1500;
              }
   if(command=='3'){
              ReceiverValue[0]=1500;
              ReceiverValue[1]=1550;
              ReceiverValue[2]=1500;
              ReceiverValue[3]=1500;
              }


Serial.println(command);    
delay(5);

}   
"My code for flight controller using kalman filter" 
