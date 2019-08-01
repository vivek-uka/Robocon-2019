#include <EasyTransfer.h>
#include <DueTimer.h>
#include <DuePWM.h>

#define SerialX SerialUSB
//#define SerialX Serial

#define N 4
#define TimerEncoder Timer1
#define UpperLowerSerial Serial1
#define maxMotRPM 500
#define maxPWM 255
#define EncoderTime 0.1
#define PWM_FREQ1 2500
#define baserpm 0
#define basePWM maxPWM/2


DuePWM pwm(PWM_FREQ1, 3000);

EasyTransfer ET;

struct DATASTRUCT {
  int16_t rpm[N];
};
DATASTRUCT mydata;

class Encoder
{
  public:
    int channel1;
    int channel2;
    int ppr;
    volatile long long int Count;
    volatile long long int prevCount;
    int rpm;
    float gearRatio;
    void initEncoder() {
      pinMode(channel1, INPUT);
      pinMode(channel2, INPUT);
    }
};

Encoder encoder1 = {46, 48, 135, 0, 0, 0, 1.0} ,   *pEncoder1 = &encoder1;
Encoder encoder2 = {50, 52, 135, 0, 0, 0, 1.0} ,   *pEncoder2 = &encoder2;
Encoder encoder3 = {43, 45, 135, 0, 0, 0, 1.0} ,   *pEncoder3 = &encoder3;
Encoder encoder4 = {39, 41, 135, 0, 0, 0, 1.0} ,   *pEncoder4 = &encoder4;

Encoder *pEncoder[N] = {&encoder1, &encoder2, &encoder3, &encoder4};

void returnCount1();
void returnCount2();
void returnCount3();
void returnCount4();

class Motor {
  public:
    int direction1;
    int direction2;
    int pwmPin;

    void initMotor() {
      pinMode(direction1, OUTPUT);
      pinMode(pwmPin, OUTPUT);
      pinMode(direction2, OUTPUT);
      digitalWrite(direction1, LOW);
      digitalWrite(direction2, LOW);
    }
};


Motor motor1 = {22, 24, 6};
Motor motor2 = {25, 23, 7};   
Motor motor3 = {27 ,29, 8};
Motor motor4 = {33 ,31, 9};

Motor *pMotor[N] = {&motor1, &motor2, &motor3, &motor4};
/*********************************************************************************************************************************************/
/******************************************************       PID          ***************************************************************************************/

class PID {
  public:
    float Kp;
    float Kd;
    float Ki;
    float maxControl;
    float minControl;
    uint16_t iCounter;
    
    float required;
    float prevRequired;
    float error;
    float prevError;
    float derivativeError;
    float integralError;
    float prev_integralError;
    void initPID(float kp, float kd, float ki, float req, float minV, float maxV);
    float pidControl(float actual);

};

PID PIDMotor1;
PID *pPIDMotor1 = &PIDMotor1;

PID PIDMotor2;
PID *pPIDMotor2 = &PIDMotor2;

PID PIDMotor3;
PID *pPIDMotor3 = &PIDMotor3;

PID PIDMotor4;
PID *pPIDMotor4 = &PIDMotor4;

PID *pPIDMotor[N] = {&PIDMotor1, &PIDMotor2, &PIDMotor3, &PIDMotor4};

//////////////////////////////////////////////////Global variables****************************************************
float output[N];
int softBrake = 1;

uint16_t ledBlinkerr[4] = {0};

void setup() {

  UpperLowerSerial.begin(19200);
  ET.begin(details(mydata), &UpperLowerSerial);
  
  for (int i = 0; i < N; ++i)
    pMotor[i]->initMotor();

  pwm.setFreq1(PWM_FREQ1);

  for (int i = 0; i < N; ++i)
    pwm.pinFreq1(pMotor[i]->pwmPin);

  for (int i = 0; i < N; ++i)
    pEncoder[i]->initEncoder();

  attachInterrupt(pEncoder1->channel1, returnCount1, RISING);
  attachInterrupt(pEncoder2->channel1, returnCount2, RISING);
  attachInterrupt(pEncoder3->channel1, returnCount3, RISING);
  attachInterrupt(pEncoder4->channel1, returnCount4, RISING);

  TimerEncoder.attachInterrupt(timerHandler);
  TimerEncoder.start(1000000 * EncoderTime);

  pPIDMotor1->initPID(2.0, 0.0, 0.0, 0, -baserpm, maxMotRPM - baserpm);//1.5
  pPIDMotor2->initPID(2.0, 0.0, 0.0, 0, -baserpm, maxMotRPM - baserpm);//1.1 
  pPIDMotor3->initPID(2.0, 0.0, 0.0, 0, -baserpm, maxMotRPM - baserpm);//2.0
  pPIDMotor4->initPID(2.0, 0.0, 0.0, 0, -baserpm, maxMotRPM - baserpm);//1.1

  pinMode(51,OUTPUT);
  set_rpm(100);
}


void loop() {
 getUpperData();  
}

void timerHandler()
{
  for(int i = 0; i < N; ++i)
    pEncoder[i]->rpm = ((pEncoder[i]->Count - pEncoder[i]->prevCount) * 60.0) / (EncoderTime * pEncoder[i]->gearRatio * pEncoder[i]->ppr);

  for(int i = 0; i < N; ++i)
    pEncoder[i]->prevCount = pEncoder[i]->Count;

  for(int i = 0; i < N; ++i)
  {
    if(pPIDMotor[i]->required * pPIDMotor[i]->prevRequired > 0)
      output[i] = baserpm + pPIDMotor[i]->pidControl(pEncoder[i]->rpm);
    
    else
    {
      pPIDMotor[i]->prevRequired = pPIDMotor[i]->required;
      output[i] = 0;
    }
  }
  
//  uint8_t localCtr = 0;
//  for(int i = 0; i < N; ++i)
//    if (pPIDMotor[i]->required == 0)
//    {
//      output[i] = 0;
//      localCtr++;
//    }
//  softBrake = 1;    
//  if(localCtr == 4)
//    softBrake = 0;

  for(int i = 0; i < N; ++i)
    driveMotorReq(output[i], pPIDMotor[i], pMotor[i], maxMotRPM);
}


