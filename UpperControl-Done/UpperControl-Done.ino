// 15-06-19
enum {BLUE = -1, RED = 1};
int8_t arenaFlag = RED;              //X = A,B   if arena == RED => Y = B,C                 else Y = A,D 
 
//////////////////////////////////////////////// MPU ///////////////////////////////////////////////////////////////////////////////////
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 18  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

////////////////////////////////////////////MPU END//////////////////////////////////////////////////////////////////////
///////////////////////////////////////////  General  /////////////////////////////////////////////////////////////
#include <EasyTransfer.h>
#include <TimerOne.h>
#include <Servo.h>
Servo myServo, hisServo;
#define SerialX Serial

#ifdef Due
  #include <DuePWM.h>
  #define SerialX SerialUSB  
#endif

EasyTransfer ET;
struct DATASTRUCT {
  int16_t rpm[4];
//  int16_t manualFlag;
};
DATASTRUCT mydata;
 
#define UpperLowerSerial Serial2
#define BluetoothSerial  Serial3
#define SerialLSA        Serial
#define pi 3.141592653589
#define maxWheelRPM 400
#define maxMotRPM 460
#define maxPWM 255
#define DegreeToRadian(x) x*0.0174532  
#define RadianToDegree(x) x*57.295779
#define Angle1 45   //1.57079  //90       //90+0
#define Angle2 135  //3.66519  //210      //90+120
#define Angle3 225  //5.75958  //330      //90+240
#define Angle4 315  //
#define RadiusOmniDrive 37.5  //centimetre
#define RadiusOmniWheel 7.5 //centimetre
#define RadiusXYWheel   2.9 //centimetre
#define VelocityToRPM(x) x*60/(2*pi*RadiusOmniWheel) //vel in cm/s
#define redLEDPin    A14
#define blueLEDPin   A13
#define greenLEDPin  A9
#define yellowLEDPin A8
#define whiteLEDPin  A10
#define whiteLEDon digitalWrite(whiteLEDPin,HIGH)
#define whiteLEDoff digitalWrite(whiteLEDPin,LOW)
#define blueLEDon digitalWrite(blueLEDPin,HIGH)
#define blueLEDoff digitalWrite(blueLEDPin,LOW)
#define redLEDon digitalWrite(redLEDPin,HIGH)
#define redLEDoff digitalWrite(redLEDPin,LOW)
#define greenLEDon digitalWrite(greenLEDPin,HIGH)
#define greenLEDoff digitalWrite(greenLEDPin,LOW)
#define yellowLEDon digitalWrite(yellowLEDPin,HIGH)
#define yellowLEDoff digitalWrite(yellowLEDPin,LOW)
#define proximityPin 10 
#define resetAlignReachS1Button 47
#define calibrateLSAButton A3
#define resetAlignNearTZButton A5

float manualSpeed = 300; 
float xaRED[13] = {0, 30, 75, 110,    150, 225, 280,    340,375,410,450};   
float yaRED[13] = {0, 40, 50,  45,    -45, -65, -50,     55, 60, 55,  0};


float xaBLUE[13] = {0,15, 75, 110,   190, 225, 265,    295, 375, 430, 450};   
float yaBLUE[13] = {0,50, 60, 55,    -60,-60,-55,       65,  65,  65,   0};//{0,50, 60, 55,    -60,-60,-55,       65,  65,  65,   0};  
/*********************************************************************************************************************************************/
/*********************************************************** Wheels **********************************************************************************/

class Wheel {
  public:
    float translationSpeed;
    float angularSpeed;
    int angle;
    float maxRPM;
    float Speed;
    float rpm;
    int prevRPM;
};

Wheel Wheel1 = {0.0, 0.0, Angle4, maxWheelRPM, 0, 0, 0 }; 
Wheel Wheel2 = {0.0, 0.0, Angle1, maxWheelRPM, 0, 0, 0 }; 
Wheel Wheel3 = {0.0, 0.0, Angle2, maxWheelRPM, 0, 0, 0 }; 
Wheel Wheel4 = {0.0, 0.0, Angle3, maxWheelRPM, 0, 0, 0 }; 

Wheel *pWheel[4] = {&Wheel1, &Wheel2, &Wheel3, &Wheel4};

class Encoder
{
  public:
    int channel1;
    int channel2;
    int ppr;
    volatile long long int Count;
    volatile long long int prevCount;
    int rpm;

    void initEncoder() {
      pinMode(channel1, INPUT_PULLUP);
      pinMode(channel2, INPUT_PULLUP);
    }
};

Encoder xencoder = {2, 4, 1000, 0, 0, 0};  //31,52                                  //2 31
Encoder *pEncoderX = &xencoder;

Encoder yencoder = {3, 7, 1000, 0, 0, 0}; //1000 ppr                         //3 39    
Encoder *pEncoderY = &yencoder;

void returnCountX();
void returnCountY();

/*********************************************************************************************************************************************/
/*********************************************************** Autonomous Bot **********************************************************************************/
class Auto_Bot {
  public:
    volatile float X_pos;
    volatile float Y_pos;
    volatile float X_pos1;
    volatile float Y_pos1;
    volatile float del_x;
    volatile float del_y;
    float Angle;
    float vel;
    float Omega;
    volatile float prev_X;
    volatile float prev_Y;
    Auto_Bot() {
      X_pos = 0;
      Y_pos = 0;
      X_pos1 = 0;
      Y_pos1 = 0;
      del_x = 0;
      del_y = 0;
      Angle = 0;
      prev_X = 0;
      prev_Y = 0;
      vel = 0;
    }
};

Auto_Bot fourWheelDrive;
Auto_Bot *pBot = &fourWheelDrive;
///////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************       PID          ***************************************************************************************/

class PID {
  public:
    float Kp;
    float Kd;
    float Ki;
    float Kp_old;
    float Kd_old;
    float Ki_old;
    float maxControl;
    float minControl;

    float required;
    float prevRequired;
    float error;
    float prevError;
    float derivativeError;
    float integralError;
    void initPID(float kp, float kd, float ki, float req, float minV, float maxV);
    float pidControl(float actual);
   
};

PID pidDistance;
PID *ppidDistance = &pidDistance;
PID pidOmega;
PID *ppidOmega = &pidOmega;
PID Linegain, Omegagain;
PID *pLinegain = &Linegain;
PID *pOmegagain = &Omegagain;

enum {forward, backward, stopp};

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

    void actuateMotor(int pwmm, uint8_t directionn)   //forward, backward, stopp
    {
      if(directionn != stopp)
      {
        digitalWrite(direction1, !directionn);
        digitalWrite(direction2, directionn);
      }
      else
      {
        digitalWrite(direction1, HIGH);
        digitalWrite(direction2, HIGH);
      }

      analogWrite(pwmPin,pwmm);
      
    }
};

Motor loadingMotor = {48, 53, 8};
Motor *pLoadingMotor = &loadingMotor;
//___________________________________________________CURVE INIT_____________________________________________________//

class Curve
{
  public :
    float x1,y1,x2,y2,k;
    uint8_t flag;
    int i;
    Curve();
    bool traceArc(float theta,float delta, float radius, float vel);
    bool traceForest(float vel);
    bool traceSine(float delta_x,float amplitude, float wavelength, float vel);
};
Curve curve;

/////////////////////////////////////////////////// Line Following ////////////////////////////////////////////////////
const int rotateRPM = 120;
unsigned int rpmmax = 500;
bool lsa = true, junctionFlag = 0, junctionFlag2 = 0;
enum {f,b,r,l,s};

class LSA08{
 public: 
  char Address;
  int Theta;
  int JunctionPin;
  int JunctionCount;
  int OePin;
  uint8_t junction_detect[4];
  
  LSA08(char a, int b, int c, int d, int eee){
    this->Address = a;
    this->Theta = b;
    this->JunctionPin = c;
    this->JunctionCount = d;
    this->OePin = eee;
  };
  
  void initLSA(int,int);
  void sendCommand(char,char,char);
  void ChangeBaud(char, char);
  void clearJunction(char);
  int getJunction(char);
  int GetByteOfLSA(int);
  void Calibrate(int);
};

float LSAforwardprev;
float LSAbackwardprev;
int LSArotateprev;
LSA08 LSAb(0x04,180,31,0,29);    // f=1
LSA08 LSAf(0x01,0,51,0,49);
LSA08 *LSAArray[2]={&LSAf,&LSAb};

volatile int ActiveSensor = 0, ActiveOmegaSensor = 1;
float Linecontrol = 0, Omegacontrol = 0, omegaLine = 0;
bool junctionn;
float BotX = 0;
float BotY = 0;
//////////////////////////////////////////////////  PS2 ///////////////////////////////////////////////////////////////
#define __AVR__
#include <PS2X_lib.h>

#include <SPI.h>
#define PS3
#define basePS2_Y 128.0
#define wirelessdelay 0
#ifdef PS3
  #define wirelessdelay 50 
  #define basePS2_Y 127.0
#endif

PS2X ps2x;
EasyTransfer ET_ps2;
  
struct PS2_data{
  int32_t ps2data;                                 
  uint8_t Rx;
  uint8_t Ry;
  uint8_t Lx;
  uint8_t Ly; 
};
PS2_data data;

double xSquare_leftdistance = 0, ySquare_leftdistance = 0, xSquare_rightdistance = 0, ySquare_rightdistance = 0;
double xCircle_rightdistance = 0, yCircle_rightdistance = 0, xCircle_leftdistance = 0, yCircle_leftdistance = 0;
double LeftAnalogTheta = 0, RightAnalogTheta = 0;
double LeftAnalogDistance = 0, RightAnalogDistance = 0;
uint8_t ps2release_count = 0;
uint8_t ps2receiveflag = 1;
uint8_t rx = 0,ry = 0,lx = 0,ly = 0;                                               

//___________________________________________ Throwing and Loading _________________________________________//
//#include <Adafruit_MCP4725.h>
//Adafruit_MCP4725 dac;

enum {grip, releasee, throww};
uint8_t pneumaticPin[3] = {23, 25, 27}; 

#define limitSwitchForward A7
#define limitSwitchBackward A0

long int LimitCounter = 0;
enum  Mechanism {Release , ReachShagai , GripUp , GripDown , AvoidCollision , ThrowShagai, GrabShagai, LoadShagai ,ReleaseShagai};
Mechanism loadState = GrabShagai, throwState = Release;
bool stopLoad=1,stopThrow=1;
//__________________________________________________________________________________________________________//
float mpuConst = 0.0, robotYaw, prevYaw = -1, omegacontrol = 0.0;
int minSpeeed, maxSpeeed;
uint16_t counter11 = 0, randomCtr = 1, rotateCount = 0;
uint8_t fsmFlag = 1, gripFlag = 2, ledBlinkerr[5] = {0}, rotateFlag=1, throwFlag = 1,sigmoidFlag = 0;
float prevVt = 100, Vt;
enum {bot};
bool controlFlag = 1 ;//[2] = {1,1};
float speeed;

enum {clockWise, antiClockWise};
enum {ALL_QUAD, ONE_FOUR, TWO_THREE, NOT_ONE, NOT_TWO, NOT_THREE, NOT_FOUR, ONE_TWO, ONE, THREE_FOUR};
enum {X_axis, Y_axis};
enum {CHECKX, CHECKY, CHECKD};
bool Axis = X_axis;
float accc = 0, controlX = 0;
enum {STOP,CONTINUE};

enum {blue, yellow, red, green, white};
uint8_t colorPin[5] = {blueLEDPin, yellowLEDPin, redLEDPin, greenLEDPin, whiteLEDPin};
uint8_t colorr = blue;

//_______________________________________Odometry___________________________________________________________________//

int sineWavelength = 300, sineAmplitude = 60;
float lambda_not = 40;
float deltta_x = lambda_not;

enum {mpuCalibration = 9, checkArena, khangaiUrtuu, forest, alignBridge, bridge, khangaiArea, towardsGobi, alignGobi,startSignal};
enum {afterAlignGobi = 22, reachShagai1, NearTZ, rotateNearTZ1, waitForMR2, TZ1, reachShagai2, TZ2, reachShagai3, TZ3, gobiToRotate};
enum {completeManual = 39, lineFollowing, stopBot}; 

uint8_t state = khangaiUrtuu;

bool grabFlag = 0, releaseFlag = 0, throwinggFlag = 0;

enum {Forward, Left, Right, Backward};
int arcFlag = 1;
float shiftX = 0, shiftY = 0;
float alignX = 0, alignY = 0;
uint8_t safeFlag1 = 0,safeFlag2 = 0;
long int calibrationTime = 0;
bool odometryFlag = 1;
float alignGobiAngle = pi/2;
float prevBotX = -1, prevBotY = -1, delta_xx = 10;
uint8_t bridgeCounter = 0;
int reading,previousReading;
long int gripperTimeOut = 0, omegaTimeOut, waitTime;
bool OmFlag = 1;
bool firstTimeBridgeFlag = 1, firstTimeServoFlag = 1;
uint8_t secondTimeServoFlag = 1;
int manualFlag = 0;
float errorBridge = 0;
int reachY, reachX, startX, startY;

//------ Acceleration----------//
float delta_v = 0, delta_d = 0, botSpeed = 0;
enum {X_Axis, Y_Axis};
bool AccAxis;uint16_t delayCounter = 0;

int sineMode = CONTINUE;
int sineQuad = ONE_FOUR;
int arcQuad = ONE_FOUR, gobiQuad = ONE_TWO;

int reachShagaiY;
//_________________________________________________________________________________________________________________
void setup() {

  #ifdef AutoOmega
  Wire.begin();
  #endif
  Serial.begin(9600);
  SerialX.println(" setup staretd");
  UpperLowerSerial.begin(19200);
  ET.begin(details(mydata), &UpperLowerSerial);

  BluetoothSerial.begin(38400);
  ET_ps2.begin(details(data), &BluetoothSerial);

  for (int k = 0; k < 4; k++)
    pWheel[k]->rpm = 0;
  
  TransmitURPM();

//  dac.begin(0x60);                        
//  dac.setVoltage(417*3.8,false); 

  myServo.attach(9);
  hisServo.attach(11);
  initLSA();
  initLoad();  
  pLoadingMotor->initMotor();
  
  pEncoderX->initEncoder();
  attachInterrupt(digitalPinToInterrupt(pEncoderX->channel1), returnCountX, RISING);

  pEncoderY->initEncoder();
  attachInterrupt(digitalPinToInterrupt(pEncoderY->channel1), returnCountY, RISING);
    
  initLEDs();
  interrupts();

  MPUsetup();

//P D I
  pLinegain->initPID(0.5, 2.0, 0, 0, -255, 255);//
  pOmegagain->initPID(0.025,0.10,0,0,-125,125); //LF
  ppidOmega->initPID(0.175,0.8,0, 0, -2.0, 2.0);            // Kp = 0.12
  ppidDistance->initPID(0.0,0,0,0,0,0);//(0.25, 1.25, 0, 0,-100, 100);
 
  ActiveSensor = f;
  ActiveOmegaSensor = b;

  speeed = 300; //cm/sec 
  randomCtr = 1;
  odometryFlag = 1;
  
  pinMode(proximityPin,INPUT_PULLUP);
  pinMode(resetAlignReachS1Button,INPUT_PULLUP);
  pinMode(calibrateLSAButton,INPUT_PULLUP);  
  pinMode(resetAlignNearTZButton,INPUT_PULLUP);
  
  Serial.println("Setup done");
  
  myServo.write(173);
  delay(100);
  hisServo.write(0);
  calibrationTime = millis(); 
  
  if(!digitalRead(calibrateLSAButton))
    {
      calibrateBothLSA();    
      delay(8000);
      redLEDon;
    }
   
  state = mpuCalibration;// reachShagai1; bridge; lineFollowing;
}

void loop(){
  finiteStateMachine();
//  getYaw();
//    omegacontrol = OmegaControlMPU(robotYaw,2);
//  Serial.println(robotYaw);
  TransmitURPM();  
}

void returnCountX()
{
  if(digitalRead(pEncoderX->channel2))
    pEncoderX->Count--;
  else
    pEncoderX->Count++;      
}

void returnCountY()
{
  if(digitalRead(pEncoderY->channel2))
    pEncoderY->Count++;
  else
    pEncoderY->Count--;
}
