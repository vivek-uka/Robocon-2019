#include <PS2X_lib.h>
#include <EasyTransfer.h>
#include <SPI.h>

#define For_mega

//#define For_uno

//#ifndef For_mega
//#define For_uno
//#endif

#define BluetoothSerial Serial

#ifdef For_mega
#define BluetoothSerial Serial3
#endif

//#define filter

#define PS3

#define basePS2_Y 128.0
#define basePS2_X 128.0
#define wirelessdelay 0
#ifdef PS3
#define wirelessdelay 50
#define basePS2_Y 127.0
#define basePS2_X 128.0
#endif


EasyTransfer ET_ps2;
PS2X ps2x;

#ifdef For_uno 
  #define PS2_DAT 12
  #define PS2_CMD 11
  #define PS2_SEL 10
  #define PS2_CLK 13
#endif

#ifdef For_mega
  #define PS2_DAT        50     
  #define PS2_CMD        51  
  #define PS2_SEL        6  
  #define PS2_CLK        52 
#endif

struct PS2_data {
  int32_t ps2data;
  uint8_t Rx;
  uint8_t Ry;
  uint8_t Lx;
  uint8_t Ly;
};
PS2_data data;

class Moving_array {
  public:
    uint8_t RX [5];
    uint8_t RY [5];
    uint8_t LX [5];
    uint8_t LY [5];

    void get_initial_data();
    int count_moving_rx[5];
    int count_moving_ry[5];
    int count_moving_lx[5];
    int count_moving_ly[5];
    void moving_data();
    void correct_data();
};
uint8_t maxCount_lx = 0;
uint8_t maxCount_ly = 0;
uint8_t maxCoumt_rx = 0;
uint8_t maxCount_ry = 0;
Moving_array moving_array;
Moving_array *pmoving_array = &moving_array;
double xSquare_leftdistance = 0, ySquare_leftdistance = 0, xSquare_rightdistance = 0, ySquare_rightdistance = 0, xCircle_leftdistance = 0, yCircle_leftdistance = 0, xCircle_rightdistance = 0, yCircle_rightdistance = 0;
double LeftAnalogTheta = 0, RightAnalogTheta = 0;
float LeftAnalogDistance = 0, RightAnalogDistance = 0;
const float rad = 1;
const float pi = 3.14159;
int ps2release_count = 0;
int error = 0;
byte type = 0;
byte vibrate = 0;

void setup()
{
    BluetoothSerial.begin(38400);
    Serial.begin(38400);
    ET_ps2.begin(details(data),&BluetoothSerial);
    initps2();
  #ifdef filter
    pmoving_array->get_initial_data();
  #endif
}

void loop()
{
  getPS2value();
  //Serial.println(String(PS2_SEL) + " slave select  ");
  #ifdef filter
    pmoving_array->moving_data();
    pmoving_array->correct_data();
  #endif
//  PS2executePressed();
  ET_ps2.sendData();
  BluetoothSerial.flush();
  delay(wirelessdelay);
}
void Moving_array :: get_initial_data()
{
  for (int i = 0; i < 5; i++)
  {
    getPS2value();
    this->RX[i] = ps2x.PS2data[5]; //data.Rx;
    this->RY[i] = ps2x.PS2data[6]; //data.Ry;
    this->LX[i] = ps2x.PS2data[7]; //data.Lx;
    this->LY[i] = ps2x.PS2data[8]; //data.Ly;
  }

}

void Moving_array :: moving_data()
{
  for (int i = 0; i < 4; i++)
  {
    this->RX[i] = this->RX[i + 1];
    this->RY[i] = this->RY[i + 1];
    this->LX[i] = this->LX[i + 1];
    this->LY[i] = this->LY[i + 1];
  }
  this->RX[4] = ps2x.PS2data[5]; //data.Rx;
  this->RY[4] = ps2x.PS2data[6]; //data.Ry;
  this->LX[4] = ps2x.PS2data[7]; //data.Lx;
  this->LY[4] = ps2x.PS2data[8]; //data.Ly;
}

void Moving_array :: correct_data()
{
  for (int i = 0; i < 5; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      if (this->RX[i] == this->RX[j])
        this->count_moving_rx[i]++;
      if (this->RY[i] == this->RY[j])
        this->count_moving_ry[i]++;
      if (this->LX[i] == this->LX[j])
        this->count_moving_lx[i]++;
      if (this->LY[i] == this->LY[j])
        this->count_moving_ly[i]++;
    }
  }
  uint8_t maxCount_lx = this->count_moving_lx[4];
  uint8_t maxCount_ly = this->count_moving_ly[4];
  uint8_t maxCount_rx = this->count_moving_rx[4];
  uint8_t maxCount_ry = this->count_moving_ry[4];

  uint8_t lx_final = this->LX[4];
  uint8_t ly_final = this->LY[4];
  uint8_t rx_final = this->RX[4];
  uint8_t ry_final = this->RY[4];

  for (int i = 3; i > -1; i--)
  {
    if (this->count_moving_lx[i] > maxCount_lx)
    {
      lx_final = this->LX[i];
      maxCount_lx = this->count_moving_lx[i];
    }
    if (this->count_moving_ly[i] > maxCount_ly)
    {
      ly_final = this->LY[i];
      maxCount_ly = this->count_moving_ly[i];
    }
    if (this->count_moving_rx[i] > maxCount_rx)
    {
      rx_final = this->RX[i];
      maxCount_rx = this->count_moving_rx[i];
    }
    if (this->count_moving_ry[i] > maxCount_ry)
    {
      ry_final = this->RY[i];
      maxCount_ry = this->count_moving_ry[i];
    }
  }
    
//    Serial.println(String(ly_final) + "    " + String(lx_final) + "   " + String(ry_final) + "    " + String(rx_final));
    data.Ly=ly_final;
    data.Lx=lx_final;
    data.Ry=ry_final;
    data.Rx=rx_final;

     
//  Serial.println(String(data.Ly) + "    " + String(data.Lx) + "   " + String(data.Ry) + "    " + String(data.Rx) + "               " + String(ly_final) + "    " + String(lx_final) + "   " + String(ry_final) + "    " + String(rx_final)); // tocheck base values

  for (int i = 0; i < 5; i++)
  {
    count_moving_lx[i] = 0;
    count_moving_ly[i] = 0;
    count_moving_rx[i] = 0;
    count_moving_ry[i] = 0;

    maxCount_lx = 0;
    maxCount_ly = 0;
    maxCount_rx = 0;
    maxCount_ry = 0;
  }
}

