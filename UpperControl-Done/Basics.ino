void handleXY(void)
{
    pBot->prev_X = pBot->X_pos1;    
    pBot->X_pos1 = (pEncoderX->Count * 2 * pi * RadiusXYWheel / pEncoderX->ppr); //cm   
    pBot->del_x = pBot->X_pos1 - pBot->prev_X;  
  
    pBot->prev_Y = pBot->Y_pos1;  
    pBot->Y_pos1 = (pEncoderY->Count * 2 * pi * RadiusXYWheel / pEncoderY->ppr); //cm
    pBot->del_y = pBot->Y_pos1 - pBot->prev_Y;
    
    float r = dist(0,0,pBot->del_x,pBot->del_y);

    float Theta = the(pBot->del_y,pBot->del_x,(int)robotYaw);
    BotX = BotX + r*(cos(Theta));
    BotY = BotY + r*(sin(Theta));
    pBot->X_pos = BotX;
    pBot->Y_pos = BotY;// + errorBridge;
}
void initLEDs()
{
  for(int colourr = blue; colourr <= white; ++colourr)
  {
    pinMode(colorPin[colourr], OUTPUT);
    digitalWrite(colorPin[colourr], LOW);
  }
}

void blinkLED(uint8_t color)
{
  ledBlinkerr[color]++;
  if(ledBlinkerr[color]%5 == 0)
    digitalWrite(colorPin[color],LOW);
  else
    digitalWrite(colorPin[color],HIGH);
}

float dist(float x1,float y1,float x2,float y2)
{
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}


void ScaleWheels(float maximum)
{
  
  float maxR = abs(pWheel[0]->rpm);
  for(int i=0;i<4;++i)
  {
    if(abs(pWheel[i]->rpm)>maxR)
    maxR = abs(pWheel[i]->rpm);
  }
  if(maxR > maximum)
  for(int i=0;i<3;++i)
  {
    pWheel[i]->rpm *= maximum/maxR;
  }
}

float Angle(float x1,float y1,float x2,float y2)
{
  float angle=atan2(y2-y1,x2-x1);
  if(angle<0)
  return 2*pi-abs(angle);
  else if(x1-x2==0 && y1-y2==0)
  return 0;
  else
  return angle;
}

void resetXY()
{
  pEncoderX->Count = 0;
  pEncoderY->Count = 0;
  shiftX = 0;
  shiftY = 0;
  pBot->X_pos = 0;
  pBot->Y_pos = 0;
  pBot->prev_X = 0;
  pBot->prev_Y = 0;
  pBot->X_pos1 = 0;
  pBot->Y_pos1 = 0;
  BotY = 0;
  BotX = 0;
}

bool botWasStopped()
{  
  if((fabs(pBot->X_pos - pBot->prev_X) < 1.0) && (fabs(pBot->Y_pos - pBot->prev_Y) < 1.0))
  {
    controlFlag = 1;
    return 1;
  }
  else 
    return 0;   
}

void brakeWheels(){
  for(int i=0;i<4;i++){
    pWheel[i]->rpm = 0;
  }
}


float the(float del_y,float del_x,int botYaw)
{
  return (atan2(del_y,del_x) - DegreeToRadian(botYaw));
}
void resetFlags()
{
  fsmFlag = 1;
  gripFlag = 2;
  throwFlag = 1;
  rotateFlag = 1;
  sigmoidFlag=1;
  randomCtr = 0;
  delayCounter = 0; 
}
void rotateServo(float angle)
{
  myServo.write(angle);
//  hisServo.write(angle);
}

float trapezoid_sigmoid (float x, float whole_length, int maxSpeed)
{
   float tau = whole_length / 20;
   int tolerance = 12;
   int constant = 7;
   if ( x < constant * tau )
   return maxSpeed*sigmoid(x,tau);
   else if ( x > constant * tau && x < whole_length - constant * tau )
        return maxSpeed;
   else if ( x > whole_length - constant * tau && x < whole_length - tolerance )
        return maxSpeed*sigmoid(-(x-whole_length),tau);
   if ( x >= whole_length - tolerance)
       return 0;
}


bool lineFlagg = 0;
float velocityLine(float x, float whole_length, int maxSpeed, int minSpeed)
{
  float velocity;
  float divisions = 2.5;
  if(x < whole_length/divisions)
  {
    if(!lineFlagg)
      startAccDist(2,500,maxSpeed,Axis);  
    lineFlagg = 1;
  }
  else if(x >= whole_length/divisions && x< whole_length - whole_length/divisions)
  {
    speeed = prevVt;
    controlFlag = 0;
  }
  else
  {
    if(lineFlagg)
      startAccDist(1,-1500,minSpeed,Axis);  
    lineFlagg = 0;
  }
  velocity = speeed;
  return velocity;
}

float sigmoid (float x , float tau)
{
  return (1/(1+exp(-x/tau))); 
}


