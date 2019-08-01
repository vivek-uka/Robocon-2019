uint16_t shagaiCounter = 0;
void initLoad()
{
   for(int i = grip; i<=throww;++i)
     pinMode(pneumaticPin[i],OUTPUT);
   
   pinMode(limitSwitchForward,INPUT_PULLUP);
   pinMode(limitSwitchBackward,INPUT_PULLUP); 
}

int pwmmm = 80, forwardPwm = 80, backwardPwm = 145, delayy = 500;//70;//135;
bool switchLoad(Mechanism mstate)
{
  if(state == completeManual)
    delayy = 0;
  else
    delayy = 500;
    
  loadState = mstate;
  switch(loadState)
  { 
    case GrabShagai:
              //SerialX.println("<<  GRIP SHAGHAI  >>");
              digitalWrite(pneumaticPin[grip],HIGH);
              delay(500);
              digitalWrite(pneumaticPin[grip],LOW);
              delay(delayy);
              return 1;
              
    case ReleaseShagai: 
              //SerialX.println("<<  GRIP SHAGHAI  >>");
              digitalWrite(pneumaticPin[releasee],HIGH);
              delay(500);
              digitalWrite(pneumaticPin[releasee],LOW);
              delay(delayy);
              return 1;  
               
    case GripDown:
              //SerialX.println("<<  GRIP SHAGHAI  >>");
              if(readSwitch(limitSwitchForward))
                {
                  pLoadingMotor->actuateMotor(pwmmm, stopp);
                  return 0;
                }
              else
                {
                  pwmmm = forwardPwm;                    
                  pLoadingMotor->actuateMotor(forwardPwm, forward);                
                }
              return 1;

    case GripUp :
              //SerialX.println("<<  LOAD SHAGAI  >>");
              if(readSwitch(limitSwitchBackward))
                {
                  pLoadingMotor->actuateMotor(pwmmm, stopp);
                  return 0;
                }
              else
                {
                  pwmmm = backwardPwm;                    
                  pLoadingMotor->actuateMotor(backwardPwm, backward);                
                }
              safeFlag1 = 1;
              return 1;
    case ThrowShagai :
              digitalWrite(pneumaticPin[throww],HIGH);
              delay(700);
              digitalWrite(pneumaticPin[throww],LOW);
              delay(delayy);      //2000
              return 1;
  }
}

//void switchThrow()
//{
//  switch(throwState)
//  {
//    case Release: 
//               SerialX.println("<<  RELEASE >>");
//              digitalWrite(pneumaticPin[releasee],HIGH);
//              delay(500);
//              digitalWrite(pneumaticPin[releasee],LOW);
//              throwState = AvoidCollision;
//              pwmmm = 70;
//              break;
//    
//    case AvoidCollision :
//              SerialX.println("<<  AvoidCollision >>");
//              if(readSwitch(limitSwitchForward))
//              {
//                pLoadingMotor->actuateMotor(pwmmm, stopp);
//                loadState = GrabShagai;
//                throwState = Release;
//                stopThrow = 1;
//                safeFlag2 = 1;
//              }
//              else
//                pLoadingMotor->actuateMotor(forwardPwm, forward);
//              break;
//  }
//}

bool readSwitch(int limitSwitch)
{
//    SerialX.print(digitalRead(limitSwitch));
//    SerialX.print("     ");
//    SerialX.println(LimitCounter);
    if(digitalRead(limitSwitch) == LOW)
    {
      LimitCounter++;
      if(LimitCounter > 8)
      {
        LimitCounter = 0;
        return 1;
      }
      else
        return 0;
    }
    else
    {
      LimitCounter = 0;
      return 0;
    }

}

void checkShagai(void)
{
  if(digitalRead(proximityPin))
    shagaiCounter++;
  else
    shagaiCounter = 0;
  if(shagaiCounter >= 5)
    {
//      odometryState = stopBot;
      fsmFlag = 0;
      rotateFlag = 0;
      gripFlag = 0;
      throwFlag = 0;
      shagaiCounter = 0;
    }

//  Serial.println(String(shagaiCounter) + "    " + String(digitalRead(proximityPin)) );
}

void grabRelease(void)
{
    if(grabFlag)
  {
    if(gripFlag == 2)
      {
        switchLoad(GrabShagai);
        gripFlag = 1;
        gripperTimeOut = millis();
      }
      else if(gripFlag == 1)
      {
        gripFlag = switchLoad(GripUp);
        if(millis()-gripperTimeOut>2000)
        {
           gripFlag = 0;
           pLoadingMotor->actuateMotor(0, stopp);
        }
      }
  }

  if(releaseFlag)
  {
    if(gripFlag == 0)
  {
    switchLoad(ReleaseShagai);
    gripFlag = 8;
    gripperTimeOut = millis();
  }
  else if(gripFlag == 8)
    {
      gripFlag = switchLoad(GripDown);
      if(millis()-gripperTimeOut>2000)
      {
         gripFlag = 3;
         pLoadingMotor->actuateMotor(0, stopp);
      }
    }
  }
}

