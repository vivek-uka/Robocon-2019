int TZCount = 0; 
int speedFlag = 0;
int gripFlagManual = 0;
void scalePS2value(){
      //SerialX.println("sahi");
      ySquare_leftdistance=(basePS2_Y-ly)/basePS2_Y;
      xSquare_leftdistance=(lx-128)/128.0;
      ySquare_rightdistance=(basePS2_Y-ry)/basePS2_Y;
      xSquare_rightdistance=(rx-128)/128.0;
      if(xSquare_leftdistance==0 && ySquare_leftdistance==0){
      xCircle_leftdistance=0;
      yCircle_leftdistance=0; 
      }
      else{
      xCircle_leftdistance= xSquare_leftdistance *sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2)-(pow(xSquare_leftdistance,2)*pow(ySquare_leftdistance,2)))/sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2));
      yCircle_leftdistance= ySquare_leftdistance *sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2)-(pow(xSquare_leftdistance,2)*pow(ySquare_leftdistance,2)))/sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2));
      }
      if(xSquare_rightdistance == 0 && ySquare_rightdistance ==0){
      xCircle_rightdistance = 0;
      yCircle_rightdistance = 0; 
      }
      else{
      xCircle_rightdistance = xSquare_rightdistance *sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2)-(pow(xSquare_rightdistance,2)*pow(ySquare_rightdistance,2)))/sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2));
      yCircle_rightdistance = ySquare_rightdistance *sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2)-(pow(xSquare_rightdistance,2)*pow(ySquare_rightdistance,2)))/sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2));
      }
      
      LeftAnalogDistance = sqrt((xCircle_leftdistance)*(xCircle_leftdistance)+(yCircle_leftdistance)*(yCircle_leftdistance));
      LeftAnalogTheta = atan2(yCircle_leftdistance,xCircle_leftdistance)*(180.0/3.1415926);
      LeftAnalogTheta = (int(LeftAnalogTheta)+360)%360;
      RightAnalogDistance = sqrt((xCircle_rightdistance)*(xCircle_rightdistance)+(yCircle_rightdistance)*(yCircle_rightdistance));
      RightAnalogTheta = atan2(yCircle_rightdistance,xCircle_rightdistance)*(180.0/3.1415926);
      RightAnalogTheta = (int(RightAnalogTheta)+360)%360;

      
      if(LeftAnalogDistance == 0.00){
        LeftAnalogTheta = 0;
      }
      if(RightAnalogDistance==0.00){
        RightAnalogTheta=0;
      }
      
} 
void operateManually()
{
  
  receivePS2data();
  scalePS2value();
  PS2executePressed();
  
  uint16_t orientationAngle = 270;//  For Abdullah Sir : 270
  float omegaSpeed = 1.8;     
  
      if(((RightAnalogTheta < 90)&&(RightAnalogTheta >= 0))||((RightAnalogTheta < 360)&&(RightAnalogTheta > 270)))
      {
        calculateSpeed(-RightAnalogDistance*omegaSpeed,DegreeToRadian((LeftAnalogTheta- orientationAngle)),LeftAnalogDistance* speeed); 
      }
      if((RightAnalogTheta<270)&&(RightAnalogTheta>90))
      {
        calculateSpeed(RightAnalogDistance*omegaSpeed,DegreeToRadian((LeftAnalogTheta- orientationAngle)),LeftAnalogDistance* speeed);  
      }
      if(LeftAnalogDistance<=0.01 && RightAnalogDistance<=0.01)
      {
        brakeWheels();
      }

//*********************************************                         INDICATION OF SPEED                       ***********************************************

      if(!speedFlag)
      {
        speeed = 200;
        yellowLEDoff;
      }
      else if(speedFlag == 1)
      {
        yellowLEDon;
        speeed = 250;     
      }
      else
      {
        blinkLED(yellow);
        speeed = 300;
      }


//*********************************************                         MECHANISM ACTUATION                   ***********************************************

    if(gripFlagManual == 1)
    {
      switchLoad(GripDown); 
      if(millis()-gripperTimeOut > 1000)
      {
        gripFlagManual = 0;
        pLoadingMotor->actuateMotor(0, stopp);
      }
    }

    else if(gripFlagManual == 2)
    {
      switchLoad(GripUp);
      if(millis()-gripperTimeOut > 2000)
      {
        gripFlagManual = 0;
        pLoadingMotor->actuateMotor(0, stopp);
      }
    }

    else if(gripFlagManual == 3)
    {
      switchLoad(GripDown);
      if(millis()-gripperTimeOut > 800)
      {
        gripFlagManual = 0;
        pLoadingMotor->actuateMotor(0, stopp);
        switchLoad(ThrowShagai);
      } 
    }
}  

void receivePS2data(){
  
  ps2x.last_buttons=ps2x.buttons;
  if(ET_ps2.receiveData()){
        
    while(ET_ps2.receiveData()>0);
    delay(10);
    ps2x.buttons=data.ps2data;
    rx=data.Rx;
    ry=data.Ry;
    lx=data.Lx;
    ly=data.Ly;  

  }
}


///////////////////////////////////////////////////////////////////                            PS2 ENDED                            ///////////////////////////////////////////

 
void PS2executePressed()
{    
    ps2release_count++;
    if(ps2release_count >3)         
     { 
        if(ps2x.ButtonPressed(PSB_L2))  
        {
          if(speedFlag)
              speedFlag--;
        }   
        if(ps2x.ButtonPressed(PSB_R2))  
        {
          if(speedFlag != 2)
              speedFlag++;
        }
        if(ps2x.ButtonPressed(PSB_PAD_DOWN)) 
        {
          getYaw();
          mpuConst += robotYaw;
          safeFlag2 = 1;
          resetXY();
          state = reachShagai1;
          switchLoad(ReleaseShagai);
        }
        if(ps2x.ButtonPressed(PSB_PAD_UP))  //releaseShagai
        {
         // if(safeFlag2 == 1)
          {
            switchLoad(ReleaseShagai);
            gripFlagManual = 1;
            gripperTimeOut = millis();
            safeFlag2 = 2;
          }          
        }
        if(ps2x.ButtonPressed(PSB_SELECT))
        { 

        }
        if(ps2x.ButtonPressed(PSB_START))
        {
            
        }
        if(ps2x.ButtonPressed(PSB_TRIANGLE))  //grabShagai
        {
         // if(safeFlag2 == 2)
          { 
            switchLoad(GrabShagai);
            gripFlagManual = 2;
            gripperTimeOut = millis();
            safeFlag2 = 3;
          }    
        }  
        if(ps2x.ButtonPressed(PSB_CIRCLE)) 
        {  
          
        } 
         
        if(ps2x.ButtonPressed(PSB_CROSS))  //throwing
        {
          if(safeFlag2 == 3)
            {
              switchLoad(ReleaseShagai);
              gripFlagManual = 3;
              gripperTimeOut = millis();              
              safeFlag2 = 1;
            }
        }
         
        if(ps2x.ButtonPressed(PSB_SQUARE)) //bridge
        {
          if(!(bridgeCounter%3))
            state = bridge;
           bridgeCounter++;
          
        }
       
        if(ps2x.ButtonPressed(PSB_L1))
        { 
          if(!secondTimeServoFlag)
          {
            hisServo.write(0);
            secondTimeServoFlag = 2;
          }
          else if(secondTimeServoFlag == 2)
          {
            rotateServo(173);
            secondTimeServoFlag = 1;
            firstTimeBridgeFlag = 0;
            firstTimeServoFlag = 1;
          }
        }
       
        if(ps2x.ButtonPressed(PSB_PAD_LEFT))
        { 
          
        }

        if(ps2x.ButtonPressed(PSB_R1))
        {
          if(!firstTimeBridgeFlag && firstTimeServoFlag)
            hisServo.write(90);
          else if(firstTimeServoFlag)
          {
            rotateServo(77);
            firstTimeServoFlag = 0;
          }
          else if(!firstTimeServoFlag)
           {
             hisServo.write(90);
             secondTimeServoFlag = 0;
           }
        }

        if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
        {
            
        } 
     delay(50);   
     }
}
void checkManual()
{
    if(ET_ps2.receiveData())
    {
       ps2x.last_buttons = ps2x.buttons;
       while(ET_ps2.receiveData()>0);
       ps2x.buttons = data.ps2data;
    }
    if(ps2x.ButtonPressed(PSB_CIRCLE))
     state = completeManual;
}

