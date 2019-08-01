
void finiteStateMachine()
{
  if((state != checkArena) && (state != waitForMR2) && (state != completeManual) && (state != afterAlignGobi) && (state != startSignal))
  {
    getYaw(); 
    handleXY();
    omegacontrol = OmegaControlMPU(robotYaw,2);
    checkManual();
  }
  
  switch(state)
  {
    case mpuCalibration:
                      calculateSpeed(0,0,0);
                      if(abs(robotYaw - prevYaw) <= 1.0 && millis() - calibrationTime > 2000)
                      {
                        mpuConst = robotYaw;
                        state = checkArena;
                        whiteLEDon;
                      }
                      break;
                      
    case checkArena:
                      calculateSpeed(0,0,0);
                      if(ET_ps2.receiveData())
                      {
                        ps2x.last_buttons = ps2x.buttons;
                        while(ET_ps2.receiveData()>0);
                        ps2x.buttons = data.ps2data;
                      }
                      if(ps2x.ButtonPressed(PSB_L1))
                      { 
                        arenaFlag = RED;
                        arcQuad = ONE_FOUR;
                        gobiQuad = ONE_TWO;
                        state = startSignal;
                        colorr = red;
                        redLEDon;
                        blueLEDoff;
                        startX = 70;
                        startY = 84;
                        switchLoad(GrabShagai);   
                        reachShagaiY = 60;                                                  
                      }
                      if(ps2x.ButtonPressed(PSB_R1))
                      { 
                        arenaFlag = BLUE;
                        arcQuad = ONE_FOUR;
                        gobiQuad = THREE_FOUR; 
                        state = startSignal;
                        colorr = blue;
                        blueLEDon;
                        redLEDoff;
                        startX = 69;
                        startY = 63;
                        switchLoad(GrabShagai);
                        reachShagaiY = 65;
                      }
                      break;
                      
    case startSignal:
                      calculateSpeed(0,0,0);
                      if(ET_ps2.receiveData())
                      {
                        ps2x.last_buttons = ps2x.buttons;
                        while(ET_ps2.receiveData()>0);
                        ps2x.buttons = data.ps2data;
                      }
                      if(ps2x.ButtonPressed(PSB_CROSS)) 
                      {
                        state = khangaiUrtuu;
                        if(!digitalRead(resetAlignReachS1Button))
                        {
                          state = reachShagai1;
                          switchLoad(ReleaseShagai);
                        }
                        if(!digitalRead(resetAlignNearTZButton))
                        {
                          state = gobiToRotate;
                          greenLEDon;    
                        }
                      }
                      break;    
                          
    case khangaiUrtuu:
                      fsmFlag = traceLine(0,0,startX,arenaFlag*startY, speeed, ALL_QUAD, CONTINUE,15);
                      if(!fsmFlag)
                      {
                        fsmFlag = 1;
                        state = forest;
                        //_____SHIFTS______//
                        shiftX = startX;
                        shiftY = startY;
                        controlFlag = 0;
                        prevVt = speeed;
                      }
                      break;

    case forest:                             
                      if(fsmFlag)
                      {
                        state = forest; 
                        int xx = pBot->X_pos - shiftX;
                        speeed = velocitySine(xx+45,180,300);  // offset+baseSpeed*cos(k*x)^2 //140
//                        setDelta_x();
                        fsmFlag = curve.traceForest(speeed); 
//                        fsmFlag = curve.traceSine(deltta_x,arenaFlag*sineAmplitude,sineWavelength,speeed);
                      }                     
                      else 
                      {
                        state = alignBridge;
                        fsmFlag = 1;

                        // Delhi
                          if(arenaFlag == RED)
                              shiftX += 3*sineWavelength/2;// + 25;
                          else
                              shiftX += 3*sineWavelength/2 + 20;
                        
                        speeed = 5;
                        alignX = pBot->X_pos;
                        alignY = pBot->Y_pos;
                      }        
                      break;
                      
    case alignBridge:
                      blinkLED(colorr);
                      fsmFlag = traceLine(alignX, alignY, shiftX, arenaFlag*shiftY,speeed,ALL_QUAD,CONTINUE,10);
                      reading = GetLSAReading(ActiveSensor);
                      speeed += 5;
                      if(speeed > 300)
                        speeed = 300;
                      if(abs(reading) < 36 && arenaFlag*pBot->Y_pos <= shiftY + 20 && pBot->X_pos >= shiftX - 20)
                      {
                        state = bridge;
                        speeed = 150;
                        Linecontrol = LineControl(reading,30,35,pLinegain);
                        calculateSpeed(-omegacontrol, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),speeed);
                        randomCtr = 0;
                        digitalWrite(colorPin[colorr],LOW);
                      }
                      break;

    case bridge: 
                      if(!junctionFlag2)
                        reading = GetLSAReading(ActiveSensor);
                      else
                      {
                       reading = previousReading;
                       junctionFlag2 = 0;
                      }
                     
                      Linecontrol = LineControl(reading,30,35,pLinegain);
                      if(abs(GetLSAReading(ActiveOmegaSensor)) > 36 && OmFlag)
                      {
                       speeed = 150;
                       calculateSpeed(-omegacontrol, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),speeed);                   
                      }
                      else 
                      { 
                        speeed += 50;
                        if(speeed > 400)
                          speeed = 400;
                        OmFlag = 0;
                        omegaLine = -OmegaControl(reading,GetLSAReading(ActiveOmegaSensor),40,pOmegagain);
                        calculateSpeed(omegaLine, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),speeed);
                      }
                      if(digitalRead(LSAArray[f]->JunctionPin))
                      {
                        if(!junctionFlag)
                        {
                          resetXY();
                          junctionFlag2 = 1;
                        }
                        junctionFlag = 1;
                        speeed = 300;
                        break;
                      }
                      else if(pBot->X_pos >= 90 && junctionFlag)
                      {
                        //Delhi
                        randomCtr = 1;  
                        
                        shiftX = 90;
                        fsmFlag = 1;
                        rotateServo(77);
                        OmFlag = 1;
                        controlFlag = 0;
                        state = khangaiArea;

                        if(arenaFlag == RED)
                          speeed = 300;
                        else
                          speeed = 150;
                      }
                      else if(junctionFlag)
                      {
                        if(!randomCtr)
                          errorBridge = ((reading)*1.5*3.0)/35.0;
                        randomCtr = 1;
                      }
                      previousReading = reading;
                      break;
                      
    case khangaiArea:
                      fsmFlag = curve.traceArc(pi/2,pi/6,80,speeed);  //angle subtend, delta_angle, radius, speed
                      redLEDon;
                      if(!fsmFlag)
                      {
                        state = towardsGobi;
                        fsmFlag = 1;
                        randomCtr = 1;
                        if(arenaFlag == RED)
                          speeed = 400;
                        else
                          speeed = 200;
                        redLEDon;
                      }
                      break;
                      
    case towardsGobi:
                     digitalWrite(colorPin[colorr],LOW);
                     if(randomCtr)
                        fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,175,arenaFlag*280,speeed,gobiQuad,CONTINUE,25,70);
                     else
                        fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,140,arenaFlag*480,speeed,ALL_QUAD,CONTINUE,30,35);
                                     
                     if(!fsmFlag)                     
                     {
                      if(randomCtr)  
                      {
                        randomCtr = 0;
                        fsmFlag = 1;
                        shiftX = 140;
                        shiftY = 280;
                        speeed = 200;
                        hisServo.write(90);
                      }
                      else               
                      {
                        calculateSpeed(-omegacontrol, 0, 0);
                        if(delayCounter > 3)
                        {
                          state = alignGobi;
                          fsmFlag = 1;
                          shiftX += -10;
                          shiftY += 390;
                          delayCounter = 0;
                          speeed = 100;
                          safeFlag2 = 1;
                        }
                        delayCounter ++;
                      }
                     }
    
                     break;

    case alignGobi:  
                     calculateSpeed(-omegacontrol, arenaFlag*alignGobiAngle, speeed);
                     if(delayCounter > 20)                   
                     {
                        if(BotY - prevBotY == 0 && alignGobiAngle == pi/2)
                        {
                          alignGobiAngle = pi;
                          delayCounter = 0;
                          break;
                        }
                        if(BotX - prevBotX ==0 && alignGobiAngle == pi)
                        {
                          speeed = 100;  
                          alignGobiAngle = 3*pi/4;
                          delayCounter = 0;
                          break;
                        }
                        
                        if(BotX - prevBotX == 0 && BotY - prevBotY == 0 && alignGobiAngle == 3*pi/4 )
                        {
                          speeed = 100;  
                          calculateSpeed(-omegacontrol,0,0);
                          resetXY();
                          odometryFlag = 0;
                          state = afterAlignGobi;
                          switchLoad(ReleaseShagai);
                          delayCounter = 0;
                          speeed = 100;
                          errorBridge = 0;
                          calibrationTime = millis();
                          randomCtr = 0;
                        }
                          prevBotX = BotX;
                          prevBotY = BotY;
                     }
                     delayCounter++;   
                     break;
                    
    case afterAlignGobi:
                     calculateSpeed(0,0,0);
                     if(ET_ps2.receiveData())
                     {
                       ps2x.last_buttons = ps2x.buttons;
                       while(ET_ps2.receiveData()>0);
                       ps2x.buttons = data.ps2data;                 
                     }
                     if(ps2x.ButtonPressed(PSB_PAD_DOWN) && randomCtr)
                     { 
                       state = reachShagai1;
                       mpuConst += robotYaw;
                       hisServo.write(0);
                     }
                     
                     if(millis() - calibrationTime <= 3000)
                     {
                       getYaw();
                       digitalWrite(colorPin[2-colorr],LOW);
                     }
                     else
                     {
                       randomCtr = 1;
                       digitalWrite(colorPin[2-colorr],HIGH);
                     }
                     
                     break;
                     
    case reachShagai1:
                     
                     speeed = 100;
                     ppidOmega->required = 0;
                     if(fsmFlag)
                     {
                       fsmFlag = traceLinePID(0, 0, 68, -arenaFlag*reachShagaiY , speeed, ALL_QUAD, STOP, 5, 15);//,0);
                       state = reachShagai1;
                       gripperTimeOut = millis();                       
                     }
                     else
                     {

                       calculateSpeed(-omegacontrol,0,0);
                       if(gripFlag)
                       {
                         gripFlag = switchLoad(GripDown);
                         if(millis()-gripperTimeOut > 2000)
                         {
                           gripFlag = 0;
                           pLoadingMotor->actuateMotor(0, stopp);
                         }
                       }
                       else
                       {
                         speeed = 170;
                         shiftX = pBot->X_pos;
                         shiftY = pBot->Y_pos;
                         resetFlags();
                         switchLoad(GrabShagai);
                         state = NearTZ;
                         gripperTimeOut = millis(); 
                         randomCtr = 1;
                       }
                     }
                     break;
    case NearTZ :
                
                    if(gripFlag)
                     {
                      gripFlag = switchLoad(GripUp);
                      if(millis()-gripperTimeOut>2000)
                      {
                        gripFlag = 0;
                        pLoadingMotor->actuateMotor(0, stopp);
                      }
                     }
                    else
                      {
                        Axis = X_axis;
                        if(randomCtr)
                          fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,19,arenaFlag*(-80),speeed,ALL_QUAD,STOP,5,15);
                        else
                          fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,20,arenaFlag*(-255),speeed,ALL_QUAD,STOP,5,15);
                      }
                                                     
                    if(fsmFlag)
                      state = NearTZ;
                    else
                    {
                      if(randomCtr)  
                      {
                        randomCtr = 0;
                        fsmFlag = 1;
                        speeed = 170;
                        shiftX = 19;
                        shiftY = -80;
                      }
                      else
                      {  
                        state = rotateNearTZ1;
                        shiftX = 20;
                        shiftY = -255; 
                        resetFlags();
                        rotateServo(180);
                        randomCtr = 1;
                        digitalWrite(colorPin[2-colorr],LOW);
//                        dac.setVoltage(417*3.8,false);
                      }
                    }
                    break;
     
    case rotateNearTZ1 :
                    if(rotateFlag)
                      rotateFlag = alignRotate(25*arenaFlag + 90);
                    else
                      state = waitForMR2;
                    waitTime = millis();
                    break;  
                    
    case waitForMR2:
                    calculateSpeed(0,0,0);
                    if(ET_ps2.receiveData())
                    {
                      ps2x.last_buttons = ps2x.buttons;
                      while(ET_ps2.receiveData()>0);
                      ps2x.buttons = data.ps2data;                 
                    }
                    if(ps2x.ButtonPressed(PSB_PAD_DOWN) && !randomCtr)
                    {
                      state = TZ1;
                      sigmoidFlag = 1;
                      speeed = 300;
                      controlFlag = 1;
                      maxSpeeed = 320;
                      minSpeeed = 150;
                    }
                    if(millis() - waitTime >= 10000)
                    {
                      randomCtr = 0;
                      digitalWrite(colorPin[2-colorr],HIGH);
                    }
                    break;  
                              
    case TZ1 :               
                    if(fsmFlag)
                    {                     
                      state = TZ1;
                      fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,-230-96,arenaFlag*(-255),speeed,ALL_QUAD,STOP,15,50);      
                    }
                    else
                    { 
                      calculateSpeed(0,0,0);
                      if(gripFlag == 2)
                      {
                        switchLoad(ReleaseShagai);
                        gripFlag = 1;
                        gripperTimeOut = millis();
                      }
                      else if(gripFlag == 1)
                      { 
                        gripFlag = switchLoad(GripDown); 
                        if(millis()-gripperTimeOut > 1000)
                        {
                          gripFlag = 0;
                          pLoadingMotor->actuateMotor(0, stopp);
                        }
                      }
                      else
                      {
                          if(throwFlag)
                          {
                            switchLoad(ThrowShagai);
                            throwFlag = 0;
                          }
                          else 
                          {
                            minSpeeed = 150;
                            state = reachShagai2;   
                            resetFlags();
                            speeed = 300;
                            shiftX = -230-96;
                            shiftY = -255;
                            gripFlag = 1;
                            OmFlag = 1;
                            pLinegain->Kp = 0.8;
//                            dac.setVoltage(417*3.8,false);
                          }
                      }
                     }
                     break;
                     
    case reachShagai2 :
                     ppidOmega->Kp = 0.09;
                     reading =100;
                     reading = GetLSAReading(ActiveSensor);
                     
                     if(abs(reading) < 25 && BotX >= -75 && OmFlag)
                       fsmFlag = 2;
                    
                     if(fsmFlag == 1)
                     {
                       fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,20,arenaFlag*(-255),speeed,ALL_QUAD,STOP,5,25); //lookahead:15
                       ppidOmega->required = 0;
                       state = reachShagai2;                       
                     }
                     else if(fsmFlag == 2)
                     {
                       Linecontrol = LineControl(reading,30,35,pLinegain);
                       calculateSpeed(-omegacontrol, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),speeed);                        
                       if(BotX >= 18)
                         {
                          fsmFlag = 0;
                          OmFlag = 0;
                          gripperTimeOut = millis();
                         }
                       
                     }
                     else if(!fsmFlag)
                     {   
                      blueLEDoff;
                      greenLEDon;
                      calculateSpeed(-omegacontrol,0,0);
                      if(gripFlag == 1)
                      {
                        redLEDoff;
                        switchLoad(GripDown); 
                        if(millis()-gripperTimeOut > 2000)
                        {
                          gripFlag = 2;
                          pLoadingMotor->actuateMotor(0, stopp);
                        }
                      }
                      else if(gripFlag == 2)
                      {
                        switchLoad(GrabShagai);
                        gripFlag = 3;
                        gripperTimeOut = millis();
                      }
                      else if(gripFlag == 3)
                      {
                        switchLoad(GripUp);
                        if(millis()-gripperTimeOut>2000)
                        {
                           gripFlag = 0;
                           pLoadingMotor->actuateMotor(0, stopp);
                        }
                      }
                      else
                      {
                        state = TZ2;
                        shiftX = 20;
                        shiftY = -255;
                        resetFlags();
//                        dac.setVoltage(417*3.8,false);                           
                        speeed = 300;
                        OmFlag = 1;
                        reachY = BotY;
                        reachX = 71;
                      }
                    }
                    break;
                    
    case TZ2 :
                    if(fsmFlag)
                    {
                      minSpeeed = 170;
                      fsmFlag = traceLinePID(shiftX, arenaFlag*shiftY,-230-96, arenaFlag*(-255), speeed, ALL_QUAD, STOP, 15,50);
                      ppidOmega->required = 40*arenaFlag + 90;
                      state = TZ2;
                    }
                    else
                    {
                        calculateSpeed(0,0,0);
                        if(gripFlag == 2)
                        {
                          switchLoad(ReleaseShagai);
                          gripFlag = 1;
                          gripperTimeOut = millis();
                        }
                        else if(gripFlag == 1)
                          {
                            switchLoad(GripDown);
                            if(millis()-gripperTimeOut > 1000)
                            {
                              gripFlag = 0;
                              pLoadingMotor->actuateMotor(0, stopp);
                            }
                          }
                        else
                        {
                          if(throwFlag)
                          {
                            switchLoad(ThrowShagai);
                            throwFlag = 0;
                          }
                          else
                          {
                            state = reachShagai3;
                            //________shifts__________
                            shiftX = -230-96;
                            shiftY = -255;
                            resetFlags();
                            gripFlag = 1;
                            minSpeeed = 150;
//                            dac.setVoltage(417*3.8,false);
                           }
                        }
                      }
                    break;
                    
    case reachShagai3:
                    reading = GetLSAReading(ActiveSensor);
                    junctionn = digitalRead(LSAArray[f]->JunctionPin);
                    
                    if(abs(reading) < 26 && BotX >= -75)
                      fsmFlag = 2; 
                    
                    if(fsmFlag == 1)
                    {
                      fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,reachX,reachY,speeed,ALL_QUAD,STOP, 5,8);
                      state = reachShagai3;
                      gripperTimeOut = millis();
                      ppidOmega->required = 0;
                    }
                    else if(fsmFlag == 2)
                    {
                      Linecontrol = LineControl(reading,30,35,pLinegain);
                      calculateSpeed(-omegacontrol, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),speeed);   
                      
                      if(junctionn)
                        fsmFlag = 1;
                    }
                    else
                    {
                        calculateSpeed(-omegacontrol,0,0);
                        if(gripFlag == 1)
                        {
                          switchLoad(GripDown); 
                          if(millis()-gripperTimeOut > 1000)
                          { 
                            gripFlag = 2;
                            pLoadingMotor->actuateMotor(0, stopp);
                          } 
                        }
                        else if(gripFlag == 2)
                          {
                            switchLoad(GrabShagai);
                            gripFlag = 3;
                            gripperTimeOut = millis();
                          }
                          else if(gripFlag == 3)
                          {
                            switchLoad(GripUp);
                            if(millis()-gripperTimeOut>2000)
                            {
                               gripFlag = 0;
                               pLoadingMotor->actuateMotor(0, stopp);
                            }
                          }
                          else
                          {
                            state = TZ3;
                            shiftX = 80;
                            shiftY = -255;
                            resetFlags();
//                            dac.setVoltage(417*3.8,false); 
                            speeed = 300;
                          }
                        }
                    break;
                    
    case TZ3 :
                    if(fsmFlag)
                    {
                      minSpeeed = 150;
                      fsmFlag = traceLinePID(shiftX,arenaFlag*shiftY,-230-96,arenaFlag*(-255),speeed,ALL_QUAD,STOP, 5,50);
                      state = TZ3; 
                      ppidOmega->required = 45*arenaFlag + 90;
                    }
                    else
                    {
                      calculateSpeed(0,0,0);
                        if(gripFlag == 2)
                        {
                          switchLoad(ReleaseShagai);
                          gripFlag = 1;
                          gripperTimeOut = millis();
                        }
                        else if(gripFlag == 1)
                          {
                            gripFlag = switchLoad(GripDown);
                            if(millis()-gripperTimeOut>2000)
                            {
                               gripFlag = 0;
                               pLoadingMotor->actuateMotor(0, stopp);
                            }
                          }
                        else
                        {
                          if(throwFlag)
                          {
                            switchLoad(ThrowShagai); 
                            throwFlag = 0;
                          }
                          else
                          {
                            state = completeManual;
                            resetFlags();
                          }
                        }
                    }

                    break;      

    case completeManual:
                    operateManually();
                    blinkLED(white);
                    break;
                    
    case gobiToRotate:
                    ppidOmega->required = 0;
                    speeed = 175;
                    fsmFlag = traceLinePID(0,0,20,(-255)*arenaFlag, speeed,ALL_QUAD,STOP,5,15);
                    if(!fsmFlag)
                    {  
                      state = rotateNearTZ1;
                      shiftX = 20;
                      shiftY = -255; 
                      resetFlags();
                      rotateServo(180);
                      randomCtr = 1;
                      digitalWrite(colorPin[2-colorr],LOW);
                    }
                    break;
                    
    case lineFollowing:
                   receivePS2data();
                   PS2executePressed();
                   reading = GetLSAReading(ActiveSensor);
                   Serial.println("Active"+String(reading));
                   Linecontrol = LineControl(reading,30,35,pLinegain);
                   if(abs(GetLSAReading(ActiveOmegaSensor)) > 36 && OmFlag)
                     calculateSpeed(-omegacontrol, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),speeed);                   
                   else 
                   {
                     OmFlag = 0;
                     omegaLine = -OmegaControl(reading,GetLSAReading(ActiveOmegaSensor),40,pOmegagain);
                     calculateSpeed(omegaLine, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),speeed);
                     speeed = 400;
                   }
                   break;
                   
    case stopBot:
                   calculateSpeed(0,0,0);
                   whiteLEDoff;
                   break;
          
            
  }
}
