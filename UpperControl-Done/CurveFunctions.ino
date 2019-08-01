bool traceLine(float x1,float y1,float x2,float y2,float vel,int quadrant,bool mode, int tolerance)    //quadrant - ALL_QUAD //mode =1 ==continue
{

  float dist_now = dist(pBot->X_pos,pBot->Y_pos,x2,y2);
  float angle_fix = Angle(x1,y1,x2,y2);                                                                
  float angle_now = Angle(pBot->X_pos,pBot->Y_pos,x2,y2);
  
  float dist_fix = dist(x1,y1,x2,y2);
  float dist_cover = dist(pBot->X_pos,pBot->Y_pos,x1,y1);

  
  if(angle_now != angle_fix)
    angle_fix = angle_now;

    if(tolerance>0)  
    {
      if(dist_now <= tolerance)
      {
        if(mode == STOP)
        {
          calculateSpeed(-omegacontrol, angle_fix, 0);
        }
        sigmoidFlag=0;
        return 0;
      }
    }
    
 switch(quadrant)
    {
      case ALL_QUAD :
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;
      
      case ONE_FOUR :
        if(angle_fix>pi/2 && angle_fix<3*pi/2)
        {
          sigmoidFlag=0;
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;
        
      case TWO_THREE :
        if(!(angle_fix>pi/2 && angle_fix<3*pi/2))
        {
          sigmoidFlag=0;
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;
         
      case ONE_TWO :
        if(!(angle_fix>0 && angle_fix<pi))
        {
          sigmoidFlag=0;
          if(mode == STOP)
            calculateSpeed(-omegacontrol,angle_fix, 0);
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;
        
      case NOT_TWO :
        if(angle_fix>pi/2 && angle_fix<pi)
        {
          sigmoidFlag=0;
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;

      case NOT_THREE :
        if(angle_fix>pi && angle_fix<3*pi/2)
        {
          sigmoidFlag=0;
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;

      case NOT_ONE :
        if(angle_fix>0 && angle_fix<pi/2)
        {
          sigmoidFlag=0;
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;

      case NOT_FOUR :
        if(angle_fix>3*pi/2 && angle_fix<2*pi)
        {
          sigmoidFlag=0;
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;

      case ONE :
        if(!(angle_fix>0 && angle_fix<=pi/2))
        {
          sigmoidFlag=0;
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;

       case THREE_FOUR:
         if(angle_fix>0 && angle_fix<pi)
        {
          sigmoidFlag=0;
          if(mode == STOP)
            calculateSpeed(-omegacontrol,angle_fix, 0);
          return 0;
        }
        calculateSpeed(-omegacontrol, angle_fix, vel);
        return 1;
      
    }
}
bool traceLinePID(float x1,float y1,float x2,float y2,float vel,int quadrant,int mode, float tolerance,float lookAhead)//, float finalAngle)    //quadrant - ALL_QUAD //mode =1 ==continue
{
  float xd=0,yd=0,x=0;
  float angle_fix = Angle(x1,y1,x2,y2);

    if(angle_fix == 0)
    {
      if(x2 > x1)
        {
          xd = pBot->X_pos + lookAhead;
          if(xd >= x2)
            xd = x2;
        }
      else
        {
          xd = pBot->X_pos - lookAhead;
          if(xd <= x2)
            xd = x2;
        }
      yd = y1;
    }
    else if(angle_fix == pi/2)
    {
        xd = x1;
        if(y2 > y1)
          {
            yd = pBot->Y_pos + lookAhead;
            if(yd >= y2)
              yd = y2;
          }
        else 
          {
            yd = pBot->Y_pos - lookAhead;
            if(yd <= y2)
              yd = y2;
          }
    }
    else
    {
      x = ((tan(angle_fix))*(pBot->Y_pos - y2) + (tan(angle_fix) * tan(angle_fix))*x2 + pBot->X_pos)/((tan(angle_fix) * tan(angle_fix)) + 1);
      
      if(x2 > x1)
        {
          xd = x + (lookAhead/sqrt((tan(angle_fix) * tan(angle_fix)) + 1));
          if(xd >= x2)
            xd = x2;
        }
      else
        {
          xd = x - (lookAhead/sqrt((tan(angle_fix) * tan(angle_fix)) + 1));
          if(xd <= x2)
            xd = x2;
        }
      
      yd = y2 + tan(angle_fix) * (xd - x2);
    }

  
  float angle_now = Angle(pBot->X_pos,pBot->Y_pos,xd,yd);
  float dist_perpendicular = dist(pBot->X_pos,pBot->Y_pos,xd,yd);
  float dist_now = dist(pBot->X_pos,pBot->Y_pos,x2,y2);
  float angle_goal = Angle(pBot->X_pos,pBot->Y_pos,x2,y2);
  float dist_cover = dist(pBot->X_pos,pBot->Y_pos,x1,y1);
  float dist_fix = dist(x1,y1,x2,y2);
  
  if(angle_now != angle_fix)
    angle_fix = angle_now;

    
    if(state==TZ1 || state==TZ2 || state==TZ3 || state==reachShagai2 || state==reachShagai3)//state != reachShagai1 && state != NearTZ && state != towardsGobi)
      vel = velocityLine(dist_cover, dist_fix, maxSpeeed, minSpeeed);
    
      if(dist_now <= tolerance)
      {
         if(mode == STOP)
         {
            calculateSpeed(-omegacontrol, angle_fix, 0);
         }
         return 0;
      }
      
    switch(quadrant)
    {
      case ALL_QUAD :
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;
      
      case ONE_FOUR :
        if(angle_goal>pi/2 && angle_goal<3*pi/2)
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;
        
      case TWO_THREE :
        if(!(angle_goal>pi/2 && angle_goal<3*pi/2))
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;
         
      case ONE_TWO :
        if(!(angle_goal>0 && angle_goal<pi))
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;
        
      case NOT_TWO :
        if(angle_goal>pi/2 && angle_goal<pi)
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;

      case NOT_THREE :
        if(angle_goal>pi && angle_goal<3*pi/2)
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;

      case NOT_ONE :
        if(angle_goal>0 && angle_goal<pi/2)
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;

      case NOT_FOUR :
        if(angle_goal>3*pi/2 && angle_goal<2*pi)
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;

      case ONE :
        if(!(angle_goal>0 && angle_goal<=pi/2))
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;

      case THREE_FOUR :
        if((angle_goal>0 && angle_goal<=pi))
        return 0;
        calculateSpeed(-omegacontrol, angle_fix, vel+abs(ppidDistance->pidControl(dist_perpendicular)));
        return 1;  
      
    }
    return 1;
}

//______________________________________________________________________________________________________
Curve :: Curve()
{
  k=0;
  x1 = 0;
  y1 = 0;
  x2 = 0;
  y2 = 0;
  flag = 0;
  i = 0;
 
}

bool Curve :: traceForest(float vel)
{
   if(flag)
   {
    if(arenaFlag == RED)
      flag = traceLinePID(xaRED[i-1]+shiftX,yaRED[i-1]+shiftY,xaRED[i]+shiftX,yaRED[i]+shiftY,vel,ALL_QUAD,CONTINUE,25,20);
    else
      flag = traceLinePID(xaBLUE[i-1]+shiftX,-yaBLUE[i-1]-shiftY,xaBLUE[i]+shiftX,-yaBLUE[i]-shiftY,vel,ALL_QUAD,CONTINUE,25,20);
   }
   else 
   {  
     i++;
     flag = 1;
   }
  
   if(pBot->X_pos >= 5*sineWavelength/4 + shiftX + 20)
    {
      flag = 0;
      k = 0;
      return 0;
    }
   return 1;
}

bool Curve :: traceArc(float theta,float Delta, float radius, float vel)
{
  
  if(flag)
    flag = traceLine(x1+shiftX,y1+arenaFlag*shiftY,x2+shiftX,y2+arenaFlag*shiftY,vel,arcQuad,CONTINUE,5);
  else
  {
    if(k < theta)
      {
        k += Delta;
        if(k > theta)
          k = theta;
      }
      else
      {
        shiftX += radius;
        shiftY += radius;
        k = 0;
        return 0;
      }
    x1 = x2;
    y1 = y2;
    y2 = arenaFlag*(-radius*cos(k) + radius);
    x2 = radius*sin(k);
    
    
      flag = 1;
     }
     return 1;
}

float sineV = 25.0, vPoint = 0;
bool Curve :: traceSine(float delta_x,float amplitude, float wavelength, float vel)
{
   k = 2.0*pi/wavelength;
   if(flag)
     flag = traceLinePID(x1+shiftX,y1+arenaFlag*shiftY,x2+shiftX,y2+arenaFlag*shiftY,vel,ONE_FOUR,CONTINUE,10, 15);    
   else 
   {  
      x1 = x2;
      y1 = y2;
      x2 = x2 + delta_x;
      y2 = amplitude*sin(k*x2);
      flag = 1;    
   }
  if(pBot->X_pos >= 5*wavelength/4 + shiftX + 15)
    {
      flag = 0;
      k = 0;
      return 0;
    }
   return 1;
}

float velocitySine(float x, float offSet, float baseSpeed)
{
  float waveConstant = 2*pi/sineWavelength;
  return offSet + baseSpeed*cos(x*waveConstant)*cos(x*waveConstant);
}

void setDelta_x()
{
  float check = pBot->X_pos  - shiftX;// + 15;
  if(check  < lambda_not)
    deltta_x = lambda_not;
  else if(check  < sineWavelength/2 - lambda_not && check  >= lambda_not)
    deltta_x = (sineWavelength/2 - 2*lambda_not);
  else if(check  < sineWavelength/2 + lambda_not && check  >= sineWavelength/2 - lambda_not)
    deltta_x = lambda_not;
  else if(check  < sineWavelength - lambda_not && check  >= sineWavelength/2 + lambda_not)
    deltta_x = (sineWavelength/2 - 2*lambda_not);  
  else if(check  < sineWavelength + lambda_not && check  >= sineWavelength - lambda_not)
    deltta_x = lambda_not;//2*20;
  else if(check  < 3*sineWavelength/2 - lambda_not && check  >= sineWavelength + lambda_not)
    deltta_x = (sineWavelength/2 - 2*lambda_not);
  else if(check  < 3*sineWavelength/2 && check  >= 3*sineWavelength/2 - lambda_not)
    deltta_x = lambda_not;   

  //Serial.println("  dx  "+String(deltta_x));
}

//______________________________________________________________________________________________________
