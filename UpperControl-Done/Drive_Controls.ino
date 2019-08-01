void calculateSpeed(float Omega , float angle , float Vtranslational)     //Omega rad/s      Vtranslational  m/s
{  
  if(!(state == bridge || state == lineFollowing || fsmFlag==2 || state == completeManual) )
    angle += DegreeToRadian(robotYaw); 

  Vt = Vtranslational;
  
  if(state != forest  && !sigmoidFlag && state != completeManual)
     Vt = controlSpeedDist(prevVt, Vtranslational, delta_xx, accc, Axis);  // vi, vf, dx, a
  
  for(int i=0;i<4;++i)
  {
    pWheel[i]->translationSpeed = Vt * sin(angle - DegreeToRadian(pWheel[i]->angle));
    pWheel[i]->angularSpeed = Omega * RadiusOmniDrive;
    pWheel[i]->Speed = pWheel[i]->translationSpeed + pWheel[i]->angularSpeed;
    pWheel[i]->rpm = VelocityToRPM(pWheel[i]->Speed); 
  }
  ScaleWheels(maxMotRPM);
  prevVt = Vt;
  
}

float controlSpeedDist(int initialSpeed, int finalSpeed, int dx, int acceleration, bool whichAxis)   //cm/sec
{
  int sped = initialSpeed;
  long int currentX;
  if(finalSpeed==0)
  {
    return 0;
  }
  if(whichAxis == X_axis)
    currentX = pBot->X_pos;
  else 
    currentX = pBot->Y_pos;
  
  if(controlFlag)
  {
    if(abs(currentX - controlX) >= dx)
    {
      controlX = currentX;
      sped += ((acceleration+0.0)/(sped+0.0))*dx;
    }
  }
  if((acceleration > 0.0 && sped >= finalSpeed) || (acceleration < 0.0 && sped <= finalSpeed) || (acceleration == 0.0) || !controlFlag)
  {
    sped = finalSpeed;
    controlFlag = 0; 
    controlX = currentX;
  }
  return sped;
}

void startAccDist(int dx, int acceleration, int sppeed, bool whichAxis)
{
  speeed = sppeed;
  delta_xx = dx;
  accc = acceleration;
  controlFlag = 1;
  if(whichAxis == X_axis)
    controlX = pBot->X_pos;
  else
    controlX = pBot->Y_pos;
}

float OmegaControlMPU(int yaw,float tolerence)
{
  float omegacontrol=0;
  float error = ppidOmega->required - yaw;

  if(abs(error) < tolerence)
    {
      omegacontrol = 0;
      whiteLEDon;
    }
  else
    {
      omegacontrol = ppidOmega->pidControl(yaw);
      whiteLEDoff;
    }
  
  return omegacontrol;
}

//___________________________________ LSA _________________________________________//
float LineControl(float Lineerror,int thetalimit,float PIDmaxbound,PID *pLinegain){
  float Linecontrol;
  if(abs(Lineerror) < PIDmaxbound){
    Linecontrol = pLinegain->pidControl(Lineerror);
  }
  else{
      if(pLinegain->prevError==0)
          Linecontrol = 0;
      else
          Linecontrol = (float)(pLinegain->prevError)*thetalimit/abs(pLinegain->prevError);
  }
  return Linecontrol;
}  

float OmegaControl(float LSAforward , float LSAbackward,int limit,PID *pgain)
{
  float Lineerror = 0;
  float omegacontrol = 0;  
  
  if(abs(LSAforward) < 36)
  {
    //do nothing
  }
  else
  {
    if(LSAforwardprev == 0)
      LSAforward = 0;
    else
      LSAforward = (float)(LSAforwardprev)*limit/abs(LSAforwardprev);
  }

  if(abs(LSAbackward) < 36)
  {
    //do nothing
  }
  else
  {
    if(LSAbackwardprev == 0)
        LSAbackward = 0;
    else
        LSAbackward = (float)(LSAbackwardprev)*limit/abs(LSAbackwardprev);
  }
  
  Lineerror = LSAforward + LSAbackward;
  if(abs(Lineerror)<3)
  {
    omegacontrol=0;
  }
  else if(abs(Lineerror) < 81)
  {
    omegacontrol = pgain->pidControl(Lineerror);
  }
  LSAbackwardprev = LSAbackward;
  LSAforwardprev = LSAforward;

  return omegacontrol;
}
