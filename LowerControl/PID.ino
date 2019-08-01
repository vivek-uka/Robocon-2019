void PID::initPID(float kp,float kd,float ki,float req,float minV,float maxV)
{
  Kp=kp;
  Kd=kd;
  Ki=ki;
  required=req;
  maxControl=maxV;
  minControl=minV;
  error=0;
  prevError=1;
  derivativeError=0;
  integralError=0;
  prev_integralError=0;
  prevRequired=req;
}

float PID::pidControl(float actual)
{
  error = fabs(required) - fabs(actual);
  derivativeError = error - prevError;
  prevError = error;
  
  float Output = Kp*error + Kd*derivativeError + Ki*integralError;
  
  if(Output > maxControl)
    Output = maxControl;
  else if(Output < minControl)
    Output = minControl;
  else
    integralError = integralError + error;

  if(Ki != 0.0)
  {
    if((fabs(derivativeError) <= 0.02*fabs(error)) && (fabs(error) >= 0.05*fabs(required)))
    {
      iCounter++;
      if(iCounter > 1000/EncoderTime)   // 1 seconds
        {
          prev_integralError = 0;
          iCounter = 0;
        }
      integralError = prev_integralError;
    }
    prev_integralError = integralError;
  }

  return Output;
}

