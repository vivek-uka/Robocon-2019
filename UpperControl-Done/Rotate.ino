bool alignRotate(float required)
{
  if(delayCounter==0)
  {
    ppidOmega->required = required;
    omegacontrol = OmegaControlMPU(robotYaw,2);
  }
  if(delayCounter <= 70)
  {
      calculateSpeed(-omegacontrol,0,0);  
      delayCounter ++;
      return 1;
  }
  delayCounter = 0;
  return 0;
}

