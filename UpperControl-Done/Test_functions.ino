void printXY_Yaw()
{   
//   SerialX.print("Yaw(degree) "+String(robotYaw));
SerialX.print((int)pBot->X_pos);
SerialX.print("   ");
SerialX.print((int)pBot->Y_pos);
SerialX.print("   ");
SerialX.println(millis()/1000);

}
void printCount()
{
  SerialX.print("      X (count):   "+String((int)pEncoderX->Count));
  SerialX.println("    Y (count):   "+String((int)pEncoderY->Count));
}

void printRtheta()
{
  SerialX.print("r (cm): "+String(sqrt(pBot->X_pos*pBot->X_pos + pBot->Y_pos*pBot->Y_pos)));
  SerialX.println("theta(degree) " + String(DegreeToRadian(atan2(pBot->Y_pos,pBot->X_pos))));
}
void printRPM()
{
  SerialX.println(String(pWheel[0]->rpm)+" "+String(pWheel[1]->rpm)+" "+String(pWheel[2]->rpm)+ " "+String(pWheel[3]->rpm));
}

void tuneLSA(int velocity)
{
  Linecontrol=LineControl(GetLSAReading(ActiveSensor),20,35,pLinegain);
  omegaLine=-OmegaControl(GetLSAReading(ActiveSensor),GetLSAReading(ActiveOmegaSensor),40,pOmegagain);
  calculateSpeed(omegaLine, DegreeToRadian((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360),velocity );
  Serial.println("LineControl:  " + String((int)((LSAArray[ActiveSensor]->Theta-Linecontrol)+360)%360) + "OmegaControl:  " + String(omegaLine));
}
