uint16_t cntr11 = 0;
void TransmitURPM(){
  for(int y=0;y<4;y++)
  {
    
    if(pWheel[y]->rpm > maxMotRPM ) 
       pWheel[y]->rpm = maxMotRPM;
    if(pWheel[y]->rpm < -maxMotRPM ) 
       pWheel[y]->rpm = -maxMotRPM;
    int temp = ((int)pWheel[y]->rpm);
    mydata.rpm[y]=temp;    
  }
  ET.sendData();    
}



