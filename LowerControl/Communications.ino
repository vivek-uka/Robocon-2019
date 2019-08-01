float BotX,BotY;
uint8_t localCntr = 0, cntr=0,flag=0;
void getUpperData(){
   if(ET.receiveData()>0)
   {
      if(cntr<=10)
      digitalWrite(51,flag);
      else
      {
        flag = !flag;
        cntr = 0;
      }
      localCntr = 0;
     
      for(int i=0;i<N;++i)
        pPIDMotor[i]->prevRequired = pPIDMotor[i]->required;
        
      for(int i=0;i<N;++i)
        {
          pPIDMotor[i]->required = mydata.rpm[i];
          if(mydata.rpm[i] == 0)
          localCntr ++;
        }
        softBrake = 1;
        if(localCntr == 4)
        { 
          softBrake = 0;
        }
        cntr++;
//    SerialX.println(String(pPIDMotor[0]->required)+" "+String(pPIDMotor[1]->required)+" "+String(pPIDMotor[2]->required)+" "+String(pPIDMotor[3]->required));
   }
}


