#include <SPI.h>
#include <PS2X_lib.h>

void scalePS2value(){
      
      ySquare_leftdistance=(basePS2_Y-data.Ly)/basePS2_Y;
      xSquare_leftdistance=(data.Lx-basePS2_X)/basePS2_X;
      ySquare_rightdistance=(basePS2_Y-data.Ry)/basePS2_Y;
      xSquare_rightdistance=(data.Rx-basePS2_X)/basePS2_X;

      
      if(xSquare_leftdistance==0 && ySquare_leftdistance==0){
      xCircle_leftdistance=0;
      yCircle_leftdistance=0; 
      }
      else{
      xCircle_leftdistance= xSquare_leftdistance *sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2)-(pow(xSquare_leftdistance,2)*pow(ySquare_leftdistance,2)))/sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2));
      yCircle_leftdistance= ySquare_leftdistance *sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2)-(pow(xSquare_leftdistance,2)*pow(ySquare_leftdistance,2)))/sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2));
      }
      if(xSquare_rightdistance==0 && ySquare_rightdistance==0){
      xCircle_rightdistance=0;
      yCircle_rightdistance=0; 
      }
      else{
      xCircle_rightdistance= xSquare_rightdistance *sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2)-(pow(xSquare_rightdistance,2)*pow(ySquare_rightdistance,2)))/sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2));
      yCircle_rightdistance= ySquare_rightdistance *sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2)-(pow(xSquare_rightdistance,2)*pow(ySquare_rightdistance,2)))/sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2));
      }
      
      LeftAnalogDistance=sqrt((xCircle_leftdistance)*(xCircle_leftdistance)+(yCircle_leftdistance)*(yCircle_leftdistance));
      LeftAnalogTheta=atan2(yCircle_leftdistance,xCircle_leftdistance)*(180.0/3.14);
      LeftAnalogTheta=(int(LeftAnalogTheta)+360)%360;
      RightAnalogDistance=sqrt((xCircle_rightdistance)*(xCircle_rightdistance)+(yCircle_rightdistance)*(yCircle_rightdistance));
      RightAnalogTheta=atan2(yCircle_rightdistance,xCircle_rightdistance)*(180.0/3.14);
      RightAnalogTheta=(int(RightAnalogTheta)+360)%360;

      if(LeftAnalogDistance==0 || LeftAnalogDistance==360){
        LeftAnalogTheta=0;
      }
      if(RightAnalogDistance==0 || RightAnalogDistance==360){
        RightAnalogTheta=0;
      }
      
} 

void getPS2value(){
  if(type==2)
    ps2x.read_gamepad();
  else
     ps2x.read_gamepad(false, vibrate);   
  //ps2x.last_buttons=ps2x.buttons;
    delay(10);
  data.ps2data=ps2x.buttons;
  data.Rx=ps2x.PS2data[5];
  data.Ry=ps2x.PS2data[6];
  data.Lx=ps2x.PS2data[7];
  data.Ly=ps2x.PS2data[8];
  Serial.println(String(data.Ly)+"    "+String(data.Lx)+"   "+String(data.Ry)+"    "+String(data.Rx)+"    "+String(data.ps2data));      // tocheck base values
  }


void initps2(){
  
  error = ps2x.config_gamepad(PS2_CLK,PS2_CMD,PS2_SEL,PS2_DAT,true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error-----data~MISO
 
 if(error == 0){
   Serial.println("Found Controller, configured successful");
   Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
  Serial.println("holding L1 or R1 will print out the analog stick values.");
  Serial.println("Go to www.billporter.info for updates and to report bugs.");
 }
   
  else if(error == 1)
   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(error == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
   
   //Serial.print(ps2x.Analog(1), HEX);
   
   type = ps2x.readType(); 
     switch(type) {
       case 0:
        Serial.println("Unknown Controller type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
       break;
       case 2:
         Serial.println("GuitarHero Controller Found");
       break;
     }  
}
void PS2executePressed()
{    
    ps2release_count++;
    if(ps2release_count >3)         
     { 
        if(ps2x.ButtonPressed(PSB_PAD_RIGHT))  
        {
          Serial.println("PSB_PAD_RIGHT");
        }   
        if(ps2x.ButtonPressed(PSB_PAD_LEFT))  
        {
          Serial.println("PSB_PAD_LEFT");
        }
        if(ps2x.ButtonPressed(PSB_PAD_DOWN)) 
        {
          Serial.println("PSB_PAD_DOWN");
        }
        if(ps2x.ButtonPressed(PSB_PAD_UP)) 
        {
          Serial.println("PSB_PAD_UP");
        }
        if(ps2x.ButtonPressed(PSB_SELECT))
        { 
          Serial.println("PSB_SELECT"); 
        }
        if(ps2x.ButtonPressed(PSB_START))
        {  
          Serial.println("PSB_START");
        }
        if(ps2x.ButtonPressed(PSB_TRIANGLE))
        {   
          Serial.println("PSB_TRIANGLE");
        }  
        if(ps2x.ButtonPressed(PSB_CIRCLE))
        {  
          Serial.println("PSB_CIRCLE");  
        }
         
        if(ps2x.ButtonPressed(PSB_CROSS))
        {
          Serial.println("PSB_CROSS");
        }
         
        if(ps2x.ButtonPressed(PSB_SQUARE))
        {
          Serial.println("PSB_SQUARE");
        }
       
        if(ps2x.ButtonPressed(PSB_L1))
        { 
           Serial.println("PSB_L1");
        }
       
        if(ps2x.ButtonPressed(PSB_L2))
        { 
            Serial.println("PSB_L2");
        }

        if(ps2x.ButtonPressed(PSB_R1))
        {
            Serial.println("PSB_R1");
        }

        if(ps2x.ButtonPressed(PSB_R2))
        {
            Serial.println("PSB_R2");
        } 
     delay(50);   
     }
}
