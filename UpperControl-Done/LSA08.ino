void LSA08 :: initLSA(int baud,int OutputEnable){
  pinMode(OutputEnable,OUTPUT);
  digitalWrite(OutputEnable,HIGH);
  for(int i=0;i<4;i++){
    LSAArray[i]->junction_detect[i]=0;  
  }
}


void LSA08 :: sendCommand(char command, char data, char address){
  char checksum = address + command + data;  
  SerialLSA.write(address);
  SerialLSA.write(command);
  SerialLSA.write(data);
  SerialLSA.write(checksum);
}

void LSA08 :: ChangeBaud(char baud, char add){
  char command='R';
  char data;

  if(baud==9600) data=0;
  else if(baud==19200) data=1;
       else if(baud==38400) data=2;
            else if(baud==57600) data=3;
                 else if(baud==115200) data=4;
                      else if(baud==230400) data=5;
   this->sendCommand(command,data,add);
}

void LSA08 :: clearJunction(char add) {
  char address = add;
  char command = 'X';
  char data = 0x00;
  this->sendCommand(command,data,address);
}

int LSA08 :: getJunction(char add){
  char address = add;
  char command = 'X';
  char data = 0x01;
  this->sendCommand(command,data,address);

  while(SerialLSA.available() <= 0);
  return (int(SerialLSA.read()));
} 
int LSA08 :: GetByteOfLSA(int OutputEnable){                                            //Initially each and every serialLSAEnX(X=1,2,3) should be HIGH
  int a = 0;
  digitalWrite(OutputEnable,LOW);
  while(SerialLSA.available()<=0);
  a = SerialLSA.read();
  digitalWrite(OutputEnable,HIGH);
  lsa = false;
  return a;   
}
 
float GetLSAReading(uint8_t sensor)
{
  float reading = LSAArray[sensor]->GetByteOfLSA(LSAArray[sensor]->OePin);
  reading = 35 - reading;
  return reading;  
}

void initLSA(void){

  SerialLSA.begin(9600);
  
  LSAArray[0]->initLSA(9600,LSAArray[0]->OePin);            //const int minControl = -255;      const int maxControl = 255;
  LSAArray[1]->initLSA(9600,LSAArray[1]->OePin);
  LSAArray[2]->initLSA(9600,LSAArray[2]->OePin);
  LSAArray[3]->initLSA(9600,LSAArray[3]->OePin);

  pinMode(LSAArray[0]->JunctionPin,INPUT);
  pinMode(LSAArray[1]->JunctionPin,INPUT);
  pinMode(LSAArray[2]->JunctionPin,INPUT);
  pinMode(LSAArray[3]->JunctionPin,INPUT);

  digitalWrite(LSAArray[0]->JunctionPin,LOW);
  digitalWrite(LSAArray[1]->JunctionPin,LOW);
  digitalWrite(LSAArray[2]->JunctionPin,LOW);
  digitalWrite(LSAArray[3]->JunctionPin,LOW);
  
  LSAArray[0]->clearJunction(LSAArray[0]->Address);
  LSAArray[1]->clearJunction(LSAArray[1]->Address);
  LSAArray[2]->clearJunction(LSAArray[2]->Address);
  LSAArray[3]->clearJunction(LSAArray[3]->Address);  
}

void LSA08 :: Calibrate(int Sensor)
{
  Serial.println("Calibrating");
  delay(2);
  LSAArray[Sensor]->sendCommand(0x43, 0x05, LSAArray[Sensor]->Address);
}

void calibrateBothLSA(void)
{
  LSAArray[ActiveSensor]->sendCommand(0x43, 0x05, LSAArray[ActiveSensor]->Address);
  delay(100);
  LSAArray[ActiveOmegaSensor]->sendCommand(0x43, 0x05, LSAArray[ActiveOmegaSensor]->Address);
  delay(100);
}

