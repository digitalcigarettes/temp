#include <Wire.h>
#include <VL6180X.h>

#define TCAADDR 0x70
#define RIOADDR 0x2c /*DUNNO RIGHT NOW, MAKE SURE TO CHANGE...*/

int sensorcount  = 4;

VL6180X s1[4];
int d[4];


void tcaselect(uint8_t addr){
  if(addr > 7) return; 
  
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << addr);
  
  Wire.endTransmission();
}

//init sensors in array format
void _init(){
  Wire.begin();
  

  for(int i=0; i<sensorcount; i++){

    tcaselect(i); s[i].init(); s[i].configureDefault();
    
    s[i].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    s[i].writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    
    s[i].setTimeout(150); s[i].stopContinuous();

    delay(100); //time to do its stuff

    s[i].startInterleavedContinuous(100);
    
    Serial.println((String)"Initialized Sensor: "+i+";"); // #debug
  } 
  
    //is_working();

  
  delay(200);

}


//to the roborio
void send(){

  Wire.beginTransmission(RIOADDR);

  for(int i=0; i<sensorcount; i++) Wire.write(d[i]);
  
  Wire.endTransmission();
  
}


void setup(){
  Serial.begin(9600);
  
  VL6180X model_s;

  for(int i=0; i<sensorcount; i++){
    s[i] = model_s;
    d[i] = 0;
  }
  
  _init();
}

void loop(){

  for(int j=0; j<sensorcount; j++){
    tcaselect(j); d[j] = s[j].readRangeContinuousMillimeters(); 
    
    Serial.println((String)"Sensor: "+j+"; Value: "+d[j]+";"); //#debug
  }
  
  
  send();
}
