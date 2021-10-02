#include <Wire.h>
#include <VL6180X.h>

#define TCAADDR 0x70
#define RIOADDR 0x2c /*DUNNO RIGHT NOW, MAKE SURE TO CHANGE...*/

//VL6180X s0, s1, s2, s3; //to lazy to have array...

VL6180X s[4];
int d[4] = {0,0,0,0};


//setup
void tcaselect(uint8_t addr){
	if(addr > 7) return; 
	
	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << addr);
	
	Wire.endTransmission();
}

//init sensors in array format
void sensors_up_debug(){
	Wire.begin();
	

//	for(int i=0; i<4; i++){

		int i = 1; //debug

		tcaselect(i); s[i].init(); s[i].configureDefault();
		
		s[i].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
		s[i].writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
		
		s[i].setTimeout(500); s[i].stopContinuous();
		s[i].startInterleavedContinuous(100);

//	}	
	
  	//is_working(s[3]);

	
	delay(300);

}


//to the roborio
void send(){
	Wire.beginTransmission(RIOADDR);

	//Sync send...
	for(int i=0; i<4; i++){
		Wire.write(d[i]);
	}
	
	Wire.endTransmission();
	
}


void setup(){
	Serial.begin(9600);
	
	VL6180X model_s;

	for(int i=0; i<4; i++){
		s[i] = model_s;
	}
	
	sensors_up_debug();
}

void loop(){
//	for(int j=0; j<4; j++){
		int j = 1; //debug
		tcaselect(j); d[j] = s[j].readRangeContinuousMillimeters(); 

//	}
	
	
	
	//debugging purposes...
	Serial.println((String)"d0: "+d[0]+" d1: "+d[1]+" d2: "+d[2]+" d3: "+d[3]);
	//send();
}
