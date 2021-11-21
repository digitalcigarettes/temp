#include <Wire.h>
#include <VL6180X.h>

#define TCAADDR 0x70
#define RIOADDR 0x2c /*DUNNO RIGHT NOW, MAKE SURE TO CHANGE...*/

//VL6180X s0, s1, s2, s3; //to lazy to have array...

VL6180X s[4];
int d[4] = {0,0,0,0};


//setup
void tcaselect(uint8_t addr){
	if(i>7) return; 
	
	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << addr);
	Wire.endTransmission();
}

void is_working(VL6180X sr){
	if(sr.init() != 0x60){
		Serial.print(F("VL6180X detected?\t")); Serial.println(F("No")); 
		return;
	}
	
	Serial.print(F("VL6180X detected?\t")); Serial.println(F("Yes")); 
}

//init sensors in array format
void sensors_up_debug(){
	Wire.begin();
	
	//Sensor 0
	tcaselect(0); s[0].init(); s[0].configureDefault();
	
	s[0].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  	s[0].writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
	
	s[0].setTimeout(500); s[0].stopContinuous();
	s[0].startInterleavedContinuous(100);
	
	is_working(s[0]);
	
	//Sensor II
	tcaselect(1); s[1].init(); s[1].configureDefault(); 
	
	s[1].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  	s[1].writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
	
	s[1].setTimeout(500); s[1].stopContinuous();
	s[1].startInterleavedContinuous(100);

	is_working(s[1]);

	//Sensor II
	tcaselect(2); s[2].init(); s[2].configureDefault(); 
	
 	s[2].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  	s[2].writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
		
	s[2].setTimeout(500); s[2].stopContinuous();
	s[2].startInterleavedContinuous(100);
	
	is_working(s[2]);

	//Sensor III
    	tcaselect(3); s[3].init(); s[3].configureDefault();
	
  	s[3].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  	s[3].writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
	
	s[3].setTimeout(500); s[3].stopContinuous();
	s[3].startInterleavedContinuous(100);
	
  	is_working(s[3]);

	
	delay(300);

}

/*
void sensors_up(){
	Wire.begin();
	s0.init(); s1.init(); s2.init(); s3.init();
	s0.configureDefault(); s1.configureDefault(); s2.configureDefault(); s3.configureDefault();
	
	
	s0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
	s1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
 	s2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  	s3.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  
  	s0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  	s1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  	s2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  	s3.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

	s1.setTimeout(500); s2.setTimeout(500); s3.setTimeout(500); s4.setTimeout(500);
	s1.stopContinuous(); s2.stopContinuous(); s3.stopContinuous(); s4.stopContinuous();

	delay(300);

	s0.startInterleavedContinuous(100); s1.startInterleavedContinuous(100); s2.startInterleavedContinuous(100); s3.startInterleavedContinuous(100);

}

*/

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
	tcaselect(0); d[0] = s[0].readRangeContinuousMillimeters(); 
	tcaselect(1); d[1] = s[1].readRangeContinuousMillimeters(); 
	tcaselect(2); d[2] = s[2].readRangeContinuousMillimeters(); 
	tcaselect(3); d[3] = s[3].readRangeContinuousMillimeters();
	
	
	
	//debugging purposes...
	//Serial.println((String)"d0:"+d[0]+"' d1:"+d[1]+"; d2:"+d[2]+" d3:"+d[3]");
	send();
}
