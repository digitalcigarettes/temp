#include <Wire.h>
#include <VL6180X.h>

#define TCAADDR 0x70

VL6180X s0, s1, s2, s3;
int d0=0, d1=0, d2=0, d3=0;

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

void tcaselect(uint8_t addr){
	if(i>7) return; 
	
	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << addr);
	Wire.endTransmission();
}

void send(){
	//Do later i'm tired
}

void setup(){
	Serial.begin(9600);
	sensors_up()
}

void loop(){
	tcaselect(0); d0 = s0.readRangeContinuousMillimeters(); 
	tcaselect(1); d1 = s1.readRangeContinuousMillimeters(); 
	tcaselect(2); d2 = s2.readRangeContinuousMillimeters(); 
	tcaselect(3); d3 = s3.readRangeContinuousMillimeters();
	//debugging purposes...
	Serial.println((String)"d0:"+d0+"' d1:"+d1+"; d2:"+d2+" d3:"+d3");

}
