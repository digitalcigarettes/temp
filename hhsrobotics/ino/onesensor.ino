//onesensor.ino
#include <Wire.h>
#include <VL6180X.h>

#define TCAADDR 0x70


VL6180X s;
int d;

//setup
void tcaselect(uint8_t addr){
	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << addr);
	
	Wire.endTransmission();
}



void setup(){
	Serial.begin(9600);
	
	Wire.begin();

	tcaselect(1); 
	s.init(); 
	s.configureDefault();
	
	s.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
	s.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
	
	s.setTimeout(500); 
	s.stopContinuous();
	s.startInterleavedContinuous(100);
	
	delay(300);
}

void loop(){
		tcaselect(1); 
		d = s.readRangeContinuousMillimeters();
		Serial.println(d);
}
