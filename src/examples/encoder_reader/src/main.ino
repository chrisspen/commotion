#include <Wire.h>

#define COMMOTION_ADDR1 30
#define COMMOTION_ADDR2 31

//http://arduino.stackexchange.com/a/3388/4478
uint16_t receive_wire_int(){
	
    // Read low byte into rxnum
    uint16_t rxnum = Wire.read();

    // Read high byte into rxnum
    rxnum += Wire.read() << 8;

    return rxnum;
}

void setup() {
	Wire.begin();        // join i2c bus (address optional for master)
	Serial.begin(9600);  // start serial for output
}

void loop() {
	Serial.println("Requesting data..."); Serial.flush();
	
	//byte c = Wire.read(); // receive a byte as character

	Wire.requestFrom(COMMOTION_ADDR1, 4);
	while(Wire.available() < 4){
		Serial.println("Waiting..."); Serial.flush();
	}
	//Wire.requestFrom(COMMOTION_ADDR2, 4);
	
	uint16_t acount = receive_wire_int();
	uint16_t bcount = receive_wire_int();

	//Serial.println(String("acount:")+String(acount));
	Serial.println(String("acount:")+String(acount)+String(" bcount:")+String(bcount));

	delay(500);
}
