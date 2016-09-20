
#include <EEPROM.h>
#include <Wire.h>
#include "IOpins.h"
#include "Notes.h"

byte mcu;                                                                     // designation as MCU 1 or MCU 2 as determined by state of pin A2
byte address;                                                                 // I²C address based on dipswitches and ID pin

//============================================================================== Motor Control Variables ================================================================================

int acount,bcount;                                                            // encoder pulse counters used to measure distance
volatile byte aflag;                                                          // flag to indicate encoder A has changed state                                                                
volatile byte bflag;                                                          // flag to indicate encoder B has changed state
volatile unsigned long apulse,bpulse;                                         // width of encoder pulses in uS
volatile unsigned long atime,btime;                                           // stores time of last encoder state change
byte motora,motorb;                                                           // values will be 0&1 on MCU1 and 2&3 on MCU2     - precalculated to increase speed
long maxpulse[4];                                                             // max time between encoder state changes in uS   - precalculated to increase speed

//============================================================================== Configuration Data======================================================================================

byte demo;                                                                    // demo mode                   modes : 0 = OFF,  1 = Line,  2 = Object
byte mode=0;                                                                  // default to standby          modes : 0 = standby, 1 = I²C command, 2 = Serial command, 3 = Scamper Demo
byte configuration=0;                                                         // default to Omni 3 wheel     config: 0 = Omni 3 wheel, 1 = Omni 4 wheel, 2 = Mecanum, 3 = Individual
byte lowbat=60;                                                               // low battery voltage (60 = 6.0V)
int maxamps[4];                                                               // used to set maximum current allowed (255 = 2.55A) for each motor
byte master=1;                                                                // external master address (default=1)
byte addroffset=0;                                                            // optional I²C address offset (default=0)
byte defaulted;                                                               // used to determine if defaults need to be loaded
byte encoders;                                                                // flag to indicate if encoder feedback is enabled or disabled
int motrpm[4];                                                                // motor rpm used to determine max speed
int encres[4];                                                                // encoder resolution (x100) used to determine max speed
byte reserve[4];                                                              // reserve power to ensure accurate speed under changing load
long stalltm[4];                                                              // stall time in mS used to determine if a motor is running slow or stalled
byte sermode;                                                                 // serial mode determines where commands come from and data goes to
unsigned int baudrate[2];                                                     // baud rate for each serial port


//============================================================================== Shield Control Variables ===============================================================================

int velocity=0;                                                               // requested speed
int angle=-360;                                                               // requested angle of travel
int rotation=0;                                                               // requested rotation
int mspeed[4];                                                                // requested speed of individual motors

float radconvert=PI/180;                                                      // used to convert radians into degrees (precalculated to improve speed)

byte analogpin[5]={0,1,3,6,7};                                                // analog inputs to scan: A0=current A, A1=current B, A3 & A6=spare, A7=voltage on MCU1 or spare / sensor on MCU2
int  analogvalue[5];                                                          // store analog input results
byte datapack[32];                                                            // command/config data packet
byte sendpack[32];                                                            // data send packet for returning status data
byte serpack[36];                                                             // serial data pack
byte syncpack=1;                                                              // sync pack type (default: 1=controller config) 
byte packsize;                                                                // size of command packet    
byte command=255;                                                             // 255 = no command 
byte analog;                                                                  // read different analog input each loop (analog conversion takes 260uS)
byte powerdown=0;                                                             // a value of 1 shuts down all motors to conserve power and prevent a brownout condition

int  voltage;                                                                 // battery voltage
byte eflag=0;                                                                 // error flag records faults such as over current and under voltage
byte i2cfreq=1;                                                               // default value for I²C

//============================================================================== Demo global variables ==================================================================================

unsigned long time=millis();
unsigned long IRtime=millis();

//unsigned long loop_count = 0
byte loop_count = 0;

//============================================================================ Motor Speed Control ====================================================================================

void Motors()
{


  unsigned long actual;                                                       // temporary calculation of desired speed in uS per encoder pulse
  static byte apwm,bpwm;                                                      // A and B motor speeds
  static byte astall,bstall;                                                  // flags to indicate a stalled motor
  

  if(aflag==1 || astall==1)                                                   // if encoder A has changed states or motor A has stalled                  
  {
    apulse=(micros()-atime);                                                  // time between last state change and this state change
    atime=micros();                                                           // update atime with time of most recent state change
        
    if(aflag) acount=acount+(mspeed[motora]>0)-(mspeed[motora]<0);            // update encoder A state change counter
    actual=maxpulse[motora]/abs(mspeed[motora]);                              // calculate desired time in uS between encoder pulses
    if(actual>apulse && apwm>0) apwm--;                                       // if motor is running too fast then decrease PWM
    if(actual<apulse && apwm<255) apwm++;                                     // if motor is running too slow then increase PWM
    if(mspeed[motora]==0) apwm=0;                                             // override speed adjust if speed is 0 for quick stop
    analogWrite(pwmapin,apwm);                                                // update motor speed
    digitalWrite(dirapin,mspeed[motora]>0);                                   // set direction of motor
    astall=0;                                                                 // reset stall flag
    aflag=0;                                                                  // reset encoder flag
  }

  if(bflag==1 || bstall==1)                                                   // if encoder B has changed states or motor B has stalled  
  {  
    bpulse=(micros()-btime);                                                  // time between last state change and this state change
    btime=micros();                                                           // update btime with time of most recent state change
        
    if(bflag) bcount=bcount+(mspeed[motorb]>0)-(mspeed[motorb]<0);            // update encoder B state change counter
    actual=maxpulse[motorb]/abs(mspeed[motorb]);                              // calculate desired time in uS between encoder pulses
    if(actual>bpulse && bpwm>0) bpwm--;                                       // if motor is running too fast then decrease PWM
    if(actual<bpulse && bpwm<255) bpwm++;                                     // if motor is running too slow then increase PWM
    if(mspeed[motorb]==0) bpwm=0;                                             // override speed adjust if speed is 0 for quick stop
    analogWrite(pwmbpin,bpwm);                                                // update motor speed
    digitalWrite(dirbpin,mspeed[motorb]>0);                                   // set direction of motor
    bstall=0;                                                                 // reset stall flag
    bflag=0;                                                                  // reset encoder flag
  }  
  
  

  //============================================================================ Check for stalled motors ===============================================================================

  if(analogvalue[0]>maxamps[motora])                                          // motor A maximum current exceeded
  {
    apwm=apwm/2;                                                              // halve pwm
    if(mcu==0) eflag=eflag|B00000001;                                         // bit 0 indicates M1 has exceeded current limit
    if(mcu==1) eflag=eflag|B00000100;                                         // bit 2 indicates M3 has exceeded current limit
  }

  if(mspeed[motora]==0)                                                       // if motor A speed is supposed to be 0 or motor current exceeds limit
  {
    apwm=0;                                                                   // ensure apwm=0
    analogWrite(pwmapin,apwm);                                                // cut power to motor A
  }
  else
  {
    if(micros()-atime>(stalltm[motora]*1000L))                                // if encoder A has not changed states within 10mS
    {
      astall=1;                                                               // set motor A stall flag
      apwm+=2;                                                                // jump start apwm value
      if(apwm>253) apwm=253;
    }
  }

  if(analogvalue[1]>maxamps[motorb])                                          // motor B maximum current exceeded
  {
    bpwm=bpwm/2;                                                              // halve pwm
    if(mcu==0) eflag=eflag|B00000010;                                         // bit 1 indicates M2 has exceeded current limit
    if(mcu==1) eflag=eflag|B00001000;                                         // bit 3 indicates M4 has exceeded current limit
  }

  if(mspeed[motorb]==0)                                                       // if motor B speed is supposed to be 0 or motor current exceeds limit
  {
    bpwm=0;                                                                   // ensure bpwm=0
    analogWrite(pwmbpin,bpwm);                                                // cut power to motor B
  }
  else
  {
    if(micros()-btime>(stalltm[motorb]*1000L))                                // if encoder B has not changed states within 10mS 
    {
      bstall=1;                                                               // set motor B stall flag
      bpwm+=2;                                                                // jump start bpwm value
      if(bpwm>253) bpwm=253;
    }
  }
}


//============================================================================== Encoder ISRs =================================================================

void Aencoder()                                                               // left  encoder Interrupt Service Routine
{
  aflag=1;                                                                    // set flag for left  encoder
}

void Bencoder()                                                               // right encoder Interrupt Service Routine
{
  bflag=1;                                                                    // set flag for right encoder
}

//========================================================================================  =======================================


void LineFollow()
{
  static byte edgedetect;
  static int sensor;
  
  velocity=40*(angle>0);
  rotation=120;
  if(mcu==1)                                         // demo controlled via MCU2 which has sensor connected to A7
  {
    if(analog==0)
    {
      sensor=analogRead(senspin);                    // read current sensor value
    }
    
    if(analog==1)
    {
      if((sensor>900 && (edgedetect&1)==0) || (sensor<800 && (edgedetect&1)==1)) edgedetect++;
      if(edgedetect>3) 
      {
        time=millis();
        edgedetect=0;
        angle+=60;
        if(angle>-300) angle=0;
      }
    }
    
    if(angle>-300 && analog==3)
    {
      
      angle=int((millis()-time)*360L/2300L);
      Wire.beginTransmission(address-1);
      Wire.write(7);
      Wire.write(highByte(angle));
      Wire.write(lowByte(angle));
      Wire.endTransmission();
    }
  }
  Trigonometry();
}
    


//========================================================================================  =======================================

void PowerDown()
{
  analogWrite(pwmapin,0);                                                                       // power down M1
  analogWrite(pwmbpin,0);                                                                       // power down M2
  eflag=eflag|B00100000;                                                                        // error flag bit 6 indicates power shut down due to low battery voltage
  
  Wire.beginTransmission(address+1);                                                             // take control of I²C bus and address MCU2
  Wire.write(3);                                                                                 // datapacket #3 - send power down command to M3 & M4
  for(byte i=0;i<8;i++)                                                                       
  {
    Wire.write(0);                                                                                // set all motor speeds to 0
  }
  Wire.endTransmission();                                                                         // transmission ok - release control of I²C bus
}

//======================================================================================== Takes data from serial port and redirects it as determined by serial mode =======================================

void SerialInput()                                                        
{
  if((mcu==0 && (sermode==1 || sermode==3)) || (mcu==1 && (sermode==2 || sermode==4)))  // Command received on serial port
  {
    byte i=0;
    command=255;
    while(Serial.available()>0)
    {
      datapack[i]=Serial.read();                                                        // data is read from the serial buffer and added to the command datapack
      i++;
    }
    
    //------------------------------------------------------------------------------------ Verify that command data has been received ---------------------------------------------------------------------
    if(datapack[0]==0 && i==10)
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==1 && (i==7 || i==25))
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==2 && (i==6 || i==8))
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==3 && i==6)
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==4 && i>2)
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(command==255)
    {
      Serial.println("Command not recognized!");
      Serial.print("Command:");Serial.print(datapack[0],DEC);
      Serial.print("Pack Size:");Serial.println(i,DEC);
      Serial.println("");
    }
    else
    {
      if(mode==1)
      {
        mode=0;                                                                         // disable demo mode
        EEPROM.write(1,mode);
      }
    }
  }
  else
  {
    byte j=4;                                                                           // byte 1-4 is serial pack header
    serpack[0]=83;                                                                      // this header is used to indicate
    serpack[1]=80;                                                                      // the start of a new data packet
    serpack[2]=mcu+49;                                                                  // and where the data came from
    serpack[3]=58;                                                                      // "SP1:" or "SP2:" is serial port 1 or 2
    while(Serial.available()>0)
    {
      serpack[j]=Serial.read();                                                         // data is read from the serial buffer and added to the datapack
      j++;
    }
    
    if(sermode==0 || (sermode==1 && mcu==1) || (sermode==2 && mcu==0))                  // send serial data to I²C master
    {
      Wire.beginTransmission(master);                                                   // address I²C master
      Wire.write(serpack,j);                                                            // send data with 4 byte header
      Wire.endTransmission();                                                           // release I²C bus
    }
    else if((configuration==3 && mcu==1) || (configuration==4 && mcu==0))               // pass serial data to other serial port
    {
      byte pass=master;
      if(mcu==0) pass=address+1;                                                        // address of MCU to pass data to
      if(mcu==1) pass=address-1;
      Wire.beginTransmission(pass);                                                     
      Wire.write(20);                                                                   // command to internal send serial data (4+16)
      Wire.write(j);                                                                    // serial pack size including header data
      Wire.write(serpack,j);                                                            // send serialpack
      Wire.endTransmission();
    }
  }
    
}

//==============================================================================  ==================================================================================

void TXconfig()
{
  if(mcu)
  {
    //Serial.begin(57600);
    Serial.println("");
    Serial.print("Demo:");Serial.println(demo,DEC);
    Serial.print("Mode:");Serial.println(mode,DEC);
    Serial.print("Configuration:");Serial.println(configuration,DEC);
    Serial.println("");
    Serial.print("Lo Bat:");Serial.print(lowbat/10.0);Serial.println("V");
    Serial.print("M1 maximum current:");Serial.print(maxamps[0]/100.0);Serial.println("A");
    Serial.print("M2 maximum current:");Serial.print(maxamps[1]/100.0);Serial.println("A");
    Serial.print("M3 maximum current:");Serial.print(maxamps[2]/100.0);Serial.println("A");
    Serial.print("M4 maximum current:");Serial.print(maxamps[3]/100.0);Serial.println("A");
    Serial.println("");
    Serial.print("Master I2C address:");Serial.println(master,DEC);
    Serial.print("I2C address offset:");Serial.println(addroffset,DEC);
    Serial.print("I2C clock frequency:");Serial.print(i2cfreq*300+100);Serial.println("Kb/sec");
    Serial.println("");
    Serial.print("M1 maximum speed:");Serial.print(motrpm[0]);Serial.println("RPM");
    Serial.print("M2 maximum speed:");Serial.print(motrpm[1]); Serial.println("RPM");
    Serial.print("M3 maximum speed:");Serial.print(motrpm[2]);Serial.println("RPM");
    Serial.print("M4 maximum speed:");Serial.print(motrpm[3]);Serial.println("RPM");
    Serial.println("");
    Serial.print("M1 encoder resolution:");Serial.println(encres[0]/100.0);
    Serial.print("M2 encoder resolution:");Serial.println(encres[1]/100.0);
    Serial.print("M3 encoder resolution:");Serial.println(encres[2]/100.0);
    Serial.print("M4 encoder resolution:");Serial.println(encres[3]/100.0);
    Serial.println("");
    Serial.print("M1 maximum encoder pulse:");Serial.print(maxpulse[0]/255L);Serial.println("uS");
    Serial.print("M2 maximum encoder pulse:");Serial.print(maxpulse[1]/255L);Serial.println("uS");
    Serial.print("M3 maximum encoder pulse:");Serial.print(maxpulse[2]/255L);Serial.println("uS");
    Serial.print("M4 maximum encoder pulse:");Serial.print(maxpulse[3]/255L);Serial.println("uS");
    Serial.println("");
    Serial.print("M1 reserve power:");Serial.print(reserve[0],DEC);Serial.println("%");
    Serial.print("M2 reserve power:");Serial.print(reserve[1],DEC);Serial.println("%");
    Serial.print("M3 reserve power:");Serial.print(reserve[2],DEC);Serial.println("%");
    Serial.print("M4 reserve power:");Serial.print(reserve[3],DEC);Serial.println("%");
    Serial.println("");
    Serial.print("M1 stall time:");Serial.print(stalltm[0]);Serial.println("mS");
    Serial.print("M2 stall time:");Serial.print(stalltm[1]);Serial.println("mS");
    Serial.print("M3 stall time:");Serial.print(stalltm[2]);Serial.println("mS");
    Serial.print("M4 stall time:");Serial.print(stalltm[3]);Serial.println("mS");
    Serial.println("");
    Serial.print("Serial mode:");Serial.println(sermode,DEC);
    Serial.print("Baud rate port 1:");Serial.println(baudrate[0]);
    Serial.print("Baud rate port 2:");Serial.println(baudrate[1]);
    Serial.print("Defaults:");Serial.println(EEPROM.read(40),DEC);
    Serial.println("");
    Serial.println("");
  }
}

//==============================================================================  ==================================================================================

void Trigonometry()
{
  if(configuration==0)                                                                        // 3 Omni wheels
  {
    if(mcu==0)
    {
      mspeed[0]=int(sin((angle+120.0)*radconvert)*float(velocity))+rotation;                  // calculate desired speed
      mspeed[1]=int(sin((angle)*radconvert)*float(velocity))+rotation;                        // calculate desired speed
    }
    else
    {
      mspeed[2]=int(sin((angle-120.0)*radconvert)*float(velocity))+rotation;                  // calculate desired speed
      mspeed[3]=0;                                                                            // 4th motor not used
    }
    return;
  }
  
  if(configuration==1)                                                                        // 4 Omni wheels
  {
    if(mcu==0)
    {
      mspeed[0]=int(sin((angle)*radconvert)*velocity)+rotation;                               // calculate desired speed
      mspeed[1]=int(sin((angle+90)*radconvert)*velocity)+rotation;                            // calculate desired speed
    }
    else
    {
      mspeed[2]=int(sin((angle+180)*radconvert)*velocity)+rotation;                           // calculate desired speed
      mspeed[3]=int(sin((angle+270)*radconvert)*velocity)+rotation;                           // calculate desired speed
    }
    return;
  }
  
  if(configuration==2)                                                                        // Mecanum Wheels
  {
    if(mcu==0)
    {
      mspeed[0]=int((-sin(angle*radconvert)+cos(angle*radconvert))*float(velocity))+rotation; // calculate desired speed
      mspeed[1]=int((+sin(angle*radconvert)+cos(angle*radconvert))*float(velocity))+rotation; // calculate desired speed
    }
    else
    {
      mspeed[2]=int((-sin(angle*radconvert)+cos(angle*radconvert))*float(velocity))-rotation; // calculate desired speed
      mspeed[3]=int((+sin(angle*radconvert)+cos(angle*radconvert))*float(velocity))-rotation; // calculate desired speed
    }
    return;
  }
}


//==============================================================================  =======================================================

void Avoidance()
{
  static int sensor;

  velocity=50;
  rotation=120;
  if(mcu==1)                                         // demo controlled via MCU2 which has sensor connected to A7
  {
    if(analog==0)
    {
      sensor=analogRead(senspin);                    // read current sensor value
    }

    if(sensor<900) time=millis();

    if(analog==3)
    {
      angle=int((millis()-time)*360L/2300L)+155;
      Wire.beginTransmission(address-1);
      Wire.write(7);
      Wire.write(highByte(angle));
      Wire.write(lowByte(angle));
      Wire.endTransmission();
    }
  }
  Trigonometry();
}

//============================================================================== Generate 2kHz beeps =======================================================

void Beep(byte beeps)
{
  
  
  for(int b=0;b<beeps;b++)                                                    // loop to generate multiple beeps
  {
    PORTB=PORTB|B00001111;                                                    // set all motor direction and PWM pins high
    for(int duration=0;duration<600;duration++)                               // generate 2kHz tone for 200mS
    {
      delayMicroseconds(250);                                                 // wait 250uS to generate 2kHz tone
      PORTB=PORTB^B00001001;                                                  // toggle direction bits
    }
    PORTB=PORTB&B11110000;                                                    // turn off all motors
    delay(200);                                                               // pause for 200mS (1/5th of a second) between beeps
  }
  
}

//============================================================================== Plays Tune ===============================================================

void Tune()                                                                   // modified code from Brett Hagman's original Tone library examples
{
  int tempo=700;                                                              // changes the tempo (speed) that the music plays at
  int notes=29;                                                               // number of notes to be played

  int melody[]={                                                              // define melody (Row, row, row your boat)
  NOTE_C6,NOTE_C6,NOTE_C6,NOTE_D6,NOTE_E6,0,
  NOTE_E6,NOTE_D6,NOTE_E6,NOTE_F6,NOTE_G6,0,
  NOTE_C7,NOTE_C7,NOTE_C7,NOTE_G6,NOTE_G6,NOTE_G6,NOTE_E6,NOTE_E6,NOTE_E6,NOTE_C6,NOTE_C6,NOTE_C6,
  NOTE_G6,NOTE_F6,NOTE_E6,NOTE_D6,NOTE_C6
  };

  byte noteDurations[]={                                                      // define the duration of each note here
  2,2,2,4,2,4,
  2,4,2,4,2,4,
  4,4,4,4,4,4,4,4,4,4,4,4,
  2,4,2,4,2
  };
  
  for (byte Note = 0; Note < notes; Note++)                                   // Play melody
  {
    long pulselength = 1000000L/melody[Note];
    long noteDuration = tempo/noteDurations[Note];
    long pulses=noteDuration*tempo/pulselength;
    
    if (pulselength>100000L)                                                  // a note of 0 generates a pause
    {
      delay(noteDuration);
    }
    else
    {
      for(int p=0;p<pulses;p++)
      {                                                                      
        PORTB=PORTB|B00001111;                                                // set all motor direction and PWM pins high
        delayMicroseconds(pulselength/2-20);                                  // frequency of note divide by 2
        
                                                                              // drive motors backward
        PORTB=PORTB&B11110110;                                                // drive motors backward - set direction bits low
        delayMicroseconds(pulselength/2-20);                                  // frequency of note divide by 2
      }
      
      PORTB=PORTB&B111000;                                                    // turn off motors
      delay(noteDuration * 3 / 10);                                           // short pause between notes
    }
  }
}

//============================================================== Receive I²C command from external device or other MCU =======================

void I2C_Receive(int bytes)                                   // received command as I²C slave     
{
  for(byte i=0;i<bytes;i++)                                 
  {
	datapack[i]=Wire.read();                                  // transfer data from I²C buffer to datapack
  }
  
  command=datapack[0];
  packsize=bytes;
  
  //Serial.print("Pack size:");Serial.println(bytes,DEC);
  //Serial.print("Command:");Serial.println(command,DEC);
  
  if(mode==1 && command<5)
  {
	mode=0;                                                   // disable demo mode
	EEPROM.write(1,mode);
  }
  
  //------------------------------------------------------------ demo syncronization ---------------------------------------------------------
  
  if(command==15 && packsize==3)
  {
	EEPROM.write(0,datapack[1]);                              // only update EEPROM if necessary
	EEPROM.write(1,datapack[2]);                              // only update EEPROM if necessary
	
	demo=datapack[1];
	mode=datapack[2];
	
	command=255;
	return;
  }
  
  if(command==7 && packsize==3)  //============================= Demo Modes Angle Update ======================================================
  {
	angle=datapack[1]*256+datapack[2];
	Trigonometry();
	command=255;
	return;
  }
}
		
// https://www.arduino.cc/en/Tutorial/MasterReader
// http://arduino.stackexchange.com/a/3388/4478

void send_wire_int(int num){

  // Uncomment below if you want so send back in same format
  // Wire.write(0);

  // Send low byte
  Wire.write((uint8_t)num);

  // Send high byte
  num >>= 8;
  Wire.write((uint8_t)num);
}

void I2C_Send()
{
	//TODO:fix
	// Send encoder A count.
	//send_wire_int(acount);
  
	// Send encoder B count.
	//send_wire_int(bcount);
	
	Wire.write(loop_count);
}


void EEPROMdefaults()
{
  byte defaults[]=
  {1,1,0,60,255,255,255,255                
  ,1,0,0,52,188,52,188,52,188,52,188
  ,3,32,3,32,3,32,3,32
  ,10,10,10,10,10,10,10,10
  ,4,37,128,37,128,170};                                            // default configuration data
                  
  for(int i=0;i<41;i++)
  {
    EEPROM.write(i,defaults[i]);                                    // store defaultes in EEPROM
  }
  EEPROMload();                                                     // load configuration from EEPROM
}


void EEPROMload()                                                   // load configuration from EEPROM into program
{
  demo=EEPROM.read(0);                                              // demo mode
  mode=EEPROM.read(1);                                              // shield mode
  
  encoders=1;                                                       // default to encoders enabled
  configuration=EEPROM.read(2);                                     // configuration
  if(configuration&16) encoders=0;                                  // disable encoders
    
  lowbat=EEPROM.read(3);                                            // low battery
  for(byte i=0;i<4;i++)                                             // load motor current limits
  {
    maxamps[i]=EEPROM.read(4+i);                                    // maxamp values
    motrpm[i]=EEPROM.read(i*2+11)*256+EEPROM.read(i*2+12);          // motor RPM values
    encres[i]=EEPROM.read(i*2+19)*256+EEPROM.read(i*2+20);          // encoder resolution (x100)
    reserve[i]=EEPROM.read(27+i);                                   // motor power reserve values
    stalltm[i]=long(EEPROM.read(31+i));                             // motor stall time in mS
  }
  
  master=EEPROM.read(8);                                            // external I²C master address
  addroffset=EEPROM.read(9);                                        // shield I²C address offset
  i2cfreq=EEPROM.read(10);                                          // I²C clock: 0=100kHz, 1=400kHz
  
  sermode=EEPROM.read(35);
  baudrate[0]=(EEPROM.read(36)*256U+EEPROM.read(37));
  baudrate[1]=(EEPROM.read(38)*256U+EEPROM.read(39));
  
  defaulted=EEPROM.read(40);                                        // flag to indicate defaults have been loaded
}


void EEPROMsave()                                                   // save configuration data from program to EEPROM
{
  EEPROM.write(0,demo);
  EEPROM.write(1,mode);
  EEPROM.write(2,configuration);
  EEPROM.write(3,lowbat);
  
  EEPROM.write(4,maxamps[0]);
  EEPROM.write(5,maxamps[1]);
  EEPROM.write(6,maxamps[2]);
  EEPROM.write(7,maxamps[3]);
  
  EEPROM.write(8,master);
  EEPROM.write(9,addroffset);
  EEPROM.write(10,i2cfreq);
  
  for(byte i=0;i<4;i++)
  {
    EEPROM.write(i*2+11,highByte(motrpm[i]));
    EEPROM.write(i*2+12, lowByte(motrpm[i]));
    EEPROM.write(i*2+19,highByte(encres[i]));
    EEPROM.write(i*2+20, lowByte(encres[i]));
    EEPROM.write(i+27,reserve[i]);
    EEPROM.write(i+31,byte(stalltm[i]));
  }
  EEPROM.write(35,sermode);
  EEPROM.write(36,highByte(baudrate[0]));
  EEPROM.write(37, lowByte(baudrate[0]));
  EEPROM.write(38,highByte(baudrate[1]));
  EEPROM.write(39, lowByte(baudrate[1]));
  
  EEPROM.write(40,170);
}

/*
    EEPROM MAP
    
    addr  description

    0     demo mode:       true=active    false=paused
    1     shield mode:     true=demo      false=I²C / Serial
    2     configuration:   0=3xomni       1=4xomni        2=mecanum        3=individual        +16 (bit 4 high)=no encoders
    3     low bat volts:   60=6.0V
    4     M1 current:      255=2.55A
    5     M2 current:      255=2.55A
    6     M3 current:      255=2.55A
    7     M4 current:      255=2.55A

    8     master I²C addr: address of external master (default=1)
    9     I²C addr offset: shield address offset (default=0)
    10    I²C Clock:       0=100  1=400
    11    M1 RPM Hi:       Motor 1 RPM high byte
    12    M1 RPM Lo:       Motor 1 RPM low  byte
    13    M2 RPM Hi:       Motor 2 RPM high byte
    14    M2 RPM Lo:       Motor 2 RPM low  byte
    15    M3 RPM Hi:       Motor 3 RPM high byte
    16    M3 RPM Lo:       Motor 3 RPM low  byte
    17    M4 RPM Hi:       Motor 4 RPM high byte
    18    M4 RPM Lo:       Motor 4 RPM low  byte

    19    E1 Res Hi:       Encoder 1 resolution high byte
    20    E1 Res Lo:       Encoder 1 resolution low  byte
    21    E2 Res Hi:       Encoder 2 resolution high byte
    22    E2 Res Lo:       Encoder 2 resolution low  byte
    23    E3 Res Hi:       Encoder 3 resolution high byte
    24    E3 Res Lo:       Encoder 3 resolution low  byte
    25    E4 Res Hi:       Encoder 4 resolution high byte
    26    E4 Res Lo:       Encoder 4 resolution low  byte

    27    M1 Reserve:      Motor 1 reserve power
    28    M2 Reserve:      Motor 2 reserve power
    29    M3 Reserve:      Motor 3 reserve power
    30    M4 Reserve:      Motor 4 reserve power
    31    M1 stall time:   Motor 1 stall time
    32    M2 stall time:   Motor 2 stall time
    33    M3 stall time:   Motor 3 stall time
    34    M4 stall time:   Motor 4 stall time
    
    35    Serial Mode:     0=pass all data to master    1=accept motor control from port 1    2-accept motor control from port 2
			   3=accept control from and return data to port 1	 4=accept control from and return data to port 2

    36    Baud 1 Hi:	   Port 1 baud rate high byte
    37	  Baud 1 Lo:       Port 1 baud rate low  byte
    38    Baud 2 Hi:	   Port 2 baud rate high byte
    39    Baud 2 Lo:       Port 2 baud rate low  byte
        
    40    defaults flag    170=defaults loaded (170 = B10101010)
*/




void Commands()
{
  if(mcu==0 && (command<5 || command==10))                                   // if this is MCU1 and command is 0-4 then repeat command to MCU2 
  {
    Wire.beginTransmission(address+1);                                       // take control of I²C bus and address MCU2
    Wire.write(datapack,packsize);                                           // relay commands to MCU2
    Wire.endTransmission();                                   
  }
  
  if(command==10) 
  {
    EEPROMdefaults();
    command=255;
    return;
  }
  
  if(command>15) command-=16;                                                // 16 is added (bit 3 set high) for internal commands (no repeat)
  
  
  
  
  if(command==1 && packsize==10) //============================================ Basic Configuration Data Received =================================================
  {
    mode=datapack[1];
    configuration=datapack[2];
    lowbat=datapack[3];
    maxamps[0]=datapack[4];
    maxamps[1]=datapack[5];
    maxamps[2]=datapack[6];
    maxamps[3]=datapack[7];
    addroffset=datapack[8];
    master=datapack[9];
    EEPROMsave();                                                            // update EEPROM
    TXconfig();
    command=255;
    return;
  }

  if(command==2) //============================================================ Encoder Configuration Data Received =================================================
  {
    if(packsize==25)                                                         // configure each encoder individually
    {
      for(byte i=0;i<4;i++)
      {
        motrpm[i]=datapack[i*2+1]*256+datapack[i*2+2];
        encres[i]=datapack[i*2+9]*256+datapack[i*2+10];
        reserve[i]=datapack[18+i];
        stalltm[i]=datapack[22+i];
      }
    }
    else if(packsize==7)                                                     // use 1 configuration for all encoders
    {
      for(byte i=0;i<4;i++)
      {
        motrpm[i]=datapack[1]*256+datapack[2];
        encres[i]=datapack[3]*256+datapack[4];
        reserve[i]=datapack[5];
        stalltm[i]=datapack[6];
      }
    }
    for(byte i=0;i<4;i++)
    {
      maxpulse[i]=60000000L/(long(motrpm[i])*long(encres[i])/100L)*255L;     // update maxpulse values
      maxpulse[i]=maxpulse[i]*(100L-long(reserve[i]))/100L;
    }
    EEPROMsave();                                                            // update EEPROM
    TXconfig();
    command=255;
    return;
  }

  if(command==3 && (packsize==7 || packsize==9)) //============================ Motor Control =======================================================================
  {
    if((configuration==3 || configuration==19) && packsize==9)               // Individual motor control  
    {
      for(byte i=0;i<4;i++)
      {
        mspeed[i]=datapack[i*2+1]*256+datapack[i*2+2];
      }
    }
    else                                                                     // Omni or Mecanum Wheels 
    {
      velocity=datapack[1]*256+datapack[2];
      angle=datapack[3]*256+datapack[4];
      rotation=datapack[5]*256+datapack[6];
      
      if(velocity>255) velocity=255;
      if(velocity<-255) velocity=-255;
      
      while(angle>=360) 
      {
        angle-=360;
      }
      while(angle<0) 
      {
        angle+=360;
      }
      
      if(rotation>255) rotation=255;
      if(rotation<-255) rotation=-255;
      
      Trigonometry();
    }
    command=255;  
    return;
  }

  if(command==4 && packsize==6) //============================================= Serial port configuration ===========================================================
  {
    baudrate[0]=datapack[1]*256U+datapack[2];
    baudrate[1]=datapack[3]*256U+datapack[4];
    sermode=datapack[5];
    Serial.begin(baudrate[mcu]);                                             // change serial port baud rate
    EEPROMsave();                                                            // update EEPROM
    command=255;
    return;
  }

  if(command==5 && packsize>2 && packsize<33) //=============================== Send Serial Data =====================================================================
  {
    if((mcu+1)==datapack[1])
    {
      for(byte i=0;i<(packsize-2);i++)
      {
        serpack[i]=datapack[i+2];
      }
      Serial.write(serpack,packsize-2);
    }
    command=255;
    return;
  }  

  if(command==6 && packsize==2) //============================================= Status request ======================================================================
  {
                                                                             // each mcu sends it's data seperately to minimize interferance with motor speed control
                                                                             
    byte spsize=0;                                                           // intitial send pack size = 0
    int request=datapack[1];                                                 // copy datapack to global variable "request" ASAP so datapack can be reused
    
    if(request&1)// Bit 0:                                                   // return encoder counter values
    {
      sendpack[0] =highByte(acount);
      sendpack[1] = lowByte(acount);
      sendpack[2] =highByte(bcount);
      sendpack[3] = lowByte(bcount);
      spsize=4;                                                              // increment pack size by 4 bytes (counts from 2 encoders only)
    }

    if(request&2)// Bit 1:                                                   // reset encoder counters
    {
      acount=0;
      bcount=0;
    }

    if(request&4)// Bit 2:                                                   // return motor currents
    {
      sendpack[spsize+0] =highByte(analogvalue[0]);
      sendpack[spsize+1] = lowByte(analogvalue[0]);
      sendpack[spsize+2] =highByte(analogvalue[1]);
      sendpack[spsize+3] = lowByte(analogvalue[1]);
      spsize+=4;                                                             // increment pack size by 4 bytes (current from 2 motors only)
    }

    if(((request&8) && mcu==0) || ((request&16) && mcu==1))// Bits 3&4:      // return MCU analog values
    {
      sendpack[spsize+0]=highByte(analogvalue[2]);
      sendpack[spsize+1]= lowByte(analogvalue[2]);
      sendpack[spsize+2]=highByte(analogvalue[3]);
      sendpack[spsize+3]= lowByte(analogvalue[3]);
      sendpack[spsize+4]=highByte(analogvalue[4]);
      sendpack[spsize+5]= lowByte(analogvalue[4]);
      spsize+=6;
    }

    if(request&32)// Bit 5:                                                  // return error log
    {
      sendpack[spsize]=eflag;
    }

    if(request&64)// Bit 6:                                                  // clear error log
    {  
      eflag=0;
    }
    
    byte returnaddress=master;                                               // return address is I²C master by default
    if((request&127) && mcu==0) returnaddress=address+1;                     // bit 7 indicates internal request - return to other processor
    if((request&127) && mcu==1) returnaddress=address-1;                     // bit 7 indicates internal request - return to other processor
    
    Wire.beginTransmission(returnaddress);
    Wire.write(sendpack,spsize);
    Wire.endTransmission();
    command=255;
  }
}


void setup()
{
  //============================================================================ Use Reset Button to select demo mode ===================================================================
  
  EEPROMload();                                                               // load configuration from EEPROM
  if(defaulted!=170) EEPROMdefaults();                                        // load defaults if no previous configuration found or in demo mode
  
  for(byte i=0;i<4;i++)
  {
    maxpulse[i]=60000000L/(long(motrpm[i])*long(encres[i])/100L)*255L;
    maxpulse[i]=maxpulse[i]*(100L-long(reserve[i]))/100L;
  }
  
  DDRD=B00000011;                                                             // ensure dipswitch pins (PD4-PD7) and encoder inputs (PD2,PD3) plus RX and TX are set to input
  PORTD=PORTD|B11111100;                                                      // enable pullup resistors on dipswitch pins (PD4-PD7) and encoder inputs (PD2,PD3)
  DDRB=DDRB|B00001111;                                                        // set motor control pins PB0 - PB3 (D8,D9,D10,D11) as output
  

  mcu=digitalRead(IDpin);                                                     // low = MCU 1    high = MCU 2
  address=((PIND&B11110000)>>3)+addroffset+mcu;                               // I²C address is selected by dip switches + offset + state of ID pin (MCU1 or MCU2).
  motora=mcu*2;
  motorb=mcu*2+1;
  
  Wire.begin(address);                                                        // initialize I²C library and set slave address
  Wire.onReceive(I2C_Receive);                                                // define I²C slave receiver ISR
  Wire.onRequest(I2C_Send);                                                   // define I²C slave transmit ISR
  Wire.setTimeout(1L);                                                        // sets a timeout of 1mS for I²C
  Wire.flush();
  delay(100);                                                                 // required to ensure both processors are initialized before inter-communications begins
    
  if(i2cfreq==0)                                                              // thanks to Nick Gammon: http://gammon.com.au/i2c
  {
    TWBR=72;                                                                  // default I²C clock is 100kHz
  }
  else
  {
    TWBR=12;                                                                  // change the I²C clock to 400kHz
  }
  
  if(mode==1 && mcu==0)                                                       // if demo mode is selected
  {
    demo+=1;                                                                  // toggle demo every time power is turned on or reset is pressed
    if(demo>1) demo=0;                                                        // limit demo to 0, 1 or 2
    
    angle=0;
    
    EEPROM.write(0,demo);                                                     // update demo mode
    
    Wire.beginTransmission(address+1);
    datapack[0]=15;                                                           // command 15 used to syncronize demo mode 
    datapack[1]=demo;
    datapack[2]=mode;
    Wire.write(datapack,3);
    Wire.endTransmission();
  }
  else
  {
    delay(10);
  }

  Serial.begin(long(baudrate[mcu]));                                          // initialize Serial library and set baud rate
  Serial.setTimeout(1L);                                                      // sets a timeout of 1mS for Serial. Baud below 9600 should not be used unless this value is increased
  
  //if(mcu) TXconfig();
  delay(1);
  
  while(millis()-time<1000)
  {
  }
  
  if(mode)                                                                    // if the shield is in demo mode
  {
    if(demo==0) Beep(3);
    if(demo==1) Tune();                                                       // Demo mode - play "Row, row, row your boat" using the motors for speakers
    if(demo==2) Beep(5);
  }
  
  //============================================================================ Encoder Interrupts =====================================================================================

  attachInterrupt(0,Aencoder,CHANGE);                                         // call ISR for left  encoder when state changes
  attachInterrupt(1,Bencoder,CHANGE);                                         // call ISR for right encoder when state changes
}

void loop()
{ 
	
	loop_count += 1;
	
  // Read analog inputs including battery voltage and motor currents
  analog++;                                                                   // select a different input each loop (analog read takes 260uS)
  if(analog>4) analog=0;                                                      // rotate through current A, current B, A3, A6 and A7
  
  if(mode && demo>0)
  {
    if(demo==1) LineFollow();
    if(demo==2) Avoidance();
    Motors();
    return;
  }
  
  analogvalue[analog]=analogRead(analogpin[analog]);                          // read selected analog input and store in array for later retrieval
  if(mcu==0 && analog==4) voltage=analogvalue[4]*30/185;                      // convert to battery voltage (60 = 6.0V)
  
  // Shut down motors if battery is equal or below lowbat

  if(mcu==0 && analog==4 && powerdown<250)                                    // battery voltage has just been read and no powerdown has occured
  {
    if(analogvalue[4]<=lowbat)                                                // compare battery voltage to low battery voltage
    {
      eflag=eflag|B00010000;                                                  // bit 5 indicates power dipping below batlow voltage
      powerdown++;                                                            // increment shutdown counter if battery voltage is low
    }
    else
    {
      powerdown=0;                                                            // reset shutdown counter if battery voltage is high
    }
    if(powerdown>249) PowerDown();                                            // if battery voltage consistantly low for 250 samples shutdown all motors
  }
  if(powerdown>249) return;                                                   // power must be cycled or reset button pressed to resume
  
  // Shield Functions
     
  if(Serial.available()>0) SerialInput();                                     // receive and transfer serial data depending on serial mode
  if(command<32) Commands();                                                  // respond to command from I²C bus or Serial interface
  if(encoders)
  {
    Motors();                                                                 // if encoders are enabled then use then to control motor speeds
  }
  else                                                                        // if encoders are disabled then feed speed values directly to PWM output
  {                                                                            
    analogWrite(pwmapin,abs(mspeed[mcu*2]));                                  // set motor A speed
    digitalWrite(dirapin,mspeed[mcu*2]>0);                                    // set motor A direction
    analogWrite(pwmbpin,abs(mspeed[mcu*2+1]));                                // set motor B speed
    digitalWrite(dirapin,mspeed[mcu*2+1]>0);                                  // set motor B direction
  }
}


