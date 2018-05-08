// free-PEMF See: biotronika.pl  biotronics.eu
// Chris Czoba (krzysiek@biotronika.pl) & hehaka

//#define SERIAL_DEBUG


#define HRDW_VER "NANO 4.2"
#define SOFT_VER "2018-05-07"
#include "freePEMF_func.h"



void btnEvent() {
    //Change button state interruption
    //unsigned long pressTime =0;

    if (digitalRead(btnPin)==HIGH){
        pressTime = millis(); //Specific use of millis(). No increment in innteruption function.
    } else {
        if (pressTime && (millis() - pressTime > 50)) pause=!pause;
        if (pressTime && (millis() - pressTime > 1000)) {
            for(unsigned int i=0; i<50000; i++) digitalWrite(buzzPin, HIGH); //Cannot use delay() therefore beep() function in innteruption
            digitalWrite(buzzPin, LOW);
            off();
        }
        pressTime = 0;
    }
}


void setup() {  
              
  // Initialize the digital pin as an in/output
  pinMode(coilPin,  OUTPUT);  // Coil driver
  pinMode(powerPin, OUTPUT);  // Power relay
  pinMode(greenPin, OUTPUT);  // LED on board  
  pinMode(redPin,   OUTPUT);  // LED on board 
  pinMode(relayPin, OUTPUT);  // Direction signal relay
  pinMode(buzzPin,  OUTPUT);  // Buzzer relay (5V or 12V which is no so loud)
  pinMode(btnPin,    INPUT);  // Main button
  pinMode(hrmPin,    INPUT_PULLUP); //Devices connection   

  //BIOzap
  // Initialize serial communication to PC communication
  Serial.begin(9600);
  // reserve the bytes for the inputString:
  inputString.reserve(65); //NANO serial buffer has 63 bytes

  
  pcConnection = (bat() < USB_POWER_SUPPLY_LEVEL);

  if (bat() < USB_POWER_SUPPLY_LEVEL) { 
    //Detected USB PC connection

    programNo = 0; //PC
    
  } else if (digitalRead(btnPin)==HIGH) {
    //Power button pressed
    
    //Turn power on
    digitalWrite(powerPin, HIGH);
    
  } else {
    //Power supplyer id pluged

    if (digitalRead(hrmPin)==LOW) {
      //Work as the water magnetizer

      //Special signal
      digitalWrite(greenPin, HIGH);
      digitalWrite(redPin, HIGH);
      beep(250);
      delay(1000);
      digitalWrite(redPin, LOW);
      
      digitalWrite(greenPin, HIGH);
      
      beep(250);
      delay(1000);

      //Forever  loop
      while (1) {
        digitalWrite(greenPin, HIGH);
        digitalWrite(powerPin, HIGH);
        delay(50);
        executeCmd("exe\n"); //default programNo;
        digitalWrite(powerPin, LOW);
        delay(10000);
      }
      
      //digitalWrite(greenPin, LOW);
    }
    
    //Work as a power charger
    rechargeBattery();
  }

  //Turn on green LED
  digitalWrite(greenPin, HIGH);
  beep(200);
  
  //Wait until turn-on button release
  startInterval = millis();
  while (digitalRead(btnPin)==HIGH){
    if ( millis() > ( startInterval + btnTimeOut ) ) {
      programNo = 5; //Coil measurement test
      digitalWrite(redPin, HIGH);
      
    }
  };
  
  delay(10);

  //Auto-correction voltage - for new device
  if ( (byte)EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS) > 130 ||
       (byte)EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS) < 70 ) {
    EEPROM.put(EEPROM_BATTERY_CALIBRATION_ADDRESS,100); // 100 =  x 1.00
  }
  
  //Define minimum battery level uses in working for perfomance puropose.
  minBatteryLevel /*0-1023*/= 100 * 
                              MIN_BATTERY_LEVEL / 
                              BATTERY_VOLTAGE_RATIO / 
                              (byte)EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS);

                                
 if (programNo) { 
    // not PC option (programNo>0)
    //Program select  
    unsigned long startInterval = millis();
    while(((millis()-startInterval) < btnTimeOut) && programNo!=5){
      if (digitalRead(btnPin)) {  
          //Reset start moment after btn preesed
          startInterval = millis();
                
          programNo++;
          if (programNo>4) programNo=1;
          for (int p=programNo; p>0; p--){
            //Signals count
            beep(80); delay(150);
          }

          //Wait until button is pressed
          while(digitalRead(btnPin));
         
          //Turn off if pressed more then 1 sec.
          if ((millis()-startInterval) > 1000 ) { 
            beep(500); 
            off();
          } 
      }
    }
    
    // Configure button interrupt
    attachInterrupt(digitalPinToInterrupt(btnPin), btnEvent, CHANGE);
    
  } else {
    //PC option
    
    //Power on
    digitalWrite(powerPin, HIGH);
     
    Serial.println(WELCOME_SCR);
    Serial.print("Device free-PEMF ");
    Serial.print(HRDW_VER);
    Serial.print(" ");
    Serial.println(SOFT_VER);
    Serial.print('>');  
    
    // Configure button interrupt for using off option
    attachInterrupt(digitalPinToInterrupt(btnPin), btnEvent, CHANGE); 
    
  }

  startInterval=millis();
}


void loop() {

  switch (programNo) {
    case 1:
      //Check if user program is load in EEPROM memory
      //eeFirst=(byte)EEPROM.read(0);
      //Serial.println(eeFirst);
      if ((byte)EEPROM.read(0)!=255 && (byte)EEPROM.read(0)!=0) {
        //User program execute
   
        executeCmd("exe\n");
        off();
        
      } else {
        //Standard program execute
    	 executeCmd("prog 1\n");
      }
      break;
      
    case 2:
    //Earth regeneration - 8 minutes
    	executeCmd("prog 2\n");
    	break;
    case 3:
    // Antistress & meditation (without feedback)
    	executeCmd("prog 3\n");
    	break;
    case 4:
    // Pineal gland
    	executeCmd("prog 4\n");
    	break;

    case 5:
      digitalWrite(redPin, LOW);
      while(1) {
        checkBattLevel(); //If too low then off
        if (digitalRead(btnPin)) { 
          startInterval = millis();
          beep(100); 
          while(digitalRead(btnPin));
          
          if ((millis()-startInterval) > btnTimeOut ) { 
            beep(500); 
            off();
          }
                            
          if (coilState == LOW) {
            coilState = HIGH;
          } else {
            coilState = LOW;
          }
        
          digitalWrite(coilPin, coilState);   // turn coil on/off 
          digitalWrite(redPin, coilState);   // turn LED on/off

        }
      }
      break;
     
    default: 
    
    // PC controled program   
      if (stringComplete) {

        //Restart timeout interval to turn off. 
        startInterval=millis();       
        
        executeCmd(inputString);
        Serial.print('>'); //Currsor for new command

        // clear the command string
        inputString = "";
        stringComplete = false;  
      } 
    
    break; 
    
  } 
  if (millis()-startInterval > pauseTimeOut) off();
} 





