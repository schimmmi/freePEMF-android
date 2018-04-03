/*
 * bioZAP_func.h
 *
 *  Created on: 22 lut 2018
 *      Author: blos
 */

#ifndef FREEPEMF_FUNC_H_
#define FREEPEMF_FUNC_H_

#include <Arduino.h>
#include <EEPROM.h>
#include "freePEMF_prog.h"
#include <TimerOne.h>
#include "FPTimer.h"


//Pin definition
#define coilPin 5      // Coil driver IRF510
#define powerPin 4     // Power relay
#define relayPin 9     // Direction relay
#define buzzPin 10     // Buzzer
#define buzerPin 10     // Buzzer
#define btnPin 3       // Power On-Off / Pause / Change program button 
#define redPin 12      // Red LED
#define greenPin 11    // Green LED
#define hrmPin 2       // Biofeedback HR meter on 3th plug pin.
#define pin3Pin 2

//Battery staff
#define batPin A7                               // Analog-in battery level 
#define BATTERY_VOLTAGE_RATIO 0.153             // nclude div 10k/4,7k resistors. 5V*(10k+4,7k)/4,7k = 0,0153 (x10)
#define EEPROM_BATTERY_CALIBRATION_ADDRESS 1023 // Memory address of battery correction factor - 100 means RATIO x 1,00
#define MIN_BATTERY_LEVEL 90                    // 90 means 9.0 V  (x10), less then that turn off
#define USB_POWER_SUPPLY_LEVEL 65               // Maximum usb voltage level means 6.5V


//BIOzap
#define WELCOME_SCR "Free BIOzap interpreter welcome! See http://biotronika.pl"
#define PROGRAM_SIZE 1000   // Maximum program size
#define PROGRAM_BUFFER 500  // SRAM buffer size, used for script loading
#define MAX_CMD_PARAMS 3    // Count of command params
#define MIN_FREQ_OUT 1      //  0.01 Hz
#define MAX_FREQ_OUT 12000000   // 50.00 Hz
#define SCAN_STEPS 100      // For scan function puropose - default steps
#define XON 17  //0x11
#define XOFF 19 //0x13


#define FREQ_MIN 1                    // 0.01 Hz
#define FREQ_MAX 12000000            // 120kHz
#define FREQ_MAX_PERIOD 1800        //Maximum duration of freq function in seconds


//BIOzap
String inputString = "";                // a string to hold incoming serial data
String param[MAX_CMD_PARAMS];           // param[0] = cmd name
boolean stringComplete = false;         // whether the string is complete
boolean memComplete = false;
unsigned long lastFreq = MIN_FREQ_OUT;  // Uses scan function
int minBatteryLevel = 0;
boolean Xoff = false;


const unsigned long checkDeltaBatteryIncreasingVoltageTime = 600000UL;  // During charging battery minimum inreasing voltage after x millisecounds. 
//  If after the x period the same voltage, green led starts lights.
const unsigned long pauseTimeOut = 600000UL;                            // 600000 Time of waiting in pause state as turn power off. (60000 = 1 min.)
const unsigned int btnTimeOut = 5000UL;                                 // Choose therapy program time out. Counted form released button.
boolean outputDir = false;
byte coilState = LOW;
unsigned long pauseTime = 0;

volatile boolean pause = false; // true = pause on
unsigned long pressTime = 0;    // Time of pressing the button
unsigned long startInterval;    // For unused timeout off.
byte programNo = 1;             // 0 = PC connection, 1= first program etc.
byte hr = 0;                    // User pulse from hrmPin

int adr = 0;

//Serial buffer
//char *memBuffer; 
char memBuffer[PROGRAM_BUFFER];

#define MAX_LABELS 9

int labelPointer[MAX_LABELS + 1];                // Next line of label
int labelLoops[MAX_LABELS + 1];                    // Number of left jump loops

boolean pcConnection = false;

byte pin3 = 0;            //State of pin3

String line;
int currentProgram = -1;


//Global vars
char key;
byte b;
int i;
long l;

unsigned long lastOperationMillis; //For auto turn off



//Labels & jumps
#define MAX_LABELS 9                            // Number of labels in script therapy (0 is not used)


//function prototypes
int readEepromLine(int fromAddress, String &lineString);

void getParams(String &inputString);

int executeCmd(String cmdLine);

void eepromUpload(int adr = 0);

boolean readSerial2Buffer(int &endBuffer);

void rechargeBattery();

//int initBatteryLevel();
void checkBattLevel();


void chp(byte outputDir);

void off();

void beep(unsigned int period);

//void freq(unsigned long Freq, unsigned int period);

void rec(unsigned long Freq, unsigned long period);  //deprecated

int bat();

void wait(unsigned long period);

void exe(int &adr, int prog = -1);

void exe();

void scan(unsigned long Freq, unsigned int period);

int mem();

void ls();

void rm();

//void cbat();
int jump(int labelNumber, int &adr);



///////////////////////////// bioZAP functions ///////////////////////////////


String formatLine(int adr, String line) {
    String printLine;
    printLine.reserve(22);
    printLine = "000" + String(adr, DEC);
    printLine = printLine.substring(printLine.length() - 3, printLine.length());
    printLine += ": " + line; //end marker for appending program
    return printLine;
}


int executeCmd(String cmdLine) {
// Main interpreter function

#ifdef SERIAL_DEBUG
        Serial.print("EXECUTE: ");
        Serial.print(cmdLine);
#endif

    getParams(cmdLine);

    if (param[0] == "mem") {
// Upload therapy to EEPROM
        mem();
    } else if (param[0] == "ls") {
//List therapy
        ls();
    } else if (param[0].charAt(0) == '#') {
// Comment
        ;
    } else if (param[0].charAt(0) == ':') {
// Label - setup for new label jump counter
        b = param[0].substring(1).toInt();
        if (b > 0 && b < MAX_LABELS) {
            if (param[1].length() >= 1) {
                if (param[1].toInt()) {
                    labelLoops[b] = param[1].toInt() - 1;
                }
#ifdef SERIAL_DEBUG
                Serial.print("label: ");
        Serial.print(b);
        Serial.print(" ");
        Serial.println(labelLoops[b]);
#endif
            } else {
                labelLoops[b] = -1; //Infinity loop
            }
        }
    } else if (param[0] == "") {
// Emptyline
        ;
    } else if (param[0] == "print") {
    	// Emptyline

    	 if (cmdLine.length()>6) {
    	        Serial.println(cmdLine.substring(6,cmdLine.length()-1));
    	      } else {
    	        Serial.println();
    	      }

    } else if (param[0] == "rm") {
// Remove, clear script therapy from memory
        rm();
    } else if (param[0] == "bat") {
// Print battery voltage
        if (pcConnection) Serial.println(bat());

    } else if (param[0] == "beep") {
// Beep [time_ms]
        beep(param[1].toInt());

    } else if (param[0] == "pin3") {
// Socket pin3 <state> If ną state pin3 = !pin3
        if (param[1].length() == 1) {
            if (param[1].charAt(0) == '~') {
                if (pin3) pin3 = 0; else pin3 = 1;
            } else if (param[1].toInt() == 1) {
                pin3 = 1;
            } else {
                pin3 = 0;
            }
            //pinMode(pin3Pin, OUTPUT);
            digitalWrite(pin3Pin, pin3);
        }
    } else if (param[0] == "jump") {
// Jump [label number]

#ifdef SERIAL_DEBUG
        //Serial.print("jump1 adr: ");
        //Serial.println(adr);
#endif
        if (jump(param[1].toInt(), adr)) {

#ifdef SERIAL_DEBUG
            //Serial.print("jump2: ");
        //Serial.println(param[1].toInt());
        //Serial.print("jump2 adr: ");
        //Serial.println(adr);
#endif
            return adr;
        }


    } else if (param[0] == "off") {
// Turn off
        off();
    } else if (param[0] == "wait") {
// Wait millis
        delay(param[1].toInt());
    } else if (param[0] == "freq" || param[0] == "rec" || param[0] == "sin") {
// Generate sinus or rectangle signal - freq [freq] [time_sec]
        l = param[1].toDouble();
        i = param[2].toDouble();
        if (l && i) {
            rec(
                    constrain(l, FREQ_MIN, FREQ_MAX),
                    constrain(i, 1, FREQ_MAX_PERIOD)
            );
        }
    } else if (param[0] == "scan") {
// Scan from lastFreq  - scan [freq to] [time_ms]
        //scan(param[1].toInt(), param[2].toInt());
        //TODO: lastFreq and Freq change to one
        l = param[1].toDouble();
        i = param[2].toDouble();
        if (l && i) {
            scan(
                    constrain(l, FREQ_MIN, FREQ_MAX),
                    constrain(i, 5, FREQ_MAX_PERIOD)
            );
        }
    } else if (param[0] == "chp") {
// Change output signal polarity

        //chp(byte(param[1].toInt()));
        chp(byte(param[1].toInt()));

#ifdef SERIAL_DEBUG
        Serial.println("OK");
#endif


    } else if (param[0] == "exe") {
        exe();
        Serial.println("EXECUTED");
    } else if (param[0] == "prog") {

    	currentProgram = param[1].toInt();
    	exe(adr, currentProgram);
    	Serial.println("EXECUTED");

    } else {
//Unknown command
        if (pcConnection) Serial.println("Unknown command: " + param[0]);
    }
    return 0;
}

void exe() {
    //Execute program

    int adr = 0;
    String line;
    while (int endLine = readEepromLine(adr, line)) {

#ifdef SERIAL_DEBUG
        Serial.print("executing: ");
        Serial.print(line);
#endif
        executeCmd(line);
        adr = adr + endLine;
    }

#ifdef SERIAL_DEBUG
    Serial.println("Script done.");
    Serial.println("OK");
#endif

}


void rm() {
// Remove, clear script therapy from memory
    EEPROM.put(0, '@');

//	for(int i=0; i<PROGRAM_SIZE; i++){
//		EEPROM.put(i, 255);
    //if (!(i % 128)) Serial.print(".");
//	}
    //Serial.println("OK");
}

void ls() {
//List script therapy
    int adr = 0;
    int endLine;
    String line;

    if (param[1] == "-n") {
        Serial.println("Adr  Command");

        while ((endLine = readEepromLine(adr, line)) && (adr < PROGRAM_SIZE)) {
            Serial.print(formatLine(adr, line));
            adr = adr + endLine;
        }

        //End marker (@) informs an user where to start appending of program
        if (adr < PROGRAM_SIZE) {
            Serial.println(formatLine(adr, "@"));
        }

    } else {

        for (int i = 0; i < PROGRAM_SIZE; i++) {
            char eeChar = (char) EEPROM.read(i);

            if ((eeChar == '@') || (eeChar == char(255))) {
                break;
            }

            Serial.print(eeChar);
        }
    }

}

int mem() {
// Upload therapy to EEPROM

    if (param[1] == "\0") {
        eepromUpload();

    } else if (param[1] == "@") {
        //Find script end
        int endAdr = 0;
        for (int i = 0; i < PROGRAM_SIZE; i++) {
            if ((byte) EEPROM.read(i) == 255 || (char) EEPROM.read(i) == '@') {
                endAdr = i;

                break;
            }
        }
        if (pcConnection) Serial.println(formatLine(endAdr, "appending from..."));
        eepromUpload(endAdr);

    } else if (param[1].toInt() > 0 && param[1].toInt() < PROGRAM_SIZE) {
        eepromUpload(param[1].toInt());
    } else {
        if (pcConnection) Serial.print("Error: unknown parameter ");
        if (pcConnection) Serial.println(param[1]);
        return -1;
    }

    return 0;
}

int jump(int labelNumber, int &adr) {

    if (labelNumber > 0 && labelNumber < MAX_LABELS) {
#ifdef SERIAL_DEBUG
        Serial.print("jump3 lblPtr: ");
        Serial.println(labelPointer[labelNumber]);
#endif

        if (labelPointer[labelNumber]) {

#ifdef SERIAL_DEBUG
            Serial.print("jump4 lblLoops: ");
            Serial.println(labelLoops[labelNumber]);
#endif

            if (labelLoops[labelNumber] > 0) {

                adr = labelPointer[labelNumber];    //Jump to new position
                labelLoops[labelNumber]--;            //Decrees jump counter

                return adr;

            } else if (labelLoops[labelNumber] == -1) { //Unlimited loop

                adr = labelPointer[labelNumber];
                return adr;

            }
        }
    }
    return 0;
}

int readFlashLine(int fromAddress, String &lineString) {
    //Read one line from EEPROM memory
    int i = 0;
    lineString = "";

#ifdef SERIAL_DEBUG
    //Serial.print("readFlashLine1 fromAddress: ");
  //Serial.println(fromAddress);
#endif

    do {

        char eeChar = char(pgm_read_byte(&internalProgram[fromAddress + i]));

#ifdef SERIAL_DEBUG
        //Serial.print("readFlashLine2 eeChar: ");
      //Serial.println(eeChar);
#endif
        if (eeChar == char('@')) {
            if (i > 0) {
                eeChar = '\n';
            } else {
                i = 0;
                break;
            }
        }
        lineString += eeChar;
        i++;
        if (eeChar == '\n') break;
    } while (1);
#ifdef SERIAL_DEBUG
    //Serial.print("readFlashLine3 i: ");
  //Serial.println(i);
#endif
    return i;
}

int readLabelPointers(int prog) {
    /* Initialize Labels pointers and jump loops
     * prog:
     * 0 - user program, jumps have counters,
     * 1-9 Internal programs,
     */
    int i;
    int adr = 0;

    for (i = 1; i < MAX_LABELS + 1; i++)
        labelLoops[i] = 0;

    i = 0;

    do {
        if (prog > 0) {
            //Internal program addresses
            adr = readFlashLine(i, line);
            getParams(line);
        } else {
            //EEPROM program labels
            adr = readEepromLine(i, line);
            getParams(line);
        }

        if (line.length() > 1)
            if (line[0] == ':') {
                byte lblNo = line[1] - 48;
                if (lblNo > 0 && lblNo < 10) {
                    labelPointer[lblNo] = i + line.length();  // Next line of label
                    //labelPointer[lblNo] = adr;  // Next line of label
                    if (param[1].length()) {

                        if (param[1].toInt() > 0) {
                            labelLoops[lblNo] = param[1].toInt() - 1;
                        }

                    } else {
                        labelLoops[lblNo] = -1;
                    }

                    if (lblNo == prog && prog > 0) return labelPointer[lblNo];

                }
            }

        i += line.length();
        //i=adr;

    } while (adr);

#ifdef SERIAL_DEBUG
    for (i=1; i<MAX_LABELS+1;i++){
        Serial.print("Label: ");
        Serial.print(i);
        Serial.print(" loops: ");
        Serial.print(labelLoops[i]);
        Serial.print(" ptr: ");
        Serial.println(labelPointer[i]);
    }
#endif


    return 0;
}

void exe(int &adr, int prog) {
//Execute program
    int endLine;

    //First time of internal program call
    if (!adr && (prog > 0)) {
        adr = readLabelPointers(prog);

    } else if (!adr) {
        readLabelPointers(0);
    }

    do {

#ifdef SERIAL_DEBUG
    Serial.print("exe1 prog: ");
    Serial.println(prog);
      Serial.print("exe1 adr: ");
    Serial.println(adr);
#endif

    if (prog > 0) {
        //EEPROM memory
        endLine = readFlashLine(adr, line);
    } else {
        //Flash memory
        endLine = readEepromLine(adr, line);
    }

#ifdef SERIAL_DEBUG
        Serial.print("exe2 endLine: ");
        Serial.println(endLine);
        Serial.print("exe2 adr: ");
        Serial.println(adr);
#endif

    //endLine = readEepromLine(adr,line);
    if (endLine) {
        //int endLine = readEepromLine(adr,line);

#ifdef SERIAL_DEBUG
        Serial.print("exe3 ");
        Serial.print(adr);
        Serial.print(": ");
        Serial.println(line);
#endif
        //executeNext line
        int adrJump = executeCmd(line);



        if (adrJump) {
            adr = adrJump;
        } else {
            adr = endLine +adr;
        }
        endLine = 0;

    } else {
        adr = 0;
    }

    } while (adr > 0);
}


void off() {
    // Power off function

    digitalWrite(coilPin, LOW);     // Turn coil off by making the voltage LOW
    delay(20);
    digitalWrite(relayPin, LOW);    // Relay off
    digitalWrite(greenPin, LOW);    // Green LED off

    digitalWrite(powerPin, LOW);  // Turn power off //If not USB power

    while (digitalRead(btnPin) == HIGH); // Wait because still power on

    //If USB PC connection is pluged microcontroller cannot turn power off
    //detachInterrupt(digitalPinToInterrupt(btnPin));

    Serial.println("OFF");

    while (1); //forever loop

}


void chp(byte outputDir) {
    //Change output polarity

    digitalWrite(coilPin, LOW);  // turning coil off
    if (outputDir) {
        digitalWrite(relayPin, HIGH); // turn relay on
    } else {
        digitalWrite(relayPin, LOW); // turn relay off
    }

}

int bat() {
    // Get battery voltage function

    return (analogRead((unsigned char)batPin) *
            BATTERY_VOLTAGE_RATIO *
            (byte) EEPROM.read(EEPROM_BATTERY_CALIBRATION_ADDRESS) /
            100
    );
}

void wait(unsigned long period) {
    // wait [period_ms]

    unsigned long serialStartPeriod = millis();
    unsigned long startInterval = millis();

    while (millis() - startInterval <= period) {
        //time loop

        //TODO serial break command - mark @

        //count each second
        if (millis() - serialStartPeriod >= 1000) {
#ifdef SERIAL_DEBUG
            Serial.print('.');
#endif
            serialStartPeriod = millis();
        }
    }

}

void beep(unsigned int period) {
    // beep [period_ms]

    unsigned long serialStartPeriod = millis();
    unsigned long startInterval = millis();

    digitalWrite(buzzPin, HIGH);
    while (millis() - startInterval <= period) {
        //time loop

        //TODO serial break command - mark @

        //count each second
        if (millis() - serialStartPeriod >= 1000) { //one second
#ifdef SERIAL_DEBUG
            Serial.print('.');
#endif
            serialStartPeriod = millis();
        }
    }

    digitalWrite(buzzPin, LOW);
}


void scan(unsigned long freq, unsigned int period) {
    // Scan from lastFreq to freq used SCAN_STEPS by period

    long scanSteps = SCAN_STEPS;


    long stepPeriod = period / scanSteps;
    if (stepPeriod < 1) {
        scanSteps = period;
        stepPeriod = 1;
    }
    long startFreq = lastFreq;
    long stepFreq = long(constrain(freq, MIN_FREQ_OUT, MAX_FREQ_OUT) - lastFreq) / scanSteps;


    for (int i = 0; i < scanSteps; i++) {
        rec(startFreq + (i * stepFreq), stepPeriod);
    }
}




void beepShort() {
    beep(100);
}

void beepNormal() {
    beep(200);
}

void beepLong() {
    beep(500);
}

void switchCoilState() {
    coilState ^= 1;
}


void verifyBatteryLevel() {

    if (analogRead((unsigned char) batPin) < minBatteryLevel) {
        //Emergency turn off
        Serial.println();
        Serial.print(F("Error: battery too low:"));
        Serial.println(bat());

        // red LED on
        digitalWrite(redPin, HIGH);
        digitalWrite(greenPin, LOW);

        // Turn all off
        digitalWrite(coilPin, LOW);    // Turn coil off by making the voltage LOW
        digitalWrite(relayPin, LOW);    // Relay off

        for (int x = 0; x < 10; x++) {
            digitalWrite(buzzPin, HIGH);   // Turn buzzer on
            delay(100);
            digitalWrite(buzzPin, LOW);    // Turn buzzer off
            delay(200);
        }
        beepLong();
        off();
    }

}

void callback() {
    switchCoilState();
    digitalWrite(greenPin, coilState);
    digitalWrite(coilPin, coilState);
}

void rec(unsigned long freq, unsigned long period) {
    //Rectangle signal generate, freq=783 for 7.83Hz, period in secounds

    lastFreq = constrain(freq, MIN_FREQ_OUT, MAX_FREQ_OUT); //For scan() function puropose
    unsigned long interval = (unsigned long) (50000000 / lastFreq);


    Serial.println();
    Serial.print(F("lastFreq: "));
    Serial.println(lastFreq);

    Serial.println();
    Serial.print(F("interval: "));
    Serial.println(interval);


    FPTimer timer;
    FPTimer secondCounter;
    unsigned long timeUp = period * 1000;

    Timer1.initialize(interval);
    Timer1.attachInterrupt(callback);

    while (timer.isTicking(timeUp)) {
        //time loop

        //TODO serial break command - mark @
        if (pause) {
            //Pause - button pressed
            fpTimer.resetTimer();
            beepNormal();
            digitalWrite(coilPin, LOW);     // turn coil ooff
            digitalWrite(greenPin, HIGH);   // turn LED on
            while (pause) {
                //wait pauseTimeOut or button pressed
                if (fpTimer.checkPauseTimeout()) {
                    beepLong();
                    off();
                }
            }
            beepNormal();
            //Correct working time
            timeUp += millis() - fpTimer.getTimerValue();
            //Continue
            digitalWrite(coilPin, coilState);    // turn coil on
            digitalWrite(greenPin, coilState);   // turn LED on/
        }

        //count each secondCounter
        if (secondCounter.checkTimeout(1000)) {  //count each secondCounter
        	Serial.print(F("."));
            secondCounter.resetTimer();
            verifyBatteryLevel(); //If too low then off
        }
    }
    Serial.println();

    Timer1.detachInterrupt();
    digitalWrite(coilPin, LOW);
    digitalWrite(coilPin, coilState);     // turn coil off
    digitalWrite(greenPin, HIGH);   // turn LED on
}



/***************** bioZAP functions end *************************/

int readEepromLine(int fromAddress, String &lineString) {
    //Read one line from EEPROM memory
    int i = 0;
    lineString = "";
    do {
        char eeChar = (char) EEPROM.read(fromAddress + i);

        if ((eeChar == char(255)) || (eeChar == char('@'))) {

            return 0;

        }

        lineString += eeChar;
        i++;
        if (eeChar == '\n') break;

    } while (1);

    return i;
}

void getParams(String &inputString) {
    for (int i = 0; i < MAX_CMD_PARAMS; i++) param[i] = "";

    int from = 0;
    int to = 0;
    for (int i = 0; i < MAX_CMD_PARAMS; i++) {
        to = inputString.indexOf(' ', from); //Wykryj spacje

        if (to == -1) {
            to = inputString.indexOf('\n', from); //Wykryj NL #10
            if (to > 0) param[i] = inputString.substring(from, to);
            param[i].trim();
            break;
        }

        if (to > 0) param[i] = inputString.substring(from, to);
        param[i].trim();
        from = to + 1;
    }
}


void checkBattLevel() {
  //Check battery level

  if ( analogRead((unsigned char)batPin) < minBatteryLevel) {
    //Emergency turn off
    Serial.println();
    Serial.print("Error: battery too low: ");
    Serial.println(bat());


    // red LED on
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);

    // Turn all off
    digitalWrite(coilPin, LOW);    // Turn coil off by making the voltage LOW
    digitalWrite(relayPin, LOW);    // Relay off

    for (int x=0; x<10; x++){
      digitalWrite(buzzPin, HIGH);   // Turn buzzer on
      delay(100);
      digitalWrite(buzzPin, LOW);    // Turn buzzer off
      delay(200);
    }

    beep(500);
    off();
  }

}


void rechargeBattery() {
  //Recharger is pluged

  digitalWrite(powerPin, LOW); // turn power relay off
  digitalWrite(redPin, HIGH);
  beep(200);
  digitalWrite(greenPin, LOW);

  unsigned long startInterval = millis();
  int startBatLevel = analogRead((unsigned char)batPin);

  do {
    if ( millis() - startInterval > checkDeltaBatteryIncreasingVoltageTime) {
      if (analogRead((unsigned char)batPin)-startBatLevel <= 0) { //no inreasing voltage
        //Battery rechareged

        digitalWrite(greenPin, HIGH);
        beep(200);
        // ... and charege further.
        while (1);
      }

      //Start new charging period with new values
      startInterval = millis();
      startBatLevel = analogRead((unsigned char)batPin);
    }
  }  while (1); //forever loop
}



void serialEvent() {
    //if (!eepromLoad) {
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char) Serial.read();
        // add it to the inputString:
        if (inChar != '\r') {
            inputString += inChar;
        }

        Serial.print(inChar); //echo

        // if the incoming character is a newline, set a flag
        if (inChar == '\n') {
            stringComplete = true;
        }

        if (inChar == '@') {
            memComplete = true;
        }
    }
    //}
}

void eepromUpload(int adr) {
    unsigned int i = 0;
    boolean flagCompleted;
    boolean bufferFull = false;
    int endBuffer;
    //eepromLoad = true;

    do {
        //Serial.print(char(XON));
        bufferFull = readSerial2Buffer(endBuffer);
        b = 0; // buffer pointer
        flagCompleted = false;
        while (!flagCompleted) {

            flagCompleted = !(i + adr < PROGRAM_SIZE) || (memBuffer[b] == '@') || !(b < endBuffer);
            if (memBuffer[b] == ';') memBuffer[b] = '\n';   //Semicollon as end line LF (#10) for windows script
            if (memBuffer[b] == '\r') memBuffer[b] = ' '; //#13 -> space, No continue because of changing script length
            EEPROM.write(i + adr, memBuffer[b]);
            //Serial.print(memBuffer[b]);
            i++;
            b++;
        }
        //End of shorter program then PROGRAM_SIZE size

    } while (bufferFull);
    if (i + adr < PROGRAM_SIZE) EEPROM.write(i + adr, '@');
    //eepromLoad=false;
}

boolean readSerial2Buffer(int &endBuffer) {
    int i; //= 0; //buffer indicator
    char c;

    //while(true) {
    for (i = 0; i < PROGRAM_BUFFER; i++) {

        while (!Serial.available());

        c = Serial.read();

        memBuffer[i] = c;
        //Echo
        //Serial.print(c);
        endBuffer = i;

        if (c == '@') {
            beep(30);
            return false;
            //break;
        }

        //i++;
    }

    return true;
}


#endif /* FREEPEMF_FUNC_H_ */
