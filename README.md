# freePEMF

See: [biotronika.pl](https://biotronika.pl)

### To compile code and upload using Arduino IDE:
1. Download files and put all of them into freePEMF folder (it must has exactly that name). 
2. Open freePEMF.ino file in Arduino IDE.
3. Install TimerOne library in Arduino (Sketch->Include Library->Add ZIP. Library..., and point at proper zip file).
4. Check if you have EEPROM libraries already installed (Sketch->Include Library-> see on list: EEPROM).
5. Configure board (Tools->Board->Arduino Nano)  (Tools->Processor->ATmega328).
6. Install Arduino Nano driver - **biotronika.pl** website: [CH341SER.ZIP]( https://biotronika.pl/sites/default/files/2016-12/CH341SER.ZIP).
7. Configure serial port. Plug USB cable to PC and freePEMF, and Tolls->Port->select right COM port.
8. Compile and upload. Sketch->Upload. Wait until on down side of Arduino IDE window see **Done uploading**.

### 2018-02-22
New software version includes:
1. PWM generating signal up to 10kHz
2. 4th standard program for a pineal gland activation (15min.)
3. jump and labels support
4. freq function
5. Decimal char suport eg. **freq 7.83 180** insted of 783 for Schumann frequency
6. More? See on page: https://biotronika.pl/en/bioZAP in a while.