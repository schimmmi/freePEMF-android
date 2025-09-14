# freePEMF (Android/Arduino sketch)

Home: biotronics.eu (archived) / biotronika.pl

### How to compile and upload with Arduino IDE
1. Download this folder and keep the name exactly: freePEMF.
2. Open freePEMF.ino in Arduino IDE.
3. Install TimerOne library: Sketch -> Include Library -> Add .ZIP Library... and select the provided ZIP.
4. Ensure EEPROM library is available: Sketch -> Include Library -> verify EEPROM is listed.
5. Select board: Tools -> Board -> Arduino Nano; Processor: ATmega328P.
6. Install the CH341 USB-Serial driver if needed: CH341SER.ZIP (original link on biotronika.pl; use a trusted source or archived copy if unavailable).
7. Select serial port: Tools -> Port -> choose the correct COM port after connecting via USB.
8. Upload: Sketch -> Upload. Wait for “Done uploading”.

### Documentation
- bioZAP script language manual (2018-10-21): the original link was hosted on biotronika.pl. If it is down, use a web archive mirror or the documentation folder in this repo.

### Notes (2018-02-22 release)
New software version includes:
1. PWM signal generation up to 10 kHz
2. 4th standard program for pineal gland activation (15 min)
3. jump and labels support
4. freq function
5. Decimal separator support, e.g., "freq 7.83 180" instead of 783 for Schumann frequency
6. More info: see bioZAP page (archived) on biotronika.pl

See also: freePEMF Android smartphone Bluetooth connection:
https://github.com/biotronika/freePEMF/wiki/Bluetooth-connection
