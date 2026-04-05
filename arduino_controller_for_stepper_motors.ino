/*  AZ/EL Antenna Rotator controller for Arduino 
 *  STEPPER MOTOR Rotator, without feedback
 *  90 degrees elevation
 *  ========================================================
 *  Uses EasyComm II protocol for computer Tracking Software
 *  Manual command by means of two rotary encoders AZ - EL
 *   
 *  Viorel Racoviteanu
 *  https://www.youtube.com/@racov
 *  https://racov.ro
 *  YO3RAK@gmail.com
 *  
 *  I cannot take any responsibility for missuse of this code
 *  or any kind of damage it may occur from using this code.
 * 
 *  nov 2025 - first release
 *
 *  Modifications by Chris (DL6LG), apr 2026:
 *  - Command resolution increased to 0.1° (EasyComm II AZ/EL commands)
 *  - ComAzim / ComElev changed from int to float
 *  - Serial parser extended to accept one decimal place (e.g. AZ180.5)
 *  - Position response uses dtostrf() for true 0.1° output (e.g. +180.4 45.2)
 *  - Encoder logic uses floor()/ceil() to snap to integer degrees
 *  - fmod() replaces % operator for float azimuth wrap-around
 */

#include <Wire.h>                     // Library for I2C communication
#include <AccelStepper.h>
// Motor Connections (unipolar motor driver)
/*
const int In1 = 8;
const int In2 = 9;
const int In3 = 10;
const int In4 = 11;
// Motor Connections (constant voltage bipolar H-bridge motor driver)
const int AIn1 = 8;
const int AIn2 = 9;
const int BIn1 = 10;
const int BIn2 = 11;
*/
// Pin Assignment
#define AzSwLo A3                      // safety switch azimuth zero position
#define AzSwHi A2
#define ElSwLo A1                      // safety switch elevation zero position
#define ElSwHi A0
#define AzEncoderPinA 2               // Az encoder right
#define AzEncoderPinB 4               // Az encoder left
#define AzClearButton 5               // Az encoder push
#define ElEncoderPinA 3               // El encoder right
#define ElEncoderPinB 6               // El encoder left
#define ElClearButton 7               // El encoder pus
// Motor Connections (TB6600 constant current, step/direction bipolar motor driver)
#define AzDirPin 9                    // direction pin
#define AzStepPin 8                   // pulse pin
#define AZ_ENA 12                     // enable pin ENA+ (ENA a GND)
#define ElDirPin 11
#define ElStepPin 10
#define EL_ENA 13                     // ENA+ EL  D13 (ENA a GND)

// Creates an instance - Pick the version you want to use and un-comment it. That's the only required change.
//AccelStepper myStepper(AccelStepper::FULL4WIRE, AIn1, AIn2, BIn1, BIn2);  // works for TB6612 (Bipolar, constant voltage, H-Bridge motor driver)
//AccelStepper AzMotor(AccelStepper::FULL4WIRE, In1, In3, In2, In4);        // works for ULN2003 (Unipolar motor driver)
//AccelStepper ElMotor(AccelStepper::FULL4WIRE, 12, 13, A6, A7);            // works for ULN2003 (Unipolar motor driver)
AccelStepper AzMotor(AccelStepper::DRIVER, AzStepPin, AzDirPin);          // works for TB6600, A4988 (Bipolar, constant current, step/direction driver)
AccelStepper ElMotor(AccelStepper::DRIVER, ElStepPin, ElDirPin);

#include <elapsedMillis.h>
elapsedMillis printTime;              // one second info printout timer.

#include <LiquidCrystal_I2C.h> // https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c (Library for LCD)
// Wiring: SDA pin is connected to A4 and SCL pin to A5.
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
LiquidCrystal_I2C lcd(0x27, 16, 2);   // address, chars, rows.

// declaring custom symbol for up/down arrow
 byte DownArrow[8] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100,
  B00000
};
 byte UpArrow[8] = {
  B00000,
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00000
};

// ========= PARAMETERS TO MODIFY ACCORDING TO YOUR MOTORS =========
  const long AzPulsPerTurn = 8140;    // azimuth step pulses per 360 (depends on microstep settings and gears)
  const long ElPulsPerTurn = 2050 ;   // elevation step pulses per 90
  const int AzPark = 120;             // azimuth and elevation coordinates
  const int ElPark = 0;               // for antenna parking
  const int Speed = 500;              // speed depends on microstepping
  inline float Accel = 0.8*Speed;     // acceleration smaller than speed. The closer value, the faster it accelerates
// =================================================================
  
// Antenna variables
  float TruAzim = 359;                // real position of the antenna azimuth
  float TruElev = 90;
  float ComAzim = 0.0;                // command azimuth, where the antenna should go
  float ComElev = 0.0;
  float OldComAzim = ComAzim;
  float OldComElev = ComElev;
  long AzMovePuls;                    // absolute positionn of the target (in steps)
  long ElMovePuls;
  inline float AzPulsToDeg = 360.0/AzPulsPerTurn;
  inline float ElPulsToDeg = 90.0/ElPulsPerTurn;
  bool Activ = false;                 // ENABLE pin status -> LOW is ENABLE
  char AzDir;                         // symbol for azim rotation display (arrow left or right)
  bool AzMove = true;
  bool ElMove = true;

// variables for serial comm
  String Azimuth = "";
  String Elevation = "";
  String ComputerRead;
  String ComputerWrite;

// Az encoder variables
  bool AzPush = 0;                    // variable to toggle with encoder push button
  volatile boolean AzEncRot;          // if rotation occured
  volatile boolean AzEncUp;           // direction of rotation
  float increment = 1.0;              // degrees to increment at encoder rotation (reserved for future use)
  
// El encoder variables
  bool ElPush = 0;
  volatile boolean ElEncRot;
  volatile boolean ElEncUp;

//========================== END DECLARATIONS ======================


void setup() {
  // pin declaration
  pinMode(AzEncoderPinA, INPUT_PULLUP);
  pinMode(AzEncoderPinB, INPUT_PULLUP);
  pinMode(AzClearButton, INPUT_PULLUP);
  pinMode(ElEncoderPinA, INPUT_PULLUP);
  pinMode(ElEncoderPinB, INPUT_PULLUP);
  pinMode(ElClearButton, INPUT_PULLUP);
  pinMode(AzSwLo, INPUT_PULLUP);
  pinMode(AzSwHi, INPUT_PULLUP);
  pinMode(ElSwLo, INPUT_PULLUP);
  pinMode(ElSwHi, INPUT_PULLUP);
  pinMode(AZ_ENA, OUTPUT);
  pinMode(EL_ENA, OUTPUT);
// Interrupt Service Routine for Az and El encoder
  attachInterrupt(0, doAzEnc, CHANGE);// Az encoder
  attachInterrupt(1, doElEnc, CHANGE);// El Encoder
// start serial port
  Serial.begin(9600);
  Serial.setTimeout(50);              // miliseconds to wait for USB data (default 1000). That's about 50 characters
// Start the LCD:
  lcd.init();
  lcd.backlight();
// creating custom symbol for up/dwn arrow
  lcd.createChar(1, DownArrow);
  lcd.createChar(2, UpArrow);

// write on display name and version
  lcd.setCursor(0, 0);                // Set the cursor on the first column first row.(counting starts at 0!)
  lcd.print("STEPPER EASYCOMM");
  lcd.setCursor(0, 1);                // Set the cursor on the first column the second row
  lcd.print("YO3RAK  NOV.2025");
  delay(2000);                        // wait 2 seconds

// display fixed text
  lcd.setCursor(0, 0);
  lcd.print("Azm.---" + String(char(223)) + "=Cd.---" + String(char(223)));  // char(223) is degree symbol
  lcd.setCursor(0, 1); 
  lcd.print("Elv. --" + String(char(223)) + "=Cd. --" + String(char(223)));
  DisplValue(round(TruAzim), 4,0);
  DisplValue(round(ComAzim),12,0);
  DisplValue(round(TruElev), 4,1);
  DisplValue(round(ComElev),12,1);

// set the maximum speed, acceleration factor
  AzMotor.setMaxSpeed(Speed);
  ElMotor.setMaxSpeed(Speed);
  AzMotor.setAcceleration(Accel);
  ElMotor.setAcceleration(Accel);
  AzMotor.setSpeed(-1*Speed);                 // rotate backwards at boot at constant speed
  ElMotor.setSpeed(-1*Speed);                 // untill it reaches 0 end-stop
// start to move antenna to ZERO at start-up
  AzMotor.setCurrentPosition(AzPulsPerTurn);  // assume antenna is all the way back
  AzMovePuls = 0;                             // set target 0
  ElMotor.setCurrentPosition(ElPulsPerTurn);
  ElMovePuls = 0;
  lcd.setCursor(8, 0);                        // display arrows
  lcd.print(String(char(127)));
  lcd.setCursor(8, 1);
  lcd.write(1);
  AntennaInit();                              // move antenna to zero
}  // end SETUP

/*========================== THE WHOLE CODE TAKES ONLY 16ms TO EXECUTE ======================*/
void loop() {
// read the command from encoder
  ReadAzimEncoder();
  ReadElevEncoder();
// read the command from computer
  if (Serial.available()) {
    SerialCommand();                          // read USB data
  }

// if there is a new target direction
  if (OldComAzim != ComAzim) {
    DisplValue(round(ComAzim),12,0);          // display
    OldComAzim = ComAzim;
    AzMovePuls = ComAzim/AzPulsToDeg;         // absolute position in pulses where motor should go
    AzMove = true;                            // prepare movement
    if (ComAzim > TruAzim) {AzDir = char(126);}   // "->"
    else {AzDir = char(127);}                     // "<-"
    lcd.setCursor(8, 0);
    lcd.print(String(AzDir));
  }
  if (OldComElev != ComElev) {
    DisplValue(round(ComElev),12,1);
    OldComElev = ComElev;
    ElMovePuls = ComElev/ElPulsToDeg;
    ElMove = true;
    if (ComElev > TruElev) {
      lcd.setCursor(8, 1);
      lcd.write(2);                           // arrow up
    }
    else {
      lcd.setCursor(8, 1);
      lcd.write(1);                           // arrow dn
    }
  }
// move the antenna
  if (AzMove || ElMove){                      // if need to move in Azim. OR Elev.
    AntennaMove();
  }
}   // end main LOOP

/**************************************************************************************/
/* =============================== FUNCTION DEFINITIONS ============================= */
/**************************************************************************************/

// _________________ Moving ANTENNA to zero at STARTUP ______________________
void AntennaInit() {
  while (AzMove || ElMove) {                    // as long as we are moving in AZ or EL
    if (printTime >= 1000) {                    // every second
          printTime = 0;
          TruAzim = AzMotor.currentPosition()*AzPulsToDeg;  // calculate antenna position
          TruElev = ElMotor.currentPosition()*ElPulsToDeg;
          DisplValue(round(TruAzim), 4,0);                  // display antenna position
          DisplValue(round(TruElev), 4,1);
          if (Serial.available()) {SerialRequest();}        // read USB to send back the coordinates if asked
        }
    if (AzMove){
      digitalWrite(AZ_ENA, Activ);              // enable the driver
      AzMotor.moveTo(AzMovePuls);               // this set target coordinates
      AzMotor.runSpeed();                       // motor move to target at constant speed (once it reached the target, stops automatically)
      ReadAzSw();                               // verify safety switches
      if (!AzMotor.run()){                      // if motor not move (reached the target)
        AzMove = false;
        digitalWrite(AZ_ENA, !Activ);           // disable the driver
        TruAzim = AzMotor.currentPosition()*AzPulsToDeg;    // final position
        DisplValue(round(TruAzim), 4,0);
        lcd.setCursor(8, 0);
        lcd.print("=");
      }
    }

    if (ElMove){
      digitalWrite(EL_ENA, Activ);
      ElMotor.moveTo(ElMovePuls);
      ElMotor.runSpeed();
      ReadElSw();
      if (!ElMotor.run()){
        ElMove = false;
        digitalWrite(EL_ENA, !Activ);
        TruElev = ElMotor.currentPosition()*ElPulsToDeg;
        DisplValue(round(TruElev), 4,1);
        lcd.setCursor(8, 1);
        lcd.print("=");
      }
    }
  }
} // end AntennaInit()

// _________________ START A MOVE ______________________
void AntennaMove() {
  while (AzMove || ElMove) {                    // as long as we are moving in AZ or EL
    if (printTime >= 1000) {                    // every second
          printTime = 0;
          TruAzim = AzMotor.currentPosition()*AzPulsToDeg;  // calculate antenna position
          TruElev = ElMotor.currentPosition()*ElPulsToDeg;
          DisplValue(round(TruAzim), 4,0);                  // display antenna position
          DisplValue(round(TruElev), 4,1);
          if (Serial.available()) {SerialRequest();}        // read USB to send back the coordinates if asked
        }
    if (AzMove){
      digitalWrite(AZ_ENA, Activ);              // enable the driver
      AzMotor.moveTo(AzMovePuls);               // this set target coordinates
      AzMotor.run();                            // this makes motor move to the target (once it reached the target, stops automatically)
      ReadAzSw();                               // verify safety switches
      if (!AzMotor.run()){                      // if motor not move (reached the target)
        AzMove = false;
        digitalWrite(AZ_ENA, !Activ);           // disable the driver
        TruAzim = AzMotor.currentPosition()*AzPulsToDeg;    // final position
        DisplValue(round(TruAzim), 4,0);
        lcd.setCursor(8, 0);
        lcd.print("=");
      }
    }

    if (ElMove){
      digitalWrite(EL_ENA, Activ);
      ElMotor.moveTo(ElMovePuls);
      ElMotor.run();
      ReadElSw();
      if (!ElMotor.run()){
        ElMove = false;
        digitalWrite(EL_ENA, !Activ);
        TruElev = ElMotor.currentPosition()*ElPulsToDeg;
        DisplValue(round(TruElev), 4,1);
        lcd.setCursor(8, 1);
        lcd.print("=");
      }
    }
  }
} // end AntennaMove()

void DisplValue(int x, int y, int z) {
  char displayString[17] = "";
  sprintf(displayString, "%3d", x);         // outputs a fixed lenght number (3 integer)
  lcd.setCursor(y, z);                      // for leading zeros '007' use "%03d"
  lcd.print(displayString);
}  // end DisplValue()

// Interrupt Service Routine for a change to encoder pin A and B
void doAzEnc () {
  if (digitalRead (AzEncoderPinA))
    AzEncUp = digitalRead (AzEncoderPinB);
  else
    AzEncUp = !digitalRead (AzEncoderPinB);
    AzEncRot = true;
}  // end doAzEnc

void doElEnc () {
  if (digitalRead (ElEncoderPinA))
    ElEncUp = digitalRead (ElEncoderPinB);
  else
    ElEncUp = !digitalRead (ElEncoderPinB);
    ElEncRot = true;
}  // end doElEnc

void ReadAzimEncoder() {
  if (digitalRead(AzClearButton) == LOW ) {       // if encoder switch depressed
    AzPush = true;                                // we have a manual input
// refresh all the writings on the screen with "*" symbol
    lcd.setCursor(0, 0);
    lcd.print("Azm.   " + String(char(223)) + "*Cd.   " + String(char(223)));  // char(223) is degree symbol
    lcd.setCursor(0, 1); 
    lcd.print("Elv.   " + String(char(223)) + "=Cd.   " + String(char(223)));
    DisplValue(round(TruAzim), 4,0);
    DisplValue(round(ComAzim),12,0);
    DisplValue(round(TruElev), 4,1);
    DisplValue(round(ComElev),12,1);
    delay(500);                                   // debounce
  }
  while (AzPush){
// read AZ encoder rotation
    if (AzEncRot){
      delay(20);                                  // debouncing
      if (AzEncUp) ComAzim = floor(ComAzim) + 1.0; // round to next integer upwards
      else         ComAzim = ceil(ComAzim)  - 1.0; // round to next integer downwards
      ComAzim = fmod((ComAzim + 360.0), 360.0);   // Az Cmd between 0 and 359 deg continuous
      ComAzim = constrain(ComAzim, 0.0, 359.0);
      AzEncRot = false;
      DisplValue(round(ComAzim),12,0);
    }
// read EL encoder rotation
    if (ElEncRot){
      delay(20);
      if (ElEncUp) ComElev = floor(ComElev) + 1.0; // round to next integer upwards
      else         ComElev = ceil(ComElev)  - 1.0; // round to next integer downwards
      ElEncRot = false;
      ComElev = constrain(ComElev, 0.0, 90.0);    // keep El Cmd value in range
      DisplValue(round(ComElev),12,1);
    }
    if (digitalRead(AzClearButton) == LOW ) {       // if encoder switch depressed
      AzPush = false;                               // we exit manual input
      lcd.setCursor(8, 0);
      lcd.print("-");
      delay(500);   
    }
  }
} //end ReadAzimEncoder()

void ReadElevEncoder() {
  if (digitalRead(ElClearButton) == LOW ) {       // set Park Position Command Az/El
    ComAzim = AzPark;                             // set parking position
    ComElev = ElPark;
    DisplValue(round(ComAzim),12,0);
    DisplValue(round(ComElev),12,1);
    lcd.setCursor(8, 1);
    lcd.print("%");                               // display P symbol for parking
    delay (2000);
  }
}  // end ReadElevEncoder()

//____________________VERIFY SAFETY SWITCHES____________________
void ReadAzSw() {
  bool a = digitalRead(AzSwLo);
  if (!a) {                                       // the antenna hit zero position
    AzMotor.setCurrentPosition(0);
    TruAzim = 0;
// if the antenna was offset and the switch was hit unexpectedly,
// this will send the antenna back to target
    OldComAzim = 0;
    lcd.setCursor(7, 0);
    lcd.print(String(char(223)));
  }
  bool b = digitalRead(AzSwHi);
  if (!b) {                                       // the antenna hit full turn
    AzMotor.setCurrentPosition(AzPulsPerTurn);
    TruAzim = 359;
    OldComAzim = 359;
    lcd.setCursor(7, 0);
    lcd.print(String(char(223)));
  }
} // end ReadAzSw()
void ReadElSw() {
  bool a = digitalRead(ElSwLo);
  if (!a) {
    ElMotor.setCurrentPosition(0);
    TruElev = 0;
    OldComElev = 0;
    lcd.setCursor(7, 1);
    lcd.print(String(char(223)));
  }
  bool b = digitalRead(ElSwHi);
  if (!b) {
    ElMotor.setCurrentPosition(ElPulsPerTurn);
    TruElev = 90;
    OldComElev = 90;
    lcd.setCursor(7, 1);
    lcd.print(String(char(223)));
  }
} // end ReadElSw()

//________ SERIAL  COMMUNICATION - receiving a command and answering a position Request ________
void SerialCommand() {
  // initialize readings
  ComputerRead = "";
  Azimuth = "";
  Elevation = "";

  while(Serial.available()) {
    ComputerRead= Serial.readString();           // read the incoming data as string
//    Serial.println(ComputerRead);              // echo the reception for testing purposes
  }
  
// looking for command <AZxxx.x>
    for (int i = 0; i <= ComputerRead.length(); i++) {
     if ((ComputerRead.charAt(i) == 'A')&&(ComputerRead.charAt(i+1) == 'Z')){ // if read AZ
      bool azDotSeen = false;
      for (int j = i+2; j <= ComputerRead.length(); j++) {
        char c = ComputerRead.charAt(j);
        if (isDigit(c)) {                                                     // if the character is number
          Azimuth = Azimuth + c;
        } else if (c == '.' && !azDotSeen) {                                  // accept one decimal point
          azDotSeen = true;
          Azimuth = Azimuth + c;
        } else {break;}
      }
     }
    }
    
// looking for command <ELxxx.x>
    for (int i = 0; i <= (ComputerRead.length()-2); i++) {
      if ((ComputerRead.charAt(i) == 'E')&&(ComputerRead.charAt(i+1) == 'L')){ // if read EL
        if ((ComputerRead.charAt(i+2)) == '-') {                               // if received elevation negative
          ComElev = 0;                                                         // keep antenna to zero
          break;
        }
        bool elDotSeen = false;
        for (int j = i+2; j <= ComputerRead.length(); j++) {
          char c = ComputerRead.charAt(j);
          if (isDigit(c)) {                                                    // if the character is number
            Elevation = Elevation + c;
          } else if (c == '.' && !elDotSeen) {                                 // accept one decimal point
            elDotSeen = true;
            Elevation = Elevation + c;
          } else {break;}
        }
      }
    }
    
// if <AZxx> received
    if (Azimuth != ""){
      ComAzim = Azimuth.toFloat();
      ComAzim = constrain(ComAzim, 0.0, 359.0); // keeping values in range
    }

// if <ELxx> received
    if (Elevation != ""){
      ComElev = Elevation.toFloat();
      ComElev = constrain(ComElev, 0.0, 90.0); // we won't tolerate no funny elevation values
    }
    
// looking for <AZ EL> interogation for antenna position
  for (int i = 0; i <= (ComputerRead.length()-4); i++) {
    if ((ComputerRead.charAt(i) == 'A')&&(ComputerRead.charAt(i+1) == 'Z')&&(ComputerRead.charAt(i+3) == 'E')&&(ComputerRead.charAt(i+4) == 'L')){
    // send back the antenna position <+xxx.x xx.x>
      char azBuf[8], elBuf[6];
      dtostrf(round(TruAzim * 10.0) / 10.0, 5, 1, azBuf);  // e.g. "180.4"
      dtostrf(round(TruElev * 10.0) / 10.0, 4, 1, elBuf);  // e.g. "45.2"
      ComputerWrite = "+" + String(azBuf) + " " + String(elBuf);
 //ComputerWrite = "AZ"+String(TruAzim)+".0 EL"+String(TruElev)+".0"; //that's for Gpredict and HamLib
      Serial.println(ComputerWrite);
    }
  }
}  // end SerialCommand()

//________ SERIAL  COMMUNICATION - answering a position Request ________
void SerialRequest() {
  ComputerRead = "";
  while(Serial.available()) {ComputerRead = Serial.readString();}     // read the incoming data as string
// looking for <AZ EL> interogation for antenna position
  for (int i = 0; i <= (ComputerRead.length()-4); i++) {
    if ((ComputerRead.charAt(i) == 'A')&&(ComputerRead.charAt(i+1) == 'Z')&&(ComputerRead.charAt(i+3) == 'E')&&(ComputerRead.charAt(i+4) == 'L')){
    // send back the antenna position <+xxx.x xx.x>
      char azBuf[8], elBuf[6];
      dtostrf(round(TruAzim * 10.0) / 10.0, 5, 1, azBuf);  // e.g. "180.4"
      dtostrf(round(TruElev * 10.0) / 10.0, 4, 1, elBuf);  // e.g. "45.2"
      ComputerWrite = "+" + String(azBuf) + " " + String(elBuf);
//ComputerWrite = "AZ"+String(TruAzim)+".0 EL"+String(TruElev)+".0"; //that's for Gpredict and HamLib
      Serial.println(ComputerWrite);
    }
  }
}  // end SerialRequest()
