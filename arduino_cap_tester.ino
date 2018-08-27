/*  RCTiming_capacitance_meter
 *   Paul Badger 2008
 *  Demonstrates use of RC time constants to measure the value of a capacitor 
 *
 * Theory   A capcitor will charge, through a resistor, in one time constant, defined as T seconds where
 *    TC = R * C
 * 
 *    TC = time constant period in seconds
 *    R = resistance in ohms
 *    C = capacitance in farads (1 microfarad (ufd) = .0000001 farad = 10^-6 farads ) 
 *
 *    The capacitor's voltage at one time constant is defined as 63.2% of the charging voltage.
 *
 *  Hardware setup:
 *  Test Capacitor between common point and ground (positive side of an electrolytic capacitor  to common)
 *  Test Resistor between chargePin and common point
 *  220 ohm resistor between dischargePin and common point
 *  Wire between common point and analogPin (A/D input)
 */

/* Libraries needed */
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <FreqCounter.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* set up macros for readability */

#define analogPin      1          // analog pin for measuring capacitor voltage
#define chargePin      12         // pin to charge the capacitor - connected to one end of the charging resistor
#define dischargePin   11         // pin to discharge the capacitor
#define resistorValue  10000.0F   // change this to whatever resistor value you are using
                                  // F formatter tells compliler it's a floating point value
#define pinLed         13         // used to flash the LED on the nano

/* variable Definitions */

unsigned long startTime;
unsigned long elapsedTime;
float microFarads;                // floating point variable to preserve precision, make calculations
float nanoFarads;

unsigned long frq;
int cnt;
unsigned long period;
float input_555;
float cap_val;


void setup()
{
  // Set the LCD address to 0x27 for a 16 chars and 2 line display
 
/* configure Arduino Pins */
  pinMode(pinLed, OUTPUT);        // set led control pin to output
  pinMode(chargePin, OUTPUT);     // set chargePin to output
  digitalWrite(chargePin, LOW);   // initialize charge pin output to low

  Serial.begin(9600);             // initialize serial transmission for debugging

  /* initialize LCD screen */ 
  lcd.begin();
  lcd.backlight();
  lcd.print("Initializing....");
  lcd.setCursor(0,1);
  for (int inx = 0; inx < 16; inx++ ) {
    lcd.write(0xA5);
    delay(250);
  }
  delay(3000);
}


// the u used for capacitance is 0xE4 for the display
// the dot symbol is 0xA5

void loop()
{
   LiquidCrystal_I2C lcd(0x27, 16, 2);
  lcd.begin();
  lcd.clear();
  lcd.print("Ready");
  lcd.setCursor(0,1);
  lcd.print("press button");

  digitalWrite(chargePin, HIGH);  // set chargePin HIGH and capacitor charging
  startTime = millis();

  while(analogRead(analogPin) < 648){       // 647 is 63.2% of 1023, which corresponds to full-scale voltage 
  }

  elapsedTime= millis() - startTime;
 // convert milliseconds to seconds ( 10^-3 ) and Farads to microFarads ( 10^6 ),  net 10^3 (1000)  
  microFarads = ((float)elapsedTime / resistorValue) * 1000;   
  Serial.print(elapsedTime);       // print the value to serial port
  Serial.print(" mS    ");         // print units and carriage return


  if (microFarads > 1){
    Serial.print((long)microFarads);       // print the value to serial port
    Serial.println(" microFarads");         // print units and carriage return
  }
  else
  {
    // if value is smaller than one microFarad, convert to nanoFarads (10^-9 Farad). 
    // This is  a workaround because Serial.print will not print floats

    nanoFarads = microFarads * 1000.0;      // multiply by 1000 to convert to nanoFarads (10^-9 Farads)
    Serial.print((long)nanoFarads);         // print the value to serial port
    Serial.println(" nanoFarads");          // print units and carriage return
  }

  /* dicharge the capacitor  */
  digitalWrite(chargePin, LOW);             // set charge pin to  LOW 
  pinMode(dischargePin, OUTPUT);            // set discharge pin to output 
  digitalWrite(dischargePin, LOW);          // set discharge pin LOW 
  while(analogRead(analogPin) > 0){         // wait until capacitor is completely discharged
  }

}
/* Function defined to specifically calculate the capacitance value using the frequency of the 555 output
 *  
 
 int calc_Freq()
 {
 
  // wait if any serial is going on
  FreqCounter::f_comp=10;   // Cal Value / Calibrate with professional Freq Counter
  FreqCounter::start(1000);  // 1000 ms Gate Time, this seems to work with low frequencies the best

   while (FreqCounter::f_ready == 0) {

      frq=FreqCounter::f_freq;
      // freq = .72/R*C, therefore: C = .72/R*freq
      cap_val = (.72 / (1000 * frq)) *100000;
      //Serial.print(cnt++);
      Serial.print("  Freq: ");
      Serial.print(frq);
      Serial.print("  Cap_val: ");
      Serial.print(cap_val);
      Serial.println(" nF");
      
      
      delay(1000);
      digitalWrite(pinLed,!digitalRead(pinLed));  // blink Led
      lcd.clear();
      lcd.print("Cap: ");
      lcd.print(cap_val);
      lcd.print("  nF");
      lcd.setCursor(0,1);
      lcd.print("Frequency: ");
      lcd.print(frq);

      //period = 2 * pulseIn(5, HIGH);
      //Serial.print("Period: ");
      //Serial.println(period);

   }  
    
 }*/
