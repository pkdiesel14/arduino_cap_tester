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
#define chargePin      12         // pin to charge the capacitor - connected to one end of the 100M charging resistor
#define sysVoltage     7          // monitor system voltage to use for more accurate calculations
#define mediumR        3          // This switches 100k in series with the cap for medium size capacitors
#define smallR         2          // this switches a 1k in series with the capacitor for large capacitors
#define dischargePin   11         // pin to discharge the capacitor
//#define resistorValue  100000000.0F // this is the base resistor value for the smallest capacitors (100M on order, using 1M for now)
                                  // F formatter tells compliler it's a floating point value
#define medCapResistor 100000.0F  // use a 100k resistor for capacitors in the medium range
#define largeCapResistor 1000.0F  // use a 1k resistor for the largest caps
#define pinLed         13         // used to flash the LED on the nano

/* variable Definitions */

unsigned long startTime;
unsigned long elapsedTime;
float microFarads;                // floating point variable to preserve precision, make calculations
float microFaradsLast = 0.0;      // floating point variable to hold the last calculated value
float nanoFarads;
float picoFarads = 0.0;
float nanoFaradsLast = 0.0;
float A = 0.5;    // constant used for Low Pass Filter => p(n) = A * p(n-1) + (1-A)p(n)
int inx=0;
unsigned long frq;
float cap_val;
float tau, calculateTau(float resistor);
long readVcc();
float resistor = 105000000.0;
// float resistor = 100000000.0;


void setup()
{
  // Set the LCD address to 0x27 for a 16 chars and 2 line display
 
/* configure Arduino Pins */
  pinMode(pinLed, OUTPUT);        // set led control pin to output
  pinMode(chargePin, OUTPUT);     // set chargePin to output
  pinMode(smallR, OUTPUT);      // this output is used to select the smallest R for the largest C
  digitalWrite(smallR, LOW);      // Initialize to keep small R transistor off
  pinMode(mediumR, OUTPUT);      // this output is used to select the Medium R for the medium C
  digitalWrite(mediumR, LOW);     // Initialize to keep medium R transistor off
  digitalWrite(chargePin, LOW);   // initialize charge pin output to low

  Serial.begin(9600);             // initialize serial transmission for debugging

  /* initialize LCD screen */ 
  lcd.begin();
  lcd.backlight();
  lcd.print("Initializing....");
  lcd.setCursor(0,1);
  for (int inx = 0; inx < 16; inx++ ) {
    lcd.write(0xA5);
    delay(100);
  }
  lcd.clear();
  lcd.print("Ready");
  lcd.setCursor(0,1);
  lcd.print("Connect Cap...");
  delay(1500);
  lcd.clear();
}

// the u used for capacitance is 0xE4 for the display
// the dot symbol is 0xA5

void loop()
{
  LiquidCrystal_I2C lcd(0x27, 16, 2);
  lcd.begin();

  tau = calculateTau(&resistor);         // call the funtion to calcuate tau which is the same as R*C

  // convert tau milliseconds to seconds ( 10^-3 ) and Farads to microFarads ( 10^6 ),  net 10^3 (1000)  
  // and run thru low pass filter: microFaradFiltered = A*microFaradLast + (1-A)microFarad  where A<1
  //modify to try using micros() instead of millis()
  
   microFarads = ((float)tau / resistor) * 1;   
   microFarads = A*microFaradsLast + (1-A)*microFarads;
   microFaradsLast = microFarads;
   Serial.print(tau);          // print the value to serial port
   Serial.print(" uS    ");    // print units and carriage return
   Serial.print(" Resistor Value is:  ");
   Serial.println(resistor);
   


  if (microFarads > 1){
    Serial.print((float)microFarads);       // print the value to serial port
    Serial.println(" microFarads");         // print units and carriage return
    
    lcd.print("Capacitance:");
    lcd.setCursor(0,1);
    lcd.print((float)microFarads);
    lcd.print(" ");
    lcd.write(0xE4);
    lcd.print("F");
  }
  else if ((microFarads*1000) < 1  ){
    // we are dealing with picoFarads now
    picoFarads = microFarads * 1000000.0;      // multiply by 1,000,000 to convert to picoFarads (10^-12 Farads)
    Serial.print((float)picoFarads);         // print the value to serial port
    Serial.println(" picoFarads");          // print units and carriage return
    lcd.print("Capacitance:");
    lcd.setCursor(0,1);
    lcd.print((float)picoFarads);
    lcd.print(" pF"); 
  }
  else
  {
    // if value is smaller than one microFarad, convert to nanoFarads (10^-9 Farad). 
    // This is  a workaround because Serial.print will not print floats

    nanoFarads = microFarads * 1000.0;      // multiply by 1000 to convert to nanoFarads (10^-9 Farads)
    Serial.print((float)nanoFarads);         // print the value to serial port
    Serial.println(" nanoFarads");          // print units and carriage return
    lcd.print("Capacitance:");
    lcd.setCursor(0,1);
    lcd.print((float)nanoFarads);
    lcd.print(" nF"); 
  }


  
   
   Serial.print("Input Voltage: ");       // print the value to serial port
   Serial.println(analogRead(analogPin));         // print units and carriage return
}



float calculateTau(float *resistor) {

  int inx = 0;
  int result = 0;
  int threedbPt = 565;  // 647 is 63.2% of 1023, which corresponds to full-scale voltage, need to fudge this number to tune the HW
                        // adjust this lower when accounting
                        // for a diode drop across the transistor when a lower pullup is selected
  int threedbPtLo = 565-130; 

  while(result == 0){
    
      inx = 0;          // need to initialize the loop escape timer each time
       
      /* dicharge the capacitor  */
      digitalWrite(chargePin, LOW);             // set charge pin to  LOW to ensure charging stopped
      pinMode(dischargePin, OUTPUT);            // set discharge pin to output 
      digitalWrite(dischargePin, LOW);          // set discharge pin LOW 

      Serial.println();
      Serial.print("System Voltage:  ");
      Serial.println(readVcc());
      Serial.print("3dB Point is:  ");
      Serial.println(threedbPt);
      Serial.print("Discharging Cap, Cap voltage is: ");
      Serial.println(analogRead(analogPin));
      
      while(analogRead(analogPin) > 2){         // wait until capacitor is completely discharged

      }
  
     // digitalWrite(dischargePin, HIGH); // the discharge pin needs to be forced HIGH to stop the discharge
     pinMode(dischargePin, INPUT);     // the discharge pin needs to be configured as a high impedance input before
                                       // charging starts

      Serial.print("Initial Analog voltage:  " );
      Serial.println(analogRead(analogPin));
  

      digitalWrite(chargePin, HIGH);  // set chargePin HIGH and capacitor begins charging
      startTime = micros();
      
      while(analogRead(analogPin) < threedbPt && inx < 10000){       
        inx++;
      }

      elapsedTime= micros() - startTime;
      Serial.print("Inx is: ");
      Serial.print(inx);
      Serial.print("  Analog voltage is: ");
      Serial.print(analogRead(analogPin));
      Serial.print("  Elapsed Time is: ");
      Serial.println(elapsedTime);
      if(inx >= 9999){
         // threedbPt = threedbPt - (int)(0.5/(readVcc()*.001/1023));  //adjust 3db down 0.6V for diode drop across NPN
         threedbPt = threedbPtLo;  // trying to hard code this due to some inconsistencies found while debugging
         //Serial.print((int)(0.6/(readVcc()*.001/1023)));
         Serial.print(threedbPt);
         Serial.print("  Elapsed time is too large: ");
         Serial.println(elapsedTime);
         Serial.print("   Selecting a new charge resistor");
         Serial.print("  New 3dB Point is: ");
         Serial.println(threedbPt);
         if (!digitalRead(mediumR)) {  // if the medium resisitor is not selected
             digitalWrite(mediumR, HIGH);
             *resistor = 100000.0;
         }
         else {  // you hit this code if the large R and meduim R has not settle in on a cap value because R*C is still too high
             digitalWrite(mediumR, LOW);
             digitalWrite(smallR, HIGH);
             *resistor = 1000.0;
         }
      }
      else {
        result = 1;
      }

  }
  
  return(elapsedTime);

}
 

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
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
