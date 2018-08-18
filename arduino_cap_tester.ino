/**
 * Displays text sent over the serial port (e.g. from the Serial Monitor) on
 * an attached LCD.
 */
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <FreqCounter.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long frq;
int cnt;
int pinLed=13;


void setup()
{
  pinMode(pinLed, OUTPUT);
  
  lcd.begin();
  lcd.backlight();
  lcd.print("Initializing....");
  lcd.setCursor(0,1);
  for (int inx = 0; inx < 16; inx++ ) {
    lcd.write(0xA5);
    delay(250);
  }
  delay(4000);
  // Initialize the serial port at a speed of 9600 baud
  Serial.begin(9600);
  Serial.println("Frequency Counter");
}


// the u used for capacitance is 0xE4 for the display
// the dot symbol is 0xA5

void loop()
{
  lcd.clear();
  lcd.print("Ready");
  lcd.setCursor(0,1);
  lcd.print("press button");

  // wait if any serial is going on
  FreqCounter::f_comp=10;   // Cal Value / Calibrate with professional Freq Counter
  FreqCounter::start(1000);  // 1000 ms Gate Time, this seems to work with low frequencies the best

   while (FreqCounter::f_ready == 0) {

      frq=FreqCounter::f_freq;
      Serial.print(cnt++);
      Serial.print("  Freq: ");
      Serial.println(frq);
      delay(20);
      digitalWrite(pinLed,!digitalRead(pinLed));  // blink Led

   }
/*
  // If characters arrived over the serial port...
  if (Serial.available()) {
    // Wait a bit for the entire message to arrive
    delay(100);
    // Clear the screen
    lcd.clear();

    // Write all characters received with the serial port to the LCD.
    while (Serial.available() > 0) {
      lcd.write(Serial.read());
      delay(1000);
    } 
  } */
}
