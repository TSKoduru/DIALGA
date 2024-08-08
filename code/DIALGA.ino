// TODO
// 1. Test the code on wokwi
// 2. Test the code on the actual hardware
// 3. Order PCB and get cooking


//----------- LED SETTINGS -----------
const int NUM_LEDS = 16; // Number of LEDs in the matrix
const int ROWS[4] = {0, 1, 2, 3};    // pins for rows A, B, C, & D
const int COLS[5] = {4, 5, 6, 7, 8};   // pins for cols 1, 2, 3, 4, & 6
const int NUM_ROWS = 4;
const int NUM_COLS = 4;
int delay_time = 1000;    // Delay between consecutive LED rows

//----------- SLEEP SETTINGS ----------
#include "LowPower.h"  

const unsigned long LENGTH_SCREEN_ON = 20 * 1000;   // Time (s) before sleeping
unsigned long sleep_timer = 0;   // How long the watch has been awake
volatile bool woke_up = false;  // Signals waking up state (Get current time)
volatile unsigned long wakeup_time = 0;   // When the on/off was triggered
bool awake = false;            // Signals awake state (Listen for buttons)
const int ON_OFF_BUTTON = 9;  
const int STATE_BUTTON = 10; 

//-----------TIME SETTINGS -----------
#include <Wire.h>       //i2c library
#include "RTClib.h" 
#include <EEPROM.h>     // To save time to memory
  
enum Mode {TIME, DATE, TEMP};        
int mode = Mode::TIME; // Current mode
int on_off_presses = 0;    // Number of button presses since awakening
int last_on_off_press = 0;    // Last time the button was pressed
int state_presses = 0;    // Number of state button presses
int last_state_press = 0;    // Last time the state button was pressed
const int PRESSES_TO_SET_TIME = 5;    // Number of presses to set time
RTC_DS3231 rtc;          // Attached RTC
DateTime now;

//----------- BATTERY -----------
int current_battery = -1;    // Current battery level (-1 means not polled)
const int CRITICAL_BATTERY = 20;    // Critical battery level (percent)
const int BATTERY_VOLTAGE_PIN = A0;    // Analog pin for battery level

//----------- TEMPERATURE -----------
#include "ChipTemp.h"   // Library for internal temperature sensor
ChipTemp chipTemp;    // Internal temperature sensor

// ---------- OTHER -----------------
#include <avr/interrupt.h>    
#include <Arduino.h>

// -------- LED HELPER FUNCTIONS -----------

void resetCol(int col){

    for(int i = 0; i < NUM_ROWS; i++){
        digitalWrite(ROWS[i], LOW);
    }

    digitalWrite(COLS[col], HIGH);
}

void decimalToBinary(int* binary, int numToConvert){
    for(int i = 0; i < 4; i++){
        binary[NUM_ROWS - i - 1] = numToConvert % 2;
        numToConvert = numToConvert / 2;
    }
}

void setCol(int col, int numToSet){

    // Convert toSet to binary, then display corresponding rows
    // But first turn on the column

    digitalWrite(COLS[col], LOW); // Columns are set LOW since they're GND.

    // Convert to binary, store results
    int binary[4] = {0, 0, 0, 0};
    decimalToBinary(binary, numToSet);

    // Now we can just iterate through the rows and set them accordingly.
    for(int i = 0; i < 4; i++){
        if(binary[i] == 1){
            pinMode(ROWS[i], OUTPUT);
            digitalWrite(ROWS[i], HIGH);
        }
    }
}

// -------- SLEEP HELPER FUNCTIONS -----------

void wake()   // Wake up the system when the ON/OFF is pressed.
{
  woke_up = true;    //flags the microcontroller as just woken up
  wakeup_time = millis();     // Save initial interrupt time
}

void wake_up_procedure(){

  // Toggle awake mode, which starts up LEDs
  //awake = true;

  // Store current time in now (Poll RTC)
  now = rtc.now();

  // Reset modes to default
  mode = Mode::TIME;

  // Start the sleep timer
  sleep_timer = millis();

  // Increment button presses
  on_off_presses++;

}

void all_leds_off(){
    for(int i = 0; i < NUM_ROWS; i++){
        digitalWrite(ROWS[i], LOW);
    }

    for(int i = 0; i < NUM_COLS; i++){
        digitalWrite(COLS[i], LOW);
    }
}

// -------- TIME/DATE/TEMP DISPLAY -----------

void show_time() {

  // Begin by turning the time status LED on (5th col, 1st row)
  digitalWrite(COLS[4], LOW);
  digitalWrite(ROWS[0], HIGH);
  delay(1000);
  
  // The current time is stored in the now variable
  // We did this when we woke up. So, now, we just need to display it.

  // Hour first/second digit, minute first/second digit
  const int time[4] = {now.hour() % 10, now.hour() / 10, 
                    now.minute() % 10, now.minute() / 10};
  
  // Loop through the time array and display the time
  for(int i = 0; i < 4; i++){
    setCol(i, time[i]);
    delayMicroseconds(delay_time);
    resetCol(i);
  }

  delay(1000);
  // Turn off the time status LED
  digitalWrite(COLS[4], HIGH);
  digitalWrite(ROWS[0], LOW);
  
}

void show_date() {

  // Same process as show_time, but with the date

  // Toggle date status LED (5th col, 2nd row)
  digitalWrite(COLS[4], LOW);
  digitalWrite(ROWS[1], HIGH);
  delay(1000);

  // Day first/second digit, month first/second digit
  const int date[4] = {now.day() % 10, now.day() / 10, 
                     now.month() % 10, now.month() / 10};

  // Loop through the date array and display the date
  for(int i = 0; i < 4; i++){
    setCol(i, date[i]);
    delayMicroseconds(delay_time);
    resetCol(i);
  }

  // Turn off the date status LED
  delay(1000);
  digitalWrite(COLS[4], HIGH);
  digitalWrite(ROWS[1], LOW);
}

void show_temp() {
  
  // Different process here, we need to poll the internal temperature sensor
  // on the chip. Before that, we still need to toggle the status LED.

  // Toggle temp status LED (5th col, 3rd row)
  delay(1000);
  digitalWrite(COLS[4], LOW);
  digitalWrite(ROWS[2], HIGH);

  float F = chipTemp.fahrenheit();  // Get the temperature in farenheit
  const int temp[4] = {int(F) % 10, int(F) / 10, 0, 0}; // Only two digits

  // Loop through the temp array and display the temp
  for(int i = 0; i < 2; i++){
    setCol(i, temp[i]);
    delayMicroseconds(delay_time);
    resetCol(i);
  }

  // Turn off the temp status LED
  delay(1000);
  digitalWrite(COLS[4], HIGH);
  digitalWrite(ROWS[2], LOW);
}

// -------- OTHER SETTINGS -----------
void get_battery() {

  // For this, we need to use the analog pin to calculate the battery level
  // We utilize the voltage divider we set up in the PCB.
  // Since R1 is 470K Ohm, R2 is 180K Ohm, and the coin cell is 3.3V, 
  //the voltage divider should output 3.3 * (180K)/(180K + 470K) = 0.914V


  // First, read in the voltage.
  // ADCs work by using the formula: (Vin/Vref) * 1023
  // The internal reference voltage for the atmega328AU is 1.1V
  // So, we can get voltage by (adcValue * (1.1 / 1023.0))

  int adcValue = analogRead(BATTERY_VOLTAGE_PIN); // This is between 0 & 1023
  float adcVoltage = adcValue * (1.1 / 1023.0); 

  // Figure out the battery voltage by reversing the voltage divider formula
  // 650 comes from adding the 180K and 470K resistors
  float batteryVoltage = adcVoltage * (650.0 / 180.0); 

  // Finally, we can calculate the battery percentage
  // We assume the battery is dead at 2.0V and full at 3.3V.
  float batteryPercentage = (batteryVoltage - 2.0) / (3.3 - 2.0) * 100;
  batteryPercentage = constrain(batteryPercentage, 0, 100);

  // Store the battery level
  current_battery = int(batteryPercentage);
}

void set_time() {
  // Allow the user to set the time using the LED matrix.
  // State button increments time by one, on/off button confirms.

  // We're going digit by digit, starting with the year.
  int time[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

  int currentDigit = 0;
  int currentCol = 0;
  bool go_on = true; // Flag to keep the loop going till a button press

  // Start with the year (First four digits)
  while (time[11] == -1 && go_on) {

    // Pause loop till we get a user input
    go_on = false;

    // Set the current digit
    setCol(currentCol, currentDigit);

    // Monitor the state button
    // If it's pressed, increment the current digit
    if(digitalRead(STATE_BUTTON) == HIGH &&
       millis() - last_state_press > 100) {

      currentDigit = (currentDigit + 1) % 9;
      last_state_press = millis();
      go_on = true;
    }

    // Monitor the on/off button
    // If it's pressed, set the current digit and reset to 0
    if(digitalRead(ON_OFF_BUTTON) == HIGH &&
       millis() - last_on_off_press > 100) {
      time[currentCol] = currentDigit;
      currentCol++;
      currentDigit = 0;
      last_on_off_press = millis();
    }
  }

  // Now that we have all the digits, we can set the time
  
  rtc.adjust(DateTime(time[0]*1000 + time[1]*100 + time[2]*10 + time[3], // Year
                      time[4]*10 + time[5], time[6]*10 + time[7],  // Month, Day
                      time[8]*10 + time[9], time[10]*10 + time[11])); // Hr, min

  // Reset the mode to time
  mode = Mode::TIME;

  // Reset the sleep timer
  sleep_timer = millis();

}

// -------- MAIN LOOP SUBROUTINES -----------

void set_current_mode() {
    switch(mode) {
      case Mode::TIME:
        show_time();
        break;
      case Mode::DATE:
        show_date();
        break;
      case Mode::TEMP:
        show_temp();
        break;
    }
}

void monitor_state_button() {

  if(digitalRead(STATE_BUTTON) == HIGH && millis() - last_state_press > 100) {
      
      state_presses++;

      // Either change mode or set time, depending on the number of presses
      if (state_presses == PRESSES_TO_SET_TIME) {set_time();} 
      else mode = (mode + 1) % 3;

      // Update relevant variables
      last_state_press = millis();
      sleep_timer = millis();
    }
}

void monitor_on_off_button() {
  if(digitalRead(ON_OFF_BUTTON) == HIGH &&
       millis() - last_on_off_press > 100) {
      awake = false;
      last_on_off_press = millis();
    }
}

void sleep_routine() {

  // Turn off LED grid
  all_leds_off();
  
  // Reset battery counts
  on_off_presses = 0;
  state_presses = 0;

  // Indicate that battery level hasn't been polled
  current_battery = -1;

  // Enter deep sleep mode to save battery
  delay(30);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

// ----------- MAIN CODE ------------
void setup() {
  
  // MUTIPLEX ARRAY
  // ---------------

    for(int r: ROWS){ // Rows are LOW when off
      pinMode(r, OUTPUT);
      digitalWrite(r, LOW); 
    }

    for(int c: COLS){ // Cols are HIGH when off
      pinMode(c, OUTPUT);
      digitalWrite(c, HIGH);
    }

  // SWITCHES
  // ---------------

  pinMode(ON_OFF_BUTTON, INPUT); // On/off
  pinMode(STATE_BUTTON, INPUT); // Set mode/date

  // RTC
  // ---------------

  // Uncomment the following line and update the date to set time.
  //rtc.adjust(DateTime(2024, 7, 25, 2, 2, 0));

  // INTERRUPTS
  // Note: Taken from https://arduino.stackexchange.com/questions/79690/assign-an-interrupt-to-any-pin-of-the-atmega328-microcontroller
  // ---------------
  
  // We need to enable pin interrupts for PCINT0, to detect the on/off switch.
  cli(); // Disable global interrupts
  PCICR |= 0b00000001; // Enable port B interrupts (Contains PCINT1, the ON/OFF)
  PCMSK0 |= 0b00000010; // Enable PCINT0 (Pin 9, the ON/OFF)
  sei(); // Enable global interrupts

  // Assign the interrupt service routine
  // This will only detect input from the on/off switch because
  // that's the only one we enabled.
  ISR(PCINT0_vect) {wake();} 

  // INTERNAL PULLUPS
  // ---------------

  // We don't use ADC6, ADC7, ADC1, ADC2, ADC3, PCINT6, or PCINT7
  // So we can enable the internal pullups for these pins

  pinMode(A1, INPUT_PULLUP); // ADC1
  pinMode(A2, INPUT_PULLUP); // ADC2
  pinMode(A3, INPUT_PULLUP); // ADC3
  pinMode(20, INPUT_PULLUP); // PCINT6
  pinMode(21, INPUT_PULLUP); // PCINT7

  // Note that ADC6/7 don't have arduino pin numbers, 
  // so we just leave them as is.
}

void loop() {

  // State 1: Just woke up
  // Make sure to debounce the button

  if(woke_up && millis() - wakeup_time > 100) {

      woke_up = false;  // Reset the flag (Happens regardless of button)

      // If button is stil pressed, wake up
      if(digitalRead(ON_OFF_BUTTON) == HIGH) {
        wake_up_procedure();
      }
    }
  


  // State 2: Currently awake
  // No deboucing necessary
  
  if (awake) {

    // Run a check on the battery, and indicate if it's dropped 
    // below a certain level.
    // This should only be run once per awake cycle, to save battery.

    if(current_battery == -1) {get_battery();}
    else if (current_battery < CRITICAL_BATTERY) {
      // Indicate low battery by toggling C5 R4
      // This will not turn off until we go into sleep.
      digitalWrite(COLS[4], LOW);
      digitalWrite(ROWS[3], HIGH);
    }

    // First, update what we're going to be displaying in case 
    // the mode has changed since the last run of the loop.
    set_current_mode();

    // Monitor state button to update the current mode.
    // We also need to debounce this.

    monitor_state_button();

    // Monitor the on/off button. If pressed and currently awake, go to sleep.
    // Also debounce this to prevent multiple presses

    monitor_on_off_button();

    // Check the sleep timer, and go to sleep if necessary
    // Keep in mind, after switching modes, the sleep timer gets reset.

    if(millis() - sleep_timer > LENGTH_SCREEN_ON) {
      awake = false; 
    }
  }

  // State 3: Asleep
  if(!awake && !woke_up) {
    sleep_routine();
  }

}
