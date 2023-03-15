/*
Jesse DeWald
DeWald Designs
March 2023
Persistence of Vision Display
*/

#include <FastLED.h>
#include <avr/pgmspace.h>
#include "Texas_flag_map_out.h"
// #include "paidyn_out.h"

//bitwise macros for setting up the timer overflows
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  // clear a bit
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   // set a bit

//images are sliced in python beginning at y = height / 2
//but the hall sensor determines where the frame starts
//set this to get the image oriented correctly. needs to be less than SLICES/2
//if offset needs to be greater than half, set frameoriented to false
#define SLICE_OFFSET 20
#define frameOriented true

//mechanical constants
const unsigned int START_SPEED = 36;                     //motor duty cycle % eq. to speed.
const unsigned int PWM_FREQ = 7;                         //pwm frequency for motor controller. in khz
const unsigned int TOP = 16000000L / (2000 * PWM_FREQ);  //TOP used for TIMER1 overflow. This actually sets the freq
const unsigned int LED_BRIGHTNESS = 100;                 //of 255

//PIN DEFINITIONS
const unsigned int HALL_PIN = 2;              //hall effect sensor
const unsigned int DIAG_PIN = 3;              //motor controller fault pin
const unsigned int ENABLE_MOTOR_PIN = A0;     //enable motor pin
const unsigned int CURRENT_MON_PIN = A1;      //current monitor
const unsigned int NOT_SHUTDOWN_5V_PIN = A2;  //5v regulator ~shutdown pin
const unsigned int PWM_1_PIN = 9;             //motor controller pwm pin 1
const unsigned int PWM_2_PIN = 10;            //motor controller pwm pin 2
const unsigned int DATA_PIN = 11;             //SPI DATA PIN (HARDWARE SPI)
const unsigned int CLK_PIN = 13;              //SPI CLOCK PIN (HARDWARE SPI)

//SMOOTHING VARS + CONSTANTS -- not currently used...too expensive
// const unsigned int SMOOTHING_NUM = 2;  //how many readings to average
// unsigned int numReadings = 0;          // how many readings have been taken
// uint32_t revTimeRunningAvg = 0;        //running average
// bool forward = true;                   //which direction is the motor turning
// int completedTimes = 0;
// int interupptedTimes = 0;

//##GLOBAL VARIABLES
//IMAGE variables
uint32_t uSecsPerSlice = 550;  //how long, usecs, does a slice take?

//##INTERRUPTS
//Hall sensor interrupt values
volatile bool hallFlag = false;  //true when interrupted
//software debounce. not using.
volatile uint32_t debounceTimeStamp = 0;  //software debounce timestamp
const uint32_t DEBOUNCE_uS = 0;           //length of debounce

//TIMER2 interrupt overflow variables for keeping pretty accurate time
volatile int clockSeconds = 10;
volatile int clockHours = 0;
volatile int clockMinutes = 17;
// unsigned long clockTime = 0;
volatile int timer2Counter = 0;   //how many times has the timer overflowed?
const int TIMER2_THRESHOLD = 31;  //how many times does it need to overflow for 1000ms

// Define the array of leds
CRGB leds[NUM_LEDS];

// pointer to progmem array. Thought global was helping speed. not sure about that anymore
uint32_t* row_ptr;

void setup() {
  //pin declarations
  pinMode(PWM_1_PIN, OUTPUT);
  pinMode(PWM_2_PIN, OUTPUT);
  pinMode(ENABLE_MOTOR_PIN, OUTPUT);
  pinMode(CURRENT_MON_PIN, INPUT);
  pinMode(HALL_PIN, INPUT);
  pinMode(DIAG_PIN, INPUT);
  pinMode(NOT_SHUTDOWN_5V_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);

  //intial pin directions
  digitalWrite(NOT_SHUTDOWN_5V_PIN, HIGH);
  digitalWrite(ENABLE_MOTOR_PIN, HIGH);
  digitalWrite(PWM_1_PIN, LOW);


  noInterrupts();                                 //turn off interrupts before we adjust the timer registers
  setupTimer2();                                  //set timer2 to overflow interrupt
  setPWMPWM_FREQ_PHASE();                         //set timer1 to manage our pwm frequency on pin PWM2
  interrupts();                                   //reenable our interrupts
  setDutyCycle(START_SPEED);                      //start your engines!
  FastLED.addLeds<DOTSTAR, BGR>(leds, NUM_LEDS);  // BGR ordering is typical
  FastLED.setCorrection(TypicalPixelString);      //gamma correction. less than useful
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.setMaxRefreshRate(0);  //not needed. here for posterity
  FastLED.setDither(0);          //prevent fastled from trying to approximate float brightness values
  // Serial.begin(115200);
  FastLED.clear();
  FastLED.show();

  delay(3000);  //wait for the motor to get up to speed
  //attach the hall sensor to an interrupt routine
  //sensor is low (7us fall time) when in presence of magnet and back to high (2ms rise) when not
  //i like the cushion from the slow rising edge, though maybe this is what holding back my high slice numbers...
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), frameStartIntVector, RISING);
}

void loop() {
  unsigned int sliceNum = 0;    //what slice of the rotation are we in
  unsigned int row;             //what row of the FRAME_ARRAY should we display
  unsigned int test;            //dummy variable to make my if/else statement prettier
  uint32_t timeStampFrame = 0;  //how long has it been since we displayed the last slice?
  bool ledDir = frameOriented;  //the direction that we fill the LEDs changes based on what slice we're in
  while (!hallFlag) {};         //let's wait until we passed the magnet to start our frame
  hallFlag = false;             //reset my flag, wait for an interrupt to change it
  //the meat and potatoes. loop through each row of FRAME_ARRAY until all slices displayed or interrupted
  while (sliceNum < SLICES && !hallFlag) {
    //our frame has SLICES-number of slices but we only need half of them because we have a full rotor
    row = (sliceNum + SLICE_OFFSET) % (SLICES / 2);
    test = (sliceNum + SLICE_OFFSET) % SLICES;
    //if we've gone through all of FRAME_ARRAY, swap the order of the leds
    if (test == (SLICES / 2)) ledDir = !frameOriented;
    else if (test % SLICES == 0) ledDir = frameOriented;
    //display the next slice only if a full slice(time) has passed
    if (micros() - timeStampFrame >= uSecsPerSlice) {
      timeStampFrame = micros();  //update our timestamp
      setLEDs(row, ledDir);       //set the damn leds
      FastLED.show();             //show them
      sliceNum++;                 //and increase the slice number
    }
  }
  //heurtistically adjust time per slice based on whether we were interrupted or not
  if (hallFlag) {
    if (uSecsPerSlice > 2) uSecsPerSlice -= 2; //if we were interrupted, slices need to get faster
  } else uSecsPerSlice += 2; //otherwise slow it down a bit
}

// #pragma GCC optimize("-O3")
void setLEDs(unsigned int row, bool ledDir) {
  row_ptr = (uint32_t*)pgm_read_word(&(FRAME_ARRAY[row])); //get the pointer to the address of the LED_SLICE# from FRAME_ARRAY
  for (int pixel = 0; pixel < NUM_LEDS; pixel++) {
    //read the value at the address of the pointer
    if (ledDir) leds[pixel] = pgm_read_dword(&(row_ptr[pixel]));//if we're less than halfway through the frame, normal assignment
    else leds[pixel] = pgm_read_dword(&(row_ptr[NUM_LEDS - 1 - pixel]));//otherwise reverse the assignment
  }
}

//set up timer2. I need to attribute this function but lost the page I grepped it from
void setupTimer2() {
  TIMSK2 = 0;      //We do NOT Generate interrupts
  cbi(ASSR, AS2);  //// use clock, not T2 pin .. probably defaulted anyway
  /*When the value of AS2 is changed, the contents of TCNT2, OCR2A,
   OCR2B, TCCR2A and TCCR2B might be corrupted. :8271C–AVR–08/10 p165*/
  //Clear the Timer Registers - now we know what we have
  TCCR2B = 0;
  TCCR2A = 0;
  TCNT2 = 0;
  /*TCCR2A contains the compare match mode and part of the wave generation
   mode bits
   */
  sbi(TCCR2A, WGM20);  //WGM20 is bit 1
  /*TCCR2B contains the prescaler and the remainer of the wave generation
   mode bits.  Results in Waveform generation mode 5 on Table 17-8 of the
   datasheet.  However, we are not actually generating a waveform.
   */
  sbi(TCCR2B, WGM22);  //WGM22 is bit 1
  //Timer2 Settings:  Timer Prescaler /1024
  sbi(TCCR2B, CS22);  //set this bit
  sbi(TCCR2B, CS21);  //set this bit
  sbi(TCCR2B, CS20);  //set this bit
  //Timer2 Overflow Interrupt Enable
  /*
   Bit 0 – TOIE2: Timer/Counter2 Overflow Interrupt Enable
   When the TOIE2 bit is written to one and the I-bit in the Status Register
   is set (one), the Timer/Counter2 Overflow interrupt is enabled. The
   corresponding interrupt is executed if an overflow in Timer/Counter2
   occurs, i.e., when the TOV2 bit is set in the Timer/Counter2 Interrupt
   Flag Register – TIFR2.
   */
  OCR2A = 252;         //252 results in a 1000ms period. 63 results in 250ms
  sbi(TIMSK2, TOIE2);  //enable the timer to raise overflow interrupts
}

//set timer1 to phase correct pwm based on TOP
void setPWMPWM_FREQ_PHASE() {
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13)
           | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
  // Set TCCR1A
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  // Set TCCR1B
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = TOP;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13)
           | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
}

//set timer1 to fast-pwm mode based on TOP
void setPWMPWM_FREQ_FAST() {
  // Clear TCCR1B
  TCCR1B = 0;
  // Set TCCR1A
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  // Set TCCR1B
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = TOP;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;
  // Disable Timer/Counter1 interrupts
  TIMSK1 = 0;
  // Start the timer
  TCCR1B |= _BV(CS10);
}

//what speed to run the motor at
void setDutyCycle(int speed) {
  //value check
  if (speed >= 0 && speed <= 100) {
    speed = map(speed, 0, 100, 0, TOP);//map 0 to TOP to a percent
    OCR1B = speed; //set the speed
  }
}

//the overflow interrupt routine for timer2 to accurately count 1000ms
ISR(TIMER2_OVF_vect) {

  /*This interrupt fires once every 
   32.64 milliseconds
   The counter has to count UP and then it counts back down due to the
   way the registers are set.
   16MHz / (1024*(255*2)) = 30.63725 interrupts per second ==>(1012 ms period)
   16MHz / (1024*(252*2)) = 31.00198413 interrupts per second 
   */
  timer2Counter += 1;
  if (timer2Counter == TIMER2_THRESHOLD) {
    if (clockSeconds < 59) clockSeconds++;
    else {
      clockSeconds = 0;
      if (clockMinutes < 59) clockMinutes++;
      else clockMinutes = 0;
    }
    timer2Counter = 0;
  }
}

//hall effect interrupt routine. super duper fast.
void frameStartIntVector() {
  // if (micros() - debounceTimeStamp > DEBOUNCE_uS) {
  //   hallFlag = true;
  // }
  // debounceTimeStamp = micros();
  hallFlag = true;
}