/*
Jesse DeWald
DeWald Designs
March 2023
Persistence of Vision Display
*/

#include <FastLED.h>
#include <avr/pgmspace.h>
// #include <avr/algorithm.h>
// #include "Texas_flag_map_out.h"
// #include "Texas_flag_map_out_full.h"
// #include "paidyn_out.h"
// #include "clockTemplate_out.h"
// #include "hollow_mech_1_out.h"
// #include "hollow_knight_simple_out.h"
#include "elsa_out.h"

//bitwise macros for setting up the timer overflows
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  // clear a bit
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   // set a bit

//images are sliced in python beginning at y = height / 2
//but the hall sensor determines where the frame starts
//set this to get the image oriented correctly. needs to be less than SLICES/2
//if offset needs to be greater than half, set frameoriented to false
#define SLICE_OFFSET 19
#define frameOriented false
#define SPEEDPIN OCR1B  //OCR1B turns clockwise (default), OCR1A CCW. Haven't coded the logic for switching the slices

// #define DIR CLOCKWISE
// #ifdef DIR
// #define MOTOR OCR1B
// #else #define MOTOR OCR1A
// #endif


//mechanical constants
const unsigned int START_SPEED = 32;                     //motor duty cycle %, roughly 2x speed in Hz @ 7kHz PWM and 12V
const unsigned int PWM_FREQ = 7;                         //pwm frequency for motor controller. in khz
const unsigned int TOP = 16000000L / (2000 * PWM_FREQ);  //sets TIMER1 overflow to set the freq
const unsigned int LED_BRIGHTNESS = 50;                  //of 255

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

const uint32_t STARTING_SLICE_TIME = (12 * (NUM_LEDS + 2)) + ((12 * (NUM_LEDS + 2)) / 3);
// clock cycles per led * # of data frames (for fastled.show()).
//setLeds() uses 1/3 of processing time that fastled.show() does

//##GLOBAL VARIABLES
//IMAGE variables
uint32_t uSecsPerSlice = STARTING_SLICE_TIME;  //how long, usecs, does a slice take?


//MOTOR variables
unsigned int motorSpeed = START_SPEED;

//##INTERRUPTS
//Hall sensor interrupt values
volatile bool hallFlag = false;  //true when interrupted
//software debounce. not using.
volatile uint32_t debounceTimeStamp = 0;  //software debounce timestamp
const uint32_t DEBOUNCE_uS = 0;           //length of debounce

//TIMER2 interrupt overflow variables for keeping pretty accurate time
const int TIMER2_THRESHOLD = 31;  //how many times does timer2 need to overflow for 1000ms
volatile int clockSeconds = 10;
volatile int clockHours = 0;
volatile int clockMinutes = 17;
volatile int timer2Counter = 0;  //how many times has the timer overflowed?


// Define the array of leds
CRGB leds[NUM_LEDS];

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
  digitalWrite(PWM_2_PIN, LOW);


  noInterrupts();                                   //turn off interrupts before we adjust the timer registers
  setupTimer2();                                    //set timer2 to overflow interrupt
  setPWMPWM_FREQ_PHASE();                           //set timer1 to manage our pwm frequency on pin SPEEDPIN
  interrupts();                                     //reenable our interrupts
  setDutyCycle(START_SPEED);                        //start your engines!
  FastLED.addLeds<APA102, BGR>(leds, NUM_LEDS);     //BGR ordering is typical
  FastLED.setCorrection(TypicalSMD5050);            //gamma correction. less than useful
  FastLED.setTemperature(FullSpectrumFluorescent);  //also less than useful. maybe these LEDs are just that good
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.setMaxRefreshRate(0);  //not needed bc we're using hardware SPI (I think) but here just in case
  FastLED.setDither(0);          //prevent fastled from trying to approximate float brightness values
  Serial.begin(115200);
  FastLED.clear();  //clear any remnants
  FastLED.show();

  delay(3000);  //wait for the motor to get up to speed
  //attach the hall sensor to an interrupt routine
  //sensor is low (7us fall time) when in presence of magnet and back to high (2ms rise) when not
  //i like the cushion from the slow rising edge, though maybe there's some penalty I'm not thinking about
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), frameStartIntVector, RISING);
}

void loop() {
  unsigned int sliceNum = 0;    //what slice of the rotation are we in
  unsigned int row;             //what row of the FRAME_ARRAY should we display
  unsigned int dirTest;         //dummy variable to make my if/else statement prettier
  uint32_t timeStampFrame = 0;  //how long has it been since we displayed the last slice?
  uint32_t debugTimeStamp = 0;
  bool ledDir = frameOriented;  //the direction that we fill the LEDs changes based on what slice we're in
  while (!hallFlag) {};         //let's wait until we passed the magnet to start our frame
  // uint32_t revStart = micros();  //could measure the rotation speed but using heuristics instead
  hallFlag = false;  //reset my flag, wait for an interrupt to change it
  //the meat and potatoes. loop through each row of FRAME_ARRAY until all slices displayed or interrupted
  while (sliceNum < SLICES && !hallFlag) {
    //our frame has SLICES-number of slices but we only need half of them because we have a full rotor
    row = (sliceNum + SLICE_OFFSET) % (SLICES / 2);  //incorporates the offset from slicing and hall placement
    dirTest = (sliceNum + SLICE_OFFSET) % SLICES;    // variable not neccesary but only costs a few clock cycles
    //if we've gone through all of FRAME_ARRAY(with len SLICES/2), swap the order of the leds
    if (dirTest == (SLICES / 2)) ledDir = !frameOriented;
    else if (dirTest % SLICES == 0) ledDir = frameOriented;
    //display the next slice only if a full slice(time) has passed
    if (micros() - timeStampFrame >= uSecsPerSlice) {
      timeStampFrame = micros();  //update our timestamp
      setLEDs(row, ledDir);       //set the damn leds
      // Serial.println(micros() - timeStampFrame);
      // while(1);
      // timeStampFrame = micros();  //update our timestamp
      FastLED.show();  //show them
      // debugTimeStamp = micros();
      // Serial.println(debugTimeStamp - timeStampFrame);
      // while (1);
      sliceNum++;  //and increase the slice number
    }
  }
  // //heurtistically adjust time per slice based on whether we were interrupted or not
  if (hallFlag) {
    if (uSecsPerSlice > 4) uSecsPerSlice -= 4;  //if we were interrupted, slices need to get faster
    else {
      setDutyCycle(motorSpeed--);           // I can do motor control this way but first I need to determine
      uSecsPerSlice = STARTING_SLICE_TIME;  //how long setLEDs takes in the best case scenario. That's my threshold for adjusting speed
    }
  } else {
    // if (uSecsPerSlice > 600) setDutyCycle(motorSpeed++);
    // else uSecsPerSlice += 4;  //otherwise slow it down a bit
    uSecsPerSlice += 4;
  }
}

//let set them LEDs!
void setLEDs(unsigned int row, bool ledDir) {
  uint32_t sliceBuffer[NUM_LEDS];                                              //buffer for the led array
  const uint32_t* row_ptr = (const uint32_t*)pgm_read_ptr(&FRAME_ARRAY[row]);  //pointer to the array in FRAME_ARRAY (LED_SLICE_n)
  memcpy_P(sliceBuffer, row_ptr, sizeof(sliceBuffer));                         //copy the pointer into the buffer
  uint32_t* ptr1 = sliceBuffer;                                                //pointer to the first half of the buffer
  uint32_t* ptr2 = sliceBuffer + NUM_LEDS / 2;                                 //pointer to the second half of the buffer
  //if through half of the slices the leds need to be reversed
  //using pointer swapping cuts processing time in half
  if (!ledDir) {
    uint32_t* tempPtr = ptr1;  //temp pointer
    ptr1 = ptr2;               //swap 2 to 1
    ptr2 = tempPtr;            //swap temp to 2
  }
  //set the led buffer
  for (int i = 0; i < NUM_LEDS / 2; i++) {
    leds[i] = *(ptr1++);
    leds[i + NUM_LEDS / 2] = *(ptr2++);
  }
}


//set up timer2. I need to attribute this function but lost the page I grepped it from
void setupTimer2() {
  TIMSK2 = 0;      //We do NOT Generate interrupts, unnecsseary since i disabled interupts but redundacy doesn't hurt me here
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
    speed = map(speed, 0, 100, 0, TOP);  //map 0 to TOP to a percent
    SPEEDPIN = speed;                    //set the speed
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
  // revEnd = micros();
}