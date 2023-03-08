#include <FastLED.h>
#include <avr/pgmspace.h>
#include "Texas_flag_map_out.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define SLICE_OFFSET 20

const unsigned int START_SPEED = 36;                     //empirical starting duty cycle %
const unsigned int PWM_FREQ = 7;                         //in khz
const unsigned int TOP = 16000000L / (2000 * PWM_FREQ);  //TOP used for TIMER overflow
const unsigned int LED_BRIGHTNESS = 50;                  //of 255

//PIN DEFINITIONS
const unsigned int HALL_PIN = 2;
const unsigned int DIAG_PIN = 3;
const unsigned int ENABLE_MOTOR_PIN = A0;
const unsigned int CURRENT_MON_PIN = A1;
const unsigned int NOT_SHUTDOWN_5V_PIN = A2;
const unsigned int PWM_1_PIN = 9;
const unsigned int PWM_2_PIN = 10;
const unsigned int DATA_PIN = 11;
const unsigned int CLK_PIN = 13;

//SMOOTHING VARS + CONSTANTS
const unsigned int SMOOTHING_NUM = 2;  //how many readings to average
unsigned int numReadings = 0;          // how many readings have been taken
uint32_t revTimeRunningAvg = 0;        //running average
bool forward = true;                   //which direction is the motor turning
int completedTimes = 0;
int interupptedTimes = 0;

volatile int clockSeconds = 10;
volatile int clockHours = 0;
volatile int clockMinutes = 17;
unsigned long clockTime = 0;


//IMAGE VARS
uint32_t uSecsPerSlice = 550;  //

volatile bool hallFlag = false;
volatile uint32_t debounceTimeStamp = 0;
const uint32_t DEBOUNCE_uS = 0;


volatile int timer2Counter = 0;
const int TIMER2_THRESHOLD = 31;

// Define the array of leds
CRGB leds[NUM_LEDS];

// uint32_t row_buffer[NUM_LEDS];
uint32_t* row_ptr;

void setup() {
  pinMode(PWM_1_PIN, OUTPUT);
  pinMode(PWM_2_PIN, OUTPUT);
  pinMode(ENABLE_MOTOR_PIN, OUTPUT);
  pinMode(CURRENT_MON_PIN, INPUT);
  pinMode(HALL_PIN, INPUT);
  pinMode(DIAG_PIN, INPUT);
  pinMode(NOT_SHUTDOWN_5V_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);

  digitalWrite(NOT_SHUTDOWN_5V_PIN, HIGH);
  digitalWrite(ENABLE_MOTOR_PIN, HIGH);
  digitalWrite(PWM_1_PIN, LOW);

  // noInterrupts();
  // setupTimer2();
  setPWMPWM_FREQ_PHASE();
  interrupts();
  setDutyCycle(START_SPEED);
  FastLED.addLeds<DOTSTAR, BGR>(leds, NUM_LEDS);  // BGR ordering is typical
  // FastLED.setCorrection(TypicalPixelString);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.setMaxRefreshRate(0);
  Serial.begin(115200);
  FastLED.clear();
  FastLED.show();

  delay(3000);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), frameStartIntVector, RISING);
}

void loop() {
  unsigned int sliceNum = 0;
  unsigned int row;
  uint32_t timeStampFrame = 0;
  bool ledDir = true;
  // if (hallFlag) {
  while (!hallFlag) {};
  hallFlag = false;
  while (sliceNum < SLICES && !hallFlag) {
    row = (sliceNum + SLICE_OFFSET) % (SLICES / 2);
    // row = sliceNum % (SLICES/2);
    // if (row == (SLICE_OFFSET + SLICES / 2) % SLICES) ledDir = false;
    if ((sliceNum + SLICE_OFFSET) % SLICES == (SLICES / 2)) ledDir = false;
    else if ((sliceNum + SLICE_OFFSET) % SLICES == 0) ledDir = true;
    if (micros() - timeStampFrame >= uSecsPerSlice) {
      timeStampFrame = micros();
      row_ptr = (uint32_t*)pgm_read_word(&(FRAME_ARRAY[row]));
      for (int pixel = 0; pixel < NUM_LEDS; pixel++) {
        // leds[pixel] = pgm_read_dword(&(row_ptr[pixel]));
        if (ledDir) leds[pixel] = pgm_read_dword(&(row_ptr[pixel]));
        else leds[pixel] = pgm_read_dword(&(row_ptr[NUM_LEDS - 1 - pixel]));
      }
      FastLED.show();
      sliceNum++;
    }
  }
  if (hallFlag) {
    if (uSecsPerSlice > 2) uSecsPerSlice -= 2;
  } else uSecsPerSlice += 2;
  // Serial.println(uSecsPerSlice);
}

void setupTimer2() {
  //set up timer2
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

void setDutyCycle(int speed) {
  if (speed >= 0 && speed <= 100) {
    speed = map(speed, 0, 100, 0, TOP);
    OCR1B = speed;
  }
}


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

void frameStartIntVector() {
  // if (micros() - debounceTimeStamp > DEBOUNCE_uS) {
  //   hallFlag = true;
  // }
  // debounceTimeStamp = micros();
  hallFlag = true;
}