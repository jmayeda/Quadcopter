#include <EnableInterrupt.h>

#define RC_NUM_CHANNELS 4

// Assign channel numbers
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 2
#define RC_CH4 3

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

// Read and copy values from rc_shared into array rc_values
void rc_read_values(){
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

// Calculate the pulse width of each incoming signal
void calc_input(uint8_t channel, uint16_t input_pin) {
  if ((PINB & (1 << input_pin))  >  0) {
    rc_start[channel] = micros(); // obtain time position when pin goes HIGH
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]); // obtain time position when pin goes LOW and calculate time difference for pulse width
    rc_shared[channel] = constrain(rc_compare,1000,2000); // constrain values between 1000-2000ms
  }
}

// calc_input to read each channel when interrupts occur
void calc_ch1() { calc_input(RC_CH1, PB0); }
void calc_ch2() { calc_input(RC_CH2, PB1); }
void calc_ch3() { calc_input(RC_CH3, PB2); }
void calc_ch4() { calc_input(RC_CH4, PB3); }

void setup() {
    // put your setup code here, to run once:
    Serial.begin(57600);
    DDRB = B00000000; // set D8 to D13 as inputs
    enableInterrupt(8, calc_ch1, CHANGE);
    enableInterrupt(9, calc_ch2, CHANGE);
    enableInterrupt(10, calc_ch3, CHANGE);
    enableInterrupt(11, calc_ch4, CHANGE);
}

void loop() {
    // Print values from digital pins 8 through 11
    rc_read_values();
    // Serial.print("CH1:"); Serial.print((PINB & (1<<PB0))); Serial.print("\t");
    // Serial.print("CH2:"); Serial.print((PINB & (1<<PB1))); Serial.print("\t");
    // Serial.print("CH3:"); Serial.print((PINB & (1<<PB2))); Serial.print("\t");
    // Serial.print("CH4:"); Serial.println((PINB & (1<<PB3)));
    Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
    Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
    Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
    Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);
    delay(200);
}
