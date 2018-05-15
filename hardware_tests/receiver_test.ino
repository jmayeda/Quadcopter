
/**
 * receiver_test.ino
 *
 * @date: 05/01/18
 * @brief: read receiver pulse position modulation signals, and write to LED's
 *         for visualization.
 */

/* Pin Definitons */
#define RCVR_CH_1 2
#define RCVR_CH_2 3
#define RCVR_CH_3 4
#define RCVR_CH_4 5
#define LED1_PIN 6
#define LED2_PIN 9
#define LED3_PIN 10
#define LED4_PIN 11

/* Global Variables */
unsigned long rc1, rc2, rc3, rc4; // reciever channel signals
int led1_pwm, led2_pwm, led3_pwm, led4_pwm; // LED PWM values

/* Setup */
void setup()
{
  // set all reciever channel pins as inputs to read the incoming signal
  pinMode(RCVR_CH_1, INPUT);
  pinMode(RCVR_CH_2, INPUT);
  pinMode(RCVR_CH_3, INPUT);
  pinMode(RCVR_CH_4, INPUT);

  // set all LED pins as outputs to output a PWM signal
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);

  // start the serial monitor for debugging
  Serial.begin(9600);
}

/* Main loop */
void loop()
{
  // read the receiver signals using pulseIn()
  rc1 = pulseIn(RCVR_CH_1, HIGH);
  rc2 = pulseIn(RCVR_CH_2, HIGH);
  rc3 = pulseIn(RCVR_CH_3, HIGH);
  rc4 = pulseIn(RCVR_CH_4, HIGH);

  // map the receiver signal which is a 1000-2000 us value to an integer
  // to write to the LED pin. The Arduino 8bit Analog-to-Digital converter
  // supports values between 0-256.
  led1_pwm = map(rc1, 990, 2010, 0, 255);
  led2_pwm = map(rc2, 990, 2010, 0, 255);
  led3_pwm = map(rc3, 990, 2010, 0, 255);
  led4_pwm = map(rc4, 990, 2010, 0, 255);
  
  analogWrite(LED1_PIN, led1_pwm);
  analogWrite(LED2_PIN, led2_pwm);
  analogWrite(LED3_PIN, led3_pwm);
  analogWrite(LED4_PIN, led4_pwm);

  Serial.print(rc1);
  Serial.print("  ");
  Serial.print(rc2);
  Serial.print("  ");
  Serial.print(rc3);
  Serial.print("  ");
  Serial.println(rc4);
}
