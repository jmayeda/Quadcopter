#include <Arduino.h>
#include <Wire.h>

void write_channel1(int a);
void write_channel2(int b);
void write_channel3(int c);
void write_channel4(int d);

void setup()
{
  DDRD |= B11110000;
  write_channel1(1000);
  write_channel2(1000);
  write_channel3(1000);
  write_channel4(1000);

  delay(1000);
}

void loop()
{
  // int rc_values[4] = {1100, 1100, 1100, 1100};
  // int a = rc_values[0];
  // int b = rc_values[1];
  // int c = rc_values[2];
  // int d = rc_values[3];
  write_channel1(1000);
  // write_channel2(1100);
  // write_channel3(1100);
  // write_channel4(1100);
}
  // Do nothing here...
void write_channel1(int a){
  PORTD |= B10000000;''
  delayMicroseconds(a);
  PORTD &= B00000000;
  delay(20-a/1000);
}
void write_channel2(int b){
  PORTD |= B01000000;
  delayMicroseconds(b);
  PORTD &= B00000000;
  delay(20-b/1000);
}
void write_channel3(int c){
  PORTD |= B00100000;
  delayMicroseconds(c);
  PORTD &= B00000000;
  delay(20-c/1000);
}
void write_channel4(int d){
  PORTD |= B00010000;
  delayMicroseconds(d);
  PORTD &= B00000000;
  delay(20-d/1000);
}
