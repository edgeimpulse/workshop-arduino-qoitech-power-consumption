/*
 * Low power mode Arduino Nano 33 BLE sense
 * more info here: 
 * https://support.arduino.cc/hc/en-us/articles/4402394378770-How-to-reduce-power-consumption-on-the-Nano-33-BLE
 */

void setup() {
 
  digitalWrite(LED_PWR, LOW);                   //Turn off the power LED
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);    //Turn off the sensors
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);     //Turn off the I2C pull-up resistors
  // Cut the 3.3 V jumper on the bottom of the board to avoid the power usage of the MPM3610 step-down converter.
}

void loop() {
  delay(1000);
}
