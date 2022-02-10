/* Edge Impulse Arduino examples
   Copyright (c) 2021 EdgeImpulse Inc.

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

/* Includes ---------------------------------------------------------------- */
#include <Tutorial_Continuous_motion_recognition_inferencing.h>
#include <Arduino_LSM9DS1.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal


/**
  @brief      Arduino setup function
*/

void setup() {
  lowpower();
  Serial1.begin(115200);
  Serial1.println("Edge Impulse Inferencing Demo");

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }
}


/**
  @brief      Get data and run inferencing

  @param[in]  debug  Get debug info if true
*/
void loop() {
  ei_printf("\nRun inference function...\n");
  run_inference();
  ei_printf("\nGoing back to low power mode...\n");
  lowpower();
  delay(10000);
}

/**
  @brief      Function to put the device in low power

  @param[in]  format     None
*/
void lowpower() {
  digitalWrite(LED_PWR, LOW);                   //Turn off the power LED
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);    //Turn off the sensors
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);     //Turn off the I2C pull-up resistors
  // Cut the 3.3 V jumper on the bottom of the board to avoid the power usage of the MPM3610 step-down converter.
}

/**
  @brief      Printf function uses vsnprintf and output using Arduino Serial

  @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial1.write(print_buf);
  }
}

void run_inference(){

  String inferenceResult = "";

  // Turn on the sensors
  ei_printf("\nSwitch on sensors...\n");
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
  if (!IMU.begin()) {  // initialize accelero
    ei_printf("Failed to initialize IMU!\r\n");
  }
  else {
    ei_printf("IMU initialized\r\n");
  }

  // Allocate a buffer here for the values we'll read from the IMU
  ei_printf("\nAllocating buffer size...\n");
  float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
  ei_printf("End allocating buffer size...\n");

  // Sample data
  ei_printf("Sampling...\n");
  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
    // Determine the next tick (and then sleep later)
    uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

    IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);

    buffer[ix + 0] *= CONVERT_G_TO_MS2;
    buffer[ix + 1] *= CONVERT_G_TO_MS2;
    buffer[ix + 2] *= CONVERT_G_TO_MS2;

    delayMicroseconds(next_tick - micros());
  }

  ei_printf("End sampling.\n");

  ei_printf("Converting to signal...\n");

  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    ei_printf("Failed to create signal from buffer (%d)\n", err);
    return;
  }

  ei_printf("End converting to signal...\n");

  ei_printf("Classifying signal...\n");

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  ei_printf("End classifying signal.\n");

  // print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  ei_printf(": \n");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    if (result.classification[ix].value > 0.7 && inferenceResult != result.classification[ix].label) {
      inferenceResult = result.classification[ix].label;
    }
  }
  Serial1.print("Movement predicted: ");
  Serial1.println(inferenceResult);
}
