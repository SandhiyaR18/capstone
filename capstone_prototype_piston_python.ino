#include <PeakDetection.h>

#include <vl53l4cd_api.h>
#include <vl53l4cd_class.h>

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;


void setup() {
  // put your setup code here, to run once:
   
  
  //sensor
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);

}

void loop() {
  //digitalWrite(LED_PIN,HIGH);
  // put your main code here, to run repeatedly:
  //need to initialize the first array to have the base measurement
  
   Serial.println(sensor.read());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  

}








