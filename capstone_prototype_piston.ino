#include <PeakDetection.h>

#include <vl53l4cd_api.h>
#include <vl53l4cd_class.h>

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

//LED Pins
//const int LED_PIN=A1;
const int RATE_LED_PIN = 3;
const int DEPTH_LED_PIN = 7;
const int RECOIL_LED_PIN = 5;

int threshold = 150; //change this value later
int depth1 = 140;
int depth2 = 190;
// float avg_points = 2; //change this later
// float recoil_range = 2; //the tolerance of values allowed for the "no compression" reading
// float no_compression = 2; //value of reading when no compression applied
unsigned long compression_time = 0;
unsigned long current_time = 0;
unsigned long time_now = 0;

bool compressed = false;


// //for moving average filter
// const int npoints = 3; //average 3 points in moving average filter
// int points [npoints]; 
// float sum = 0;
// float curr = 0; //index for the current value

PeakDetection peakDetection; // create PeakDetection object
int val;
int prev_peak;

void setup() {
  // put your setup code here, to run once:
   
  //LEDS
  //pinMode(LED_PIN,OUTPUT);
  pinMode(DEPTH_LED_PIN, OUTPUT);
  pinMode(RATE_LED_PIN, OUTPUT);
  pinMode(RECOIL_LED_PIN, OUTPUT);

  //peak detection
  //Influence affects how peak values will change mean, higher influence means peaks will increase the mean/threshold
  //More stationary data --> should have higher lag
  //48,3,0.1 was not bad
  //30,3,0.1 was better
  //10,3,0.1 was best, considering 10,2,0.1
  peakDetection.begin(10,3,0.05); // sets the lag, threshold and influence, initially 48,3,0.6
  
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
   val = sensor.read();
   Serial.print(val);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  

  //smooth_val = analogRead(A0); //THIS IS TEMPORARY, WILL HAVE THE SIGNAL READ LINE HERE

  peakDetection.add(val); // adds a new data point
  int peak = peakDetection.getPeak(); // returns 0, 1 or -1
  double filtered = peakDetection.getFilt(); // moving average
  String m=" Is it a peak: ";
  String k=m+ peak;
  Serial.print(k);
  Serial.println();

  // if (peak == 1){
  //   digitalWrite(LED_PIN,HIGH);
  // }
  // else{
  //   digitalWrite(LED_PIN,LOW);
  // }

  //CHECK IF PEAK IS -1 --> MEANS THE DATA POINT IS A LOW PEAK, WHICH IS WHAT WE ARE LOOKING FOR
  
  
  if (peak == -1 && val <= threshold){ //detecting an attempted compression
  //LED PIN TURNS WHITE
      if (val <= depth1){ //should check in both directions here
          digitalWrite(DEPTH_LED_PIN, HIGH); //turn green
      }
      //digitalWrite(DEPTH_LED_PIN, LOW); //turn red
      current_time = millis();
      if (prev_peak!=-1){ //make sure it's a real new compression
      // 100 to 120 cpr/min
      //min / compression 60/100=0.6 seconds = 600 ms
      // 60/120=0.5=500 md
      // buffer time of 0.3 ms
      // compression rate--> RED LED
           if ((current_time - compression_time) <= 800 && (current_time-compression_time)>=600){ 
                digitalWrite(RATE_LED_PIN,HIGH);
          }
          else{
                digitalWrite(RATE_LED_PIN,LOW);
          }
      
      compression_time = millis(); //set new compression time to current time
      }

  }
  else{
    digitalWrite(DEPTH_LED_PIN,LOW);
  }
  prev_peak = peak;

  //have to check for recoil after compression, maybe use boolean flag to say compression occurred --> have to detect "base" value within a certain time period
  time_now = millis();
  if ((time_now - compression_time) <= 800){
    if (val >= threshold){ //returning to initial state between compressions
       digitalWrite(RECOIL_LED_PIN,HIGH);
    }     
  }
  else{
    digitalWrite(RECOIL_LED_PIN,LOW);
  }
}

// float movingAvg(float val) {
//   sum = sum - points[curr];
//   points[curr] = val; //add the current reading into end of array
//   sum += points[curr];
//   curr++;
//   if (curr >= npoints){
//     curr = 0; //reset index
//   }

//   return sum/npoints;
// }








