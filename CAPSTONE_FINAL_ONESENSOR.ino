

//#include <vl53l4cd_api.h>
//#include <vl53l4cd_class.h>

#include <Wire.h>
#include <VL53L1X.h>
#include <SoftwareSerial.h>


VL53L1X sensor;
SoftwareSerial mySerial(8, 9);
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider

//LED Pins
//const int LED_PIN=A1;
const int RATE_LED_PIN = 5;//blue
const int DEPTH_LED_PIN = 3;//green
const int RECOIL_LED_PIN = 7;//yellow

int threshold = 100; //threshold for returning to baseline state of no compressions
int attempt_threshold = 90;
int min_depth = 60; //minimum value that should be reached for a correct depth compression
int max_depth = 40; //maximum value that should be reached for a correct depth compression

unsigned long compression_time = 0;
unsigned long current_time = 0;
unsigned long time_now = 0;

bool compressed = false;
bool correct_depth = false;
bool correct_return = true;
bool correct_rate = false;

// //for moving average filter
// const int npoints = 3; //average 3 points in moving average filter
//int FILTER_WINDOW = 5; 
int points[10]; 
int sum = 0;
int curr = 0; //index for the current value

int val;
int filtered_val;
int compression_count = 0;

//flags to send to app
int depth_flag = 0;
int rate_flag = 0;
int recoil_flag = 0;

void setup() {

  // for the app
  //mySerial.begin(9600);
  Serial.begin(115200);
  delay(100);

  //// app ends here for integration
  // put your setup code here, to run once:
   
  //LEDS
  //pinMode(LED_PIN,OUTPUT);
  pinMode(DEPTH_LED_PIN, OUTPUT);
  pinMode(RATE_LED_PIN, OUTPUT);
  pinMode(RECOIL_LED_PIN, OUTPUT);
  
  //sensor
  while (!Serial) {}
  // Serial.begin(115200);
  Serial.begin(230400);
  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    // mySerial.println("Failed to detect and initialize sensor!"); // for the app
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
  digitalWrite(DEPTH_LED_PIN,LOW);
  digitalWrite(RATE_LED_PIN,LOW);
  digitalWrite(RECOIL_LED_PIN,LOW);

}

void loop() {
  
 
  //digitalWrite(LED_PIN,HIGH);
  // put your main code here, to run repeatedly:
  //need to initialize the first array to have the base measurement
   val = sensor.read();
   //run through moving average filter 
   filtered_val = movingAvg(val);
   Serial.println(filtered_val);

   //fsrReading = analogRead(fsrPin);
   
   delay(5);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  
  //smooth_val = analogRead(A0); //THIS IS TEMPORARY, WILL HAVE THE SIGNAL READ LINE HERE
  Serial.println(String(filtered_val));
 
  
  if (compressed == false && filtered_val <= attempt_threshold){ //Detects an attempted compression to check for correct rate
    compressed = true;
    if ((millis() - compression_time)>=600 && (millis() - compression_time)<=900){
      digitalWrite(RATE_LED_PIN,HIGH);
    }
    else{
      digitalWrite(RATE_LED_PIN,LOW);
    }
    compression_time = millis();
    //starting a compression, get the current time
   
  }
  else if(compressed == true && filtered_val > attempt_threshold){
    compressed = false;
  }

  if (correct_return == false && filtered_val >= threshold){
    //returning to not compressed state
    correct_return = true;
    digitalWrite(RECOIL_LED_PIN,HIGH);//turn on recoil LED
  }
  else if (correct_return == true && filtered_val < threshold){
    correct_return = false;
    digitalWrite(RECOIL_LED_PIN,LOW);//turn off recoil LED
    
  }

  //checking for correct depth target
  if (correct_depth == false && filtered_val <= min_depth && filtered_val >= max_depth){
    //user has compressed correctly and reached depth target
    correct_depth = true;
    digitalWrite(DEPTH_LED_PIN,HIGH);
  }
  else if (correct_depth == true && filtered_val > min_depth){
    //done compressing, check for return to threshold
    correct_depth = false;
    digitalWrite(DEPTH_LED_PIN,LOW);
  }
  
}

int movingAvg(int val) {
  sum = sum - points[curr]; //subtract the oldest value within the window from the sum
  points[curr] = val; //add the current reading into end of array
  sum += val;
  curr = (curr+1)%10; //index increments, reset the index to the beginning of the window if it exceeds
 
  
  return sum/10;
}







