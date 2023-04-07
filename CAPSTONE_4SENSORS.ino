//connecting multiple time of flight sensors
#include <VL53L1X.h>
#include <SoftwareSerial.h>

//initializing sensors - reference code from Polulu vl53l1x library example
const uint8_t numSensors = 4; //total of 4 sensors used
const uint8_t xshutPins[numSensors] = {10,9,8,11}; //xshut pins
VL53L1X sensors[numSensors];

SoftwareSerial mySerial(12, 13);
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider

//LED Pins
//const int LED_PIN=A1;
const int RATE_LED_PIN = 5;//blue
const int DEPTH_LED_PIN = 3;//green
const int RECOIL_LED_PIN = 7;//yellow

int threshold = 90;//threshold for returning to baseline state of no compressions
int attempt_threshold = 80;
int min_depth = 75; //minimum value that should be reached for a correct depth compression
int max_depth = 30; //maximum value that should be reached for a correct depth compression

unsigned long compression_time = 0;
unsigned long current_time = 0;
unsigned long time_now = 0;

bool compressed = false;
bool correct_depth = false;
bool correct_return = true;
bool correct_rate = false;

int sensorvals[4];

//for moving avg filters
int points[10];
int sum = 0;
int curr = 0;
int points2[10];
int sum2 = 0;
int curr2 = 0;
int points3[10];
int sum3 = 0;
int curr3 = 0;
int points4[10];
int sum4 = 0;
int curr4 = 0;
int left_points[10]; 
int left_sum = 0;
int left_curr = 0; //index for the current value
int right_points[10]; 
int right_sum = 0;
int right_curr = 0; //index for the current value
int top_points[10]; 
int top_sum = 0;
int top_curr = 0; //index for the current value
int bottom_points[10]; 
int bottom_sum = 0;
int bottom_curr = 0; //index for the current value

int left_val, right_val, top_val, bottom_val,other_val;
int difference_horizontal = 0;
int difference_vertical = 0;
//flags to send to app

int hand_flag = 0;
int comp_flag = 0;
int comp_count = 0;
int compression_count = 0;
int rate_count = 0;
int recoil_count = 0;
int depth_count = 0;

void setup()
{
  pinMode(DEPTH_LED_PIN, OUTPUT);
  pinMode(RATE_LED_PIN, OUTPUT);
  pinMode(RECOIL_LED_PIN, OUTPUT);

  while (!Serial) {}
  Serial.begin(230400);
  mySerial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // xshut pins driven to low to reset all 4 sensors
  for (uint8_t i = 0; i < numSensors; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // enable and initialize each sensor 
  for (uint8_t i = 0; i < numSensors; i++)
  {
    // turn off XSHUT low
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // change address of each to unique non default address 
    sensors[i].setAddress(0x2A + i);

    sensors[i].setDistanceMode(VL53L1X::Short);
    sensors[i].setMeasurementTimingBudget(20000);

    sensors[i].startContinuous(20);
  }

  digitalWrite(DEPTH_LED_PIN,LOW);
  digitalWrite(RATE_LED_PIN,LOW);
  digitalWrite(RECOIL_LED_PIN,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < numSensors; i++)
  {
    sensorvals[i] = sensors[i].read();
    //Serial.print(sensorvals[i]);
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    //Serial.print('\t');
  }
  other_val = movingAvg(sensorvals[1]);
  left_val = movingSecondAvg(sensorvals[0]);
  bottom_val = movingThirdAvg(sensorvals[2]);
  top_val = movingFourthAvg(sensorvals[3]);

  other_val = (right_val+left_val+bottom_val+top_val)/4; //average all 4 sensor readings 

 
  //LEFT= 1
  //RIGHT = 2
  //BOTTOM = 3
  // //TOP = 4
 
  // //if there is a big difference between the above two values, the platform is probably angled and hand placement is incorrect 
  int n = 4;
  int max_val = sensorvals[0];
  int min_val = sensorvals[0];
  for (int i = 1; i < n; i++) { //find maximum value by continuosly comparing values to each other
  if (sensorvals[i] > max_val) {
    max_val = sensorvals[i];
  }
  if (sensorvals[i] < min_val){
    min_val = sensorvals[i];
  }

 
  
}


  if (abs(min_val - max_val)>30){ //threshold set to 30 based on testing
    hand_flag = 0;
  }
  else{
    hand_flag = 1; //CORRECT
  }

 

  //2. calculate depth,rate,recoil based on TOP sensor 
   
   delay(5);
  

  //smooth_val = analogRead(A0); //THIS IS TEMPORARY, WILL HAVE THE SIGNAL READ LINE HERE
  //Serial.println(String(top_val));

  
  if (compressed == false && other_val <= attempt_threshold){ //Detects an attempted compression to check for correct rate
    compressed = true;
    comp_count += 1;

    if ((millis() - compression_time)>=400 && (millis() - compression_time)<=700){
      digitalWrite(RATE_LED_PIN,HIGH);
      compression_count+=1;
      rate_count+= 1;
    }
    else{
      digitalWrite(RATE_LED_PIN,LOW);
      
    }
    compression_time = millis();
    //starting a compression, get the current time
     
        
  }
  else if(compressed == true && other_val > attempt_threshold){
    compressed = false;
  }

  if (correct_return == false && other_val >= threshold){
    //returning to not compressed state
    correct_return = true;
    digitalWrite(RECOIL_LED_PIN,HIGH);//turn on recoil LED
    recoil_count += 1;
  }
  else if (correct_return == true && other_val < threshold){
    correct_return = false;
    digitalWrite(RECOIL_LED_PIN,LOW);//turn off recoil LED
    
  }

  //checking for correct depth target
  if (correct_depth == false && other_val <= min_depth && other_val >= max_depth){
    //user has compressed correctly and reached depth target
    correct_depth = true;
    digitalWrite(DEPTH_LED_PIN,HIGH);
    depth_count += 1;
  }
  else if (correct_depth == true && other_val > min_depth){
    //done compressing, check for return to threshold
    correct_depth = false;
    digitalWrite(DEPTH_LED_PIN,LOW);
  }
  if (recoil_count>=comp_count){
    recoil_count = comp_count;

  }

  Serial.println(String(other_val)+","+String(depth_count)+","+String(recoil_count)+","+String(rate_count)+","+String(comp_count));
  mySerial.println(String(other_val)+","+String(depth_count)+","+String(recoil_count)+","+String(rate_count)+","+String(comp_count)+","+String(hand_flag)); //SENDING OVER BLUETOOTH

}


//moving avg filter
int movingAvg(int val) {
  sum = sum - points[curr]; //subtract the oldest value within the window from the sum
  points[curr] = val; //add the current reading into end of array
  sum += val;
  curr = (curr+1)%10; //index increments, reset the index to the beginning of the window if it exceeds
 
  
  return sum/10;
}

//moving avg filter
int movingSecondAvg(int val) {
  sum2 = sum2 - points2[curr2]; //subtract the oldest value within the window from the sum
  points2[curr2] = val; //add the current reading into end of array
  sum2 += val;
  curr2 = (curr2+1)%10; //index increments, reset the index to the beginning of the window if it exceeds
 
  
  return sum2/10;
}


int movingThirdAvg(int val) {
  sum3 = sum3 - points3[curr3]; //subtract the oldest value within the window from the sum
  points3[curr3] = val; //add the current reading into end of array
  sum3 += val;
  curr3 = (curr3+1)%10; //index increments, reset the index to the beginning of the window if it exceeds
 
  
  return sum3/10;
}

int movingFourthAvg(int val) {
  sum4 = sum4 - points4[curr4]; //subtract the oldest value within the window from the sum
  points4[curr4] = val; //add the current reading into end of array
  sum4 += val;
  curr4 = (curr4+1)%10; //index increments, reset the index to the beginning of the window if it exceeds
 
  
  return sum4/10;
}





