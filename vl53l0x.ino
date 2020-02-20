#include "Adafruit_VL53L0X.h"
#include <Servo.h>
#include "MainClock.h"
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
// left
#define SHT_LOX1 10
// right
#define SHT_LOX2 8

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

bool ready = true;
int threshold = 300;
Servo servo;
int valueFirstSensor = 1000;
int valueSecondSensor = 1000;
unsigned long time1 = 0;
unsigned long time2 = 0;
int ticksSecondSensor = 0;
int ticks = 0;

MainClock mainclk = MainClock();



/*  
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print("1: ");
  if(measure1.RangeStatus != 4) {     // if not out of range
    measure1.RangeMilliMeter = measure1.RangeMilliMeter - 20;
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" ");

  // print sensor two reading
  Serial.print("2: ");
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  servo.attach(9);
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Shutdown pins inited...");

  Serial.println("Both in reset mode...(pins are low)");
  
  Serial.println("Starting...");
  setID();
  mainclk.startTimer();
}

void loop() {
  //test();
    if(mainclk.isTick()){
      ticks++;
    }
    //read_dual_sensors();
    peaksBerechnenUndAusgeben();

    if(Serial.read()!=-1) {
      Serial.println("ZEIT:");
      Serial.println(ticks - ticksSecondSensor);
      Serial.println(millis() - time1);
      ticks = 0;
      ticksSecondSensor = 0;
      feuer();
    }
}

void laden(){
  if(!ready){
    servo.write(30); //Laden
    delay(300);
    servo.write(110);
    ready = true;
  }

}

void feuer(){
  if(ready){
    servo.write(140); 
    delay(450);
    servo.write(110);
    delay(200);
    ready = false;
    laden();
  }
}

void peaksBerechnenUndAusgeben() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  if(ticks - ticksSecondSensor > 15) {
  if(measure1.RangeMilliMeter < valueFirstSensor) {
    valueFirstSensor = measure1.RangeMilliMeter;
  } else if(valueFirstSensor < 335){
      Serial.println("ValueFirstSensor: "); 
      Serial.println(valueFirstSensor);
    valueFirstSensor = 1000;
  }
  if(measure2.RangeMilliMeter < valueSecondSensor) {
    valueSecondSensor = measure2.RangeMilliMeter;
  } else if(valueSecondSensor < 315){
      Serial.println("valueSecondSensor: "); 
      Serial.println(valueSecondSensor);
      valueSecondSensor = 1000;
      Serial.println();
      ticksSecondSensor = ticks;
      time1 = millis();
  }
  }
  
}

void test() {
  Serial.println("TESTING");
 while(ticks < 10000){
    if(mainclk.isTick()){
      ticks++;
      Serial.println(ticks);
    }
  }
}


/*bool berechneDelay(int value) {
  if(value > threshold - 20) {
    delay(100);
  } else if(value > threshold - 50) {
    delay(130);
  } else if(value > threshold - 90){
    delay(160); 
  } else {
    return false;
  }
  return true;
}*/
