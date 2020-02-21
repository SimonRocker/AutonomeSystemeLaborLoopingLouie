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
bool testing = false;
bool erwarteFeuer = false;
int threshold = 300;
Servo servo;
int valueFirstSensor = 1000;
int valueSecondSensor = 1000;
unsigned long millisZweiterSensor = 0;
unsigned long millisErsterSensor = 0;
int ticksSecondSensor = 0;
int ticks = 0;
int testMillis = 0;

MainClock mainclk = MainClock();



/*  
 *   Setzt die Adressen für beide Sensoren, sodass die Werte beider Sensoren abgerufen werden können. Ohne diese Methode ist das Nutzen mehrerer Tiefensensoren nicht möglich. 
 *   Genutzt werden hier zwei Adafruit VL53l0X Sensoren. 
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

/*
 * Um die Daten der Sensoren zu sehen kann man diese Methode benutzen. Diese ist aktuell in der Loop auskommentiert. Um die Werte dann zu sehen muss entweder der serielle
 * Monitor (STRG + Umschalt + M, zeigt die Werte als Zahl an) oder (besser) der serielle Plotter (STRG + Umschalt + L, zeigt die Werte als Kurven an) genutzt werden.
 */
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

/*
 * Setup Methode, wird einmalig beim Start des Programms zuerst aufgerufen. Wenn diese Methode mehrmals aufgerufen wird, wird während der Laufzeit eine Exception geworfen 
 * (OutOfBounds, kein Speicher mehr, etc.). 
 */
void setup() {
  Serial.begin(115200);
  servo.attach(9);
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Starting...");
  setID();
  mainclk.startTimer();
}

/*
 * Loop Methode, welche nach der Setup Methode immer wieder ausgeführt wird. Aktuell dauert ein Durchlauf ca 80-90 ms. 
 * Um Testdaten zu generieren den bool testing auf true setzen. Jegliche Eingabe (z.B. Enter) löst nun den Hebelarm aus, wenn für den zweiten Sensor ein Wert ermittelt wurde. 
 * Die Ausgabe beinhaltet dann den Wert des ersten Peaks, den Wert des zweiten Peaks und den zeitlichen Versatz von zweitem Peak und Auslösen des Hebelarms. 
 * Wenn nach 3 Sekunden nach Erhalten eines Wertes für zweiten Sensor keine Eingabe festgestellt wurde, wird der Zeitpunkt auf -1 gesetzt. 
 * Dies ist somit unserer Fehlerfall/ Wert für den Fall, dass das Flugzeug zu hoch ist um es zu treffen/ nicht ausgelöst wurde. 
 * Während auf eine Eingabe gewartet wird, werden die Sensordaten nicht ausgewertet.
 */
void loop() {
    //read_dual_sensors();
    if(!erwarteFeuer) {
    peaksBerechnenUndAusgeben();
    } else if(testing){ 
      if(Serial.read()!=-1) {
      Serial.println("ZEIT:");
      ticks = mainclk.getTicks();
      Serial.println(ticks);
      Serial.println();
      ticks = 0;
      feuer();
      erwarteFeuer = false;
    }
    if(mainclk.getTicks() > 3000) {
      Serial.println("ZEIT:");
      ticks = -1;
      Serial.println(ticks);
      Serial.println();
      erwarteFeuer = false;
    }
    }
}

/*
 * Methode zum Laden des Hebelarms. 
 */
void laden(){
  if(!ready){
    servo.write(30); //Ladeposition
    delay(300);
    servo.write(110); //Ruheposition
    ready = true;
  }

}

/* Methode zum Auslösen des Hebelarms. Wenn der Hebelarm ready ist (wird nur gesetzt wenn davor geladen wurde bzw. das Programm gestartet ist, da hierbei davon ausgegangen wird,
 * dass der Hebelarm geladen ist) wird er ausgelöst. Dafür wird der Servo auf verschiedene Werte gesetzt und jeweils kurz gewartet, sodass die einzelnen Positionen auch erreicht
 * werden. Direkt nach dem Feuern soll gleich wieder geladen werden. 
 */
void feuer(){
  if(ready){
    servo.write(140); // Feuerposition
    delay(450);
    servo.write(110); //Ruheposition
    delay(200);
    ready = false;
    erwarteFeuer = false;
    laden();
  }
}

/*
 * Methode zum Berechnen und Ausgeben der Peaks der Sensoren.
 */
void peaksBerechnenUndAusgeben() {
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

// Wir warten eine Weile bis wir den nächsten Peak detekten. Damit können Messfehler dezimiert werden (und es wird nur wie gewünscht ein Peak ermittelt).
  if(millis() - millisErsterSensor > 500) {
    // Sofern wir noch geringere Werte ermitteln, haben wir den Peak noch nicht erfasst und müssen die Werte noch weiter auslesen.
  if(measure1.RangeMilliMeter < valueFirstSensor) {
    valueFirstSensor = measure1.RangeMilliMeter;
    // Sofern dies nicht mehr der Fall ist haben wir einen Peak erkannt. Wenn dieser Wert nun kleiner als ein je nach Plazierung und Aufbau gewählter Threshold ist, 
    // wird dies als das Flugzeug gesehen. Für unseren Aufbau hat sich 335 als sinnvoll erwiesen. Nachvollziehbar sind die Werte über das Beispielprojekt von
    // Adafruit (Adafruit_VL53l0X dual) oder über die Ausgabe mit der Methode read_dual_sensors, welche aktuell in der Loop auskommentiert ist. 
    // Beide bieten die Möglichkeit zu messen, welche Werte für die Peak-Detektion gewählt werden müssen.
  } else if(valueFirstSensor < 335){
      Serial.println("ValueFirstSensor: "); 
      Serial.println(valueFirstSensor);
      // Wir setzen den Wert zurück, sodass ein neuer Peak erfasst werden kann.
    // Hier wird der Zeitpunkt des ersten Peaks erfasst. Dieser wird aktuell nicht benötigt, kann aber in weiteren Arbeiten als Parameter für die KI miteinbezogen werden.
    // Dafür entweder wie hier die Millisekunden (seit Start des Programms) oder über den MainClock (Timer) mit getTicks() die Anzahl an vergangenen Ticks 
    // (seit Start des Timers bzw. letztem Reset mit resetTicks()) nutzen.
    millisErsterSensor = millis();
  }
  }
  // Gleiches bzgl. dem Warten vorm nächsten Detektieren gilt wie für Sensor 1 auch für Sensor 2.
  if(millis() - millisZweiterSensor > 500) {
    // Die Kriterien für das Erkennen eines Peaks sind analog zu Sensor 1.
  if(measure2.RangeMilliMeter < valueSecondSensor) {
    valueSecondSensor = measure2.RangeMilliMeter;
    // Einziger Unterschied ist der Wert, ab welchem eine Abweichung als Peak gesehen wird. Durch die Platzierung der Sensoren ist dieser Wert um 20 niedriger als bei Sensor 1.
  } else if(valueSecondSensor < 315){
      Serial.println("valueSecondSensor: "); 
      Serial.println(valueSecondSensor);
      erwarteFeuer = true;
      // Hier wird die ermittelte Gerade genutzt und je nach dem entweder gefeuert (sofern ein Wert ermittelt werden konnte) oder nichts getan (z.B. 
      // wenn das Flugzeug zu hoch fliegt). Für Testdaten den bool testing auf true setzen. Aktuell wird die Methode mit nur den Daten vom zweiten Sensor
      // benutzt. Für das Nutzen der anderen Methode einfach die zweite If Bedingung einkommentieren und statt der ersten nutzen.
      if(!testing) {
      if(warteDelay(valueSecondSensor)) {
      //if(warteDelay2(valueFirstSensor, valueSecondSensor)) {
      feuer();
      } else {
        erwarteFeuer = false;
      }
      }
      valueFirstSensor = 1000;
      valueSecondSensor = 1000;
      // Hier werden die Ticks der MainClock zurückgesetzt. Dies hat zur Folge, dass wir für das Ermitteln von Testdaten beim Auslösen nur noch den Zeitpunkt auslesen müssen 
      // und direkt den zeitlichen Abstand zwischen zweitem Peak und Auslösen des Hebelarms erhalten. Beim Nutzer der zeitlichen Information des ersten Peaks sollte dies evtl.
      // direkt beim ersten Peak genutzt werden und dann dementsprechend beim zweiten Peak und beim Auslösen die Anzahl an Ticks ausgewertet werden. 
      mainclk.resetTicks();
      millisZweiterSensor = millis();
  }
  }
  
}

/*
 * Berechnet den delay vom Zeitpunkt des zweiten Peaks bis zum Auslösen des Hebelarms.
 * Hier wird die Gerade genutzt, welche zuvor mit den Messwerten berechnet wurde. Bisher werden nur die Werte von Sensor Zwei (dem rechten Sensor) genutzt.
 * Hier kann zukünftig die KI und das Training eingesetzt werden.
 * Aktuell wird jeder negative Wert als nicht feuern interpretiert (siehe auch in der Loop).
 * Zudem wird die berechnete Zeitspanne dann auch noch gewartet.
 */
bool warteDelay(int valueSensorZwei) {
  int del = (85* valueSensorZwei) / 100 - 105;
  if(del <= 0) {
    return false;
  } else {
    delay(del);
    return true;
  }
 
}
/*
 * Ähnlich wie die vorangegangene Methode ist auch diese dafür da, den zeitlichen Versatz zwischen zweitem Peak und Auslösen des Hebelarms zu berechnen und abzuwarten.
 * Dafür wird nun eine multiple Regression mit zwei Parametern genutzt. So können die Werte beider Sensoren in die Berechnung mit einfließen. 
 * Die Werte wurden, genauso wie bei warteDelay() mithilfe der Testdaten im Excel Sheet berechnet (vgl. Doku).
 */
bool warteDelay2(int valueSensorEins, int valueSensorZwei) {
  int del = ((1113 * valueSensorEins) / 1000) - ((950 * valueSensorZwei) / 1000) + 101;
  if(del <= 0) {
    return false;
  } else {
    delay(del);
    return true;
  }
 
}

/*
 * Methode zum mathematischen Berechnen des Zeitpunkts zum Auslösen des Hebelarms.
 */
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
