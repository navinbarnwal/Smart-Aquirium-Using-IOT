// -ve  GnD
// A0 positive
#include <LiquidCrystal.h> //lib for LCD

LiquidCrystal lcd(12, 11, 5, 4, 3, 2); 

int Contrast = 10;

//LCD   Ard-UNO
//pin 1 to gnd
//pin 2 5v
//pin 3 6
//pin 4 12
//pin 5 gnd
//pin 6 11
//pin 11 5
//pin 12 4
//pin 13 3
//pin 14 2
//pin 15 5v
//pin 16 gnd

#include <Servo.h>  //add '<' and '>' before and after servo.h
 
int servoPin = 8;
 
Servo servo;  
 
int servoAngle = 0;   // servo position in degrees
 

#include <SimpleDHT.h>

// for DHT11, 
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
int pinDHT11 = 7;
SimpleDHT11 dht11(pinDHT11);


#define SensorPin A0 //pH meter Analog output to Arduino Analog Input 0
  static unsigned long samplingTime = millis();
#define Offset 0.00 //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40 //times of collection

int pHArray[ArrayLenth]; //Store the average value of the sensor feedback
int pHArrayIndex = 0;
void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  Serial.println("pH meter experiment!"); //Test the serial monitor
  servo.attach(servoPin);

  //Lcd Display code Starts Here
  analogWrite(6, Contrast);
  lcd.begin(16, 0);
  lcd.setCursor(0, 1);
  lcd.print(" Weight ");
  lcd.print(" Measurement ");
  delay(1000);
  //lcd.clear(); 
  //Lcd Display code ends Here

}

void loop() {

  //Ph Sensor Code.....
  static unsigned long printTime = millis();
  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval) {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - printTime > printInterval) {
    Serial.print("Voltage:");
    Serial.print(voltage, 2);
    Serial.print(" pH value: ");
    Serial.println(pHValue, 2);
    digitalWrite(LED, digitalRead(LED) ^ 1);
    printTime = millis();

  }

  //PH sensor code ends here

  // TEMPERATUE sensor start here...
  Serial.println("=================================");
  
  // read without samples.
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT11 failed, err="); Serial.println(err);delay(1000);
    return;
  }
  
  Serial.print("Temparate and Humidity : -  ");
  Serial.print((int)temperature); Serial.print(" *C, "); 
  Serial.print((int)humidity); Serial.println(" H");
  
  // DHT11 sampling rate is 1HZ.
  delay(1500);

  //TEMP sensor ends here

  // Servo Motor SG90 COde starts here
  //control the servo's direction and the position of the motor
  /***
   servo.write(10);      // Turn SG90 servo Left to 45 degrees
   delay(2000);          // Wait 1 second
   servo.write(140);      // Turn SG90 servo back to 90 degrees (center position)
   delay(2000);          // Wait 1 second
  ***/
 
//end control the servo's direction and the position of the motor
 
 
//control the servo's speed  
 /***
//if you change the delay value (from example change 50 to 10), the speed of the servo changes
  for(servoAngle = 0; servoAngle < 180; servoAngle++)  //move the micro servo from 0 degrees to 180 degrees
  {                                  
    servo.write(servoAngle);              
    delay(50);                  
  }
 
  for(servoAngle = 180; servoAngle > 0; servoAngle--)  //now move back the micro servo from 0 degrees to 180 degrees
  {                                
    servo.write(servoAngle);          
    delay(10);      
  }
  //end control the servo's speed  
  ***/
  // Servo Motor SG90 Code ends here
}



//Manual Function for PH Sensor starts here
double avergearray(int* arr, int number) {

  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  }
  else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min; //arr<min
        min = arr[i];
      }
      else {
        if (arr[i] > max) {
          amount += max; //arr>max
          max = arr[i];
        }
        else {
          amount += arr[i]; //min<=arr<=max
        }
      }
    }
    avg = (double)amount / (number - 2);
    
  }
  return avg;
}
//Manual Function for PH Sensor ends here
