/*

Michael Britt-Crane 2014.7.17
This program will run the stoc cGlide glider with no advanced add-ons. 
learn more at http://cGlide.com there you will find all of the source files, a bill of materials, instructions, and lessons. 

*/

#include <IRremote.h>                // include the IRremote library: http://github.com/shirriff/Arduino-IRremote
#include <Servo.h>                   // include the stock "Servo" library for use in this sketch
Servo myservo;                       // create a new Servo object called myservo

// Constants
static int minCoast =  1000;         // if the pot is turned all the way to the counter-clockwise the glider will coast for 1 seccond
static int maxCoast = 20000;         // if the pot is turned all the way to the clockwise the glider will coast for 10 secconds
static byte servoDiveCommand = 0;    // this is the angle value that the dive method sends to the servo
static byte servoRiseCommand = 180;  // this is the angle value that the rise method sends to the servo

static int riseDriveTime = 13000;   //18000; // fornew lead plunger 

// Pins 
static byte SERVO_PIN = 10;          // the pin that the "continuous rotation servo" is attached to, this motor drives the buoyancy engine
static byte DIVE_STOP = 11;          // the pin that the dive limmit switch (round push button) is attached to
static byte POT_PIN = A3;            // the pin that the wiper of the little orange trim pot is attached to
static byte RECV_PIN = 2;            // IR reciever signal pin
static byte IR_GND = 3;              // IR middle pin, ground
static byte IR_PWR = 4;              // IR power pin

static byte RED_LED = 9;             // these are the three pins that the RED
static byte GREEN_LED = 6;           //                                   GREEN
static byte BLUE_LED = 5;            //                               and BLUE LED cathodes are attached to
static byte LED_BASE = 7;            // this is the pin that the "common anode" of the RGB LED is attached to

static byte tempPin = A0;
static byte pressurePin = A1;

//------Sensors--------
float voltage = 0;
double SensorValueT = 0; //Save Sensor input Voltage
double SensorValueD = 0; //Save Sensor input Voltage
float ResultkPa = 0; // Save Result in kPa
float ResultBar = 0; // Save Result in Bar
float ResultPsi = 0; // Save Result in Psi
float ResultDepth = 0; // Save Result in Depth
float degreesC= 0;
char depthStr[10];
char tempStr[10];

//------Logger---------
int sampleInt = 4000;
int milsec = 0;
int i = 0;

// IR definitions
IRrecv irrecv(RECV_PIN);
decode_results results;
#define PAUSE 0xFD807F               // Pauses Buoyancy Engine Cycling
#define LEFT  0xFD10EF               // When paused, joggs plunger 
#define RIGHT 0xFD50AF               // When paused, joggs plunger
#define THREE 0xFD48B7               // Runs to the end stop and then drives to mid point

//not currently in use can use all buttons
#define ONE 0xFD08F7
#define TWO 0xFD8877
#define UP 0xFDA05F
#define DOWN 0xFDB04F


void setup() {                       // begin setup method
  delay(1000);
  Serial.begin(9600);                // fire up the serial port. This allows us to print values to the serial console
  Serial.println("SeaGlide Temperature and Depth Data");
  Serial.println("Time, Temp, Depth");
  IRsetup();                         // Start the Infa-Red reciever
  pinMode(POT_PIN, INPUT);           // initialize the potentiometer, this pot will determine the coast time turn it right to coast longer
  pinMode(SERVO_PIN, OUTPUT);        // initialize the continuous rotation servo, this motor drives the buoyancy engine
  pinMode(DIVE_STOP, INPUT_PULLUP);  // initialize the dive stop switch, and turn on the internal pull-up resistor. This limmit switch is lets the Arduino know when the buoyancy engine reaches the end of its travle in the dive direction
  pinMode(RED_LED, OUTPUT);          // initialise the RED
  pinMode(GREEN_LED, OUTPUT);        //                GREEN
  pinMode(BLUE_LED, OUTPUT);         //            and BLUE pins on the LED
  pinMode(LED_BASE, OUTPUT);         // initialize the common pin of the LED 
  pinMode(tempPin, INPUT);
  pinMode(pressurePin, INPUT);
  
  // initialize RGB LED
  ledRGB_Write(0, 0, 0);             // set the R, G, & B LEDs to OFF
  digitalWrite(LED_BASE, HIGH);      // set the LED Base pin to HIGH this LED it is a common anode, providing +5V to all 3 LEDs
  readPot(POT_PIN);
  delay(50);                         // wait for 1.0 sec
}                                    // end setup method

void loop(){                    // begin main loop
  dive(0);                      // DIVE-DIVE-DIVE: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
  delayTwo(readPot(POT_PIN));      // read the pot and delay bassed on it's position, coast
  rise(riseDriveTime);          // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
  delayTwo(readPot(POT_PIN)*1.1);  // Read the pot and delay bassed on it's position, coast
}                               // end main loop

void dive(int time){                            // Dive: Run the "dive" method. This will start turning the servo to take in water & pitch the glider down
  int x = 0;
  ledRGB_Write(255, 0, 0);                      // set LED to RED to indicate that the glider is diving
  //Serial.println("diving");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on "SERVO_PIN" to the servo object so that we can command the servo to turn
  myservo.write(servoDiveCommand);              // drive servo clockwise, take in water & pull weight forward (pull counterweight & plunger towards servo, at the bow of the glider)
      unsigned long currentMillis = millis();
      long previousMillis = currentMillis;
  if (time == 0){
    while (digitalRead(DIVE_STOP) == HIGH){     // keep checking the DIVE_STOP pin to see if the button is pressed
      currentMillis = millis();
      if (checkIR()){
        myservo.attach(SERVO_PIN);              // attaches the servo on SERVO_PIN to the servo object
        myservo.write(servoDiveCommand);        // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
        ledRGB_Write(255, 0, 0);                // set LED to RED to indicate that the glider is diving
      }
        if((currentMillis - previousMillis)>(sampleInt*x)){
          Sample();
          x++;
        }
      // wait...                                // just keep checking: when the button is pressed, continue to the next line
    }
  }
  else{
    while (currentMillis - previousMillis < time && digitalRead(DIVE_STOP) ) { 
      currentMillis = millis();   
    }   
  }
  myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  //Serial.println("coasting (dive)");            // print status change to the serial port
  ledRGB_Write(255, 80, 0);                     // set LED to ORANGE to indicate that the glider is coasting in a dive
}                                               // end of method

void rise(int time){                            // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
  int x = 0;
  ledRGB_Write(0, 200, 0);                      // set LED to GREEN to indicate that the glider is rising
  //Serial.println("rising");                     // print status change to the serial port
  myservo.attach(SERVO_PIN);                    // attaches the servo on SERVO_PIN to the servo object
  myservo.write(servoRiseCommand);              // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
  unsigned long currentMillis = millis();
  long previousMillis = currentMillis;
  while (currentMillis - previousMillis < time) { 
    currentMillis = millis();  
    if (checkIR()){
      myservo.attach(SERVO_PIN);                // attaches the servo on SERVO_PIN to the servo object
      myservo.write(servoRiseCommand);          // drive servo counter-clockwise, pull weight aft (push counterweight & plunger away from servo)
      ledRGB_Write(0, 200, 0);                  // set LED to GREEN to indicate that the glider is rising
    }
    if((currentMillis - previousMillis)>(sampleInt*x)){
      Sample();
      x++;
    }
    // wait...                                  // just keep checking, when the sensor sees the edge, continue to the next line
  }
  myservo.detach();                             // stop the servo, detaches the servo on SERVO_PIN from the servo object
  //Serial.println("coasting (rise)");            // print status change to the serial port
  ledRGB_Write(0, 0, 255);                      // set LED to BLUE to indicate that the glider is coasting in a rise
}                                               // end of method

void ledRGB_Write(byte R, byte G, byte B){      // This method takes care of the details of setting a color and intensity of the RGB LED
  analogWrite(RED_LED, 255-R);                  // These are backwards because you write low values to turn these LEDs on
  analogWrite(GREEN_LED, 255-G);                // This method reverses the counterintuitive nature of the LEDs
  analogWrite(BLUE_LED, 255-B);                 // If using common anode rather than common anode LEDs remove the "255-"es
}                                               // end of method

int readPot(int potPin){                        // this method reads a potentiometer to determine the pause time
  int potValue = analogRead(potPin);            // Read the Potentiometer
  int pauseTime = map(potValue, 0, 1023, minCoast, maxCoast); // scale the value to the diveDriveTime range defined by minDriveTime & maxDriveTime
  //Serial.print("Coast Time: ");                 // print a lable to the serial port
  //Serial.println(pauseTime);                    // print the pause time value to the serial port
  return pauseTime;                             // return the pause time, an intiger (int) value
}                                               // end of method

void IRsetup(){
  irrecv.enableIRIn();
  pinMode(IR_GND, OUTPUT);
  pinMode(IR_PWR, OUTPUT);
  digitalWrite(IR_GND, 0);
  digitalWrite(IR_PWR, 1);
}

boolean checkIR(){
  if (checkPause()){
    myservo.detach();
    ledRGB_Write(9, 0, 20);
    boolean paused = true;
    while (paused){
      if (irrecv.decode(&results)) {
        if (results.value == PAUSE){
          //Serial.println("PLAY");
          delay(100);
          paused = false;
        }
        if (results.value == RIGHT){
          //Serial.println("Right");
           rise(800);          
        }
        irrecv.resume();
        if (results.value == LEFT){
          //Serial.println("Left");
          dive(800);          
        }
        irrecv.resume();
        if (results.value == TWO){
          //Serial.println("2");
          flash(2, 150);
          dive(0);
          delay(150);
          rise(riseDriveTime/2);
        }
        irrecv.resume();
      }      
    }
    ledRGB_Write(60, 50, 50);
    return true;
  }
  else{
    return false;  
  }
}

boolean checkPause(){
  if (irrecv.decode(&results)) {
    if (results.value == PAUSE) {
      //Serial.println("PAUSE");
      irrecv.resume();
      return true;
    }
    else{
      irrecv.resume();
      return false;
    }
  }
}

// not in use
void flash(int flashes, int time){
  for(int i = flashes; i > 0; i--){
    digitalWrite(13, 1);
    delay(time);
    digitalWrite(13, 0);
    delay(time);
  }
}
void Sample(){
  readTemp(tempPin);
  readDepth(pressurePin);
  Serial.print((millis() / 1000));
  Serial.print(", ");
  Serial.print(degreesC);
  Serial.print(", ");
  Serial.print(ResultDepth);
  Serial.println();
}

float readTemp(int tempPinn){
        //--------------Temp------------------
    voltage = analogRead(tempPinn);
    SensorValueT = voltage * 0.004882814;
    //degreesC = ((SensorValueT - 0.5) * 105.35)+3.4;
    degreesC = (SensorValueT-.21) *100;
    //dtostrf(degreesC, 6, 2, tempStr);
        //Serial.write(tempStr); 
        //Serial.write("T = ");
        //Serial.write(tempStr); 
        //Serial.write("");
    return degreesC;  
}

float readDepth(int pressurePinn){
        //--------------Pressure------------------
    SensorValueD = analogRead(pressurePinn);
    ResultkPa = (SensorValueD*(.00488)/(.022)+20);
    ResultBar = (ResultkPa * 0.01) - 1.0172; //multiply (1 kPa x 0.01 bar) and deduct atmospheric pressure
    ResultPsi = (ResultBar * 14.5038); //Multiply (Bar * 14.5) to get psi
    ResultDepth = (ResultPsi * 2.31 * 0.3048); //Psi to feet to meters
    ResultDepth = ((((-0.042)*(pow(ResultDepth, 2)))+(1.1575*ResultDepth)-0.0426)*100)+3; //6/9/16 calculate equation from calibrating m depth of water-polynomial
    dtostrf(ResultDepth, 6, 2, depthStr);
  
    //Serial.write("   D = ");
    //Serial.write(depthStr);
    //Serial.write("#");
    
    return ResultDepth;
}

void delayTwo(int time){                            // Rise: Run the "rise" method. This will start turning the servo to push out water & pitch the glider up
  int x = 0;
  //Serial.println("rising");                     // print status change to the serial port
    unsigned long currentMillis = millis();
  long previousMillis = currentMillis;
  while (currentMillis - previousMillis < time) { 
    currentMillis = millis();  
    if((currentMillis - previousMillis)>(sampleInt*x)){
      Sample();
      x++;
    }
    // wait...                                  // just keep checking, when the sensor sees the edge, continue to the next line
  }                // set LED to BLUE to indicate that the glider is coasting in a rise
}                                               // end of method

