#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS A0 // Data wire is pUgged into port A0 (14) on the Arduino
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
int numberOfDevices; // Number of temperature devices found
//Def pins for motors
int MotorL = 2; // Motor turn to the right
int MotorR = 3; // Motor turn to the left
int MotorD = 5; // Motor turn down
int MotorU = 6; // Motor turn up
int Pulse = 7; // Controll laser
int Buzzer = 11; // Buzz if something is wrong (alarm)
// Define pins for leds to visualize the turning of the motors and if the laser is on
int Led_Left = A5; // Turn on left led if motor turns mirror to the left
int Led_Down = A4; // Turn on lower led if motor turns mirror down
int Led_Right = A3; // Turn on right led if motor turns mirror to the right
int Led_Up = A2; // Turn on left upper if motor turns mirror up
int Led_Laser = 9; // Turn on led if pulse signal from Arduino is high (laser is on)

bool alarm = 0; // T to high or lost connection => buzzer on
float T[4] = {0}; // Save current temperature
float dT[4] = {0}; // Save change in T

int ExpectedNumberOfDevices = 4; // If not equal to actual amout of found devices turn on buzzer
float turnDifdT = 0; // Difine the minimum difdT (threshold) to turn a motor
float turnTime[2] = {800,800}; // Turn motors turnTime amount of ms
float turnTthreshold = 0.5; // Temperature difference threshold between two plates as part of if statement to turn motor
float difdT;  // Difference between diagonal dT
int maxPos; // Plate that heats the fastest

String pos[4] = {"Left", "Down", "Right", "Up"}; //Correlate the the sensor to the motor
char turnMotor; //Define the direction the motor has to turn

int lastTurnDir[2]; //Remember the last turned direction, change of direction => turnTime/2 
bool turnedLastIteration[2] = {false, false}; //Somethimes the system detects that it is heating up over multiple iterations (pressed PULSE for to long), to combat only turn motors if plates cooled down a bit
int counterSameTurnDir[2] = {0, 0}; //Save the amount of time the same plate heats the fastest sequentially

void setup(void) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //Def outputs
  pinMode (Buzzer, OUTPUT);
  pinMode (Pulse, OUTPUT);
  pinMode (MotorR, OUTPUT);
  pinMode (MotorL, OUTPUT);
  pinMode (MotorU, OUTPUT);
  pinMode (MotorD, OUTPUT);
  LowMotors();

  
  pinMode (Led_Left, OUTPUT);
  pinMode (Led_Down, OUTPUT);
  pinMode (Led_Right, OUTPUT);
  pinMode (Led_Up, OUTPUT);
  pinMode (Led_Laser, OUTPUT);
  
  
  // start serial port
  Serial.begin(9600);
  Serial.println("Time(0.1s):,T_Left:,T_Right:,T_Down:,T_Up:");
  //Serial.println("T_Left:,20*dT_Left:,T_Right:,20*dT_Right:,T_Down:,20*dT_Down:,T_Up:,20*dT_Up");
  
  // Start up the library
  sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  //Display that there are not 4 sensors connected
  while (numberOfDevices != ExpectedNumberOfDevices) 
  {
    //Serial.println("Only found " + String(numberOfDevices) + " sensors, expeced " + String(ExpectedNumberOfDevices) + "!!!!!");
    delay(250);
    sensors.begin();
    numberOfDevices = sensors.getDeviceCount();
  }
  
  // Locate devices on the bus
  //Serial.print("Locating devices...");
  //Serial.println("Found " + String(numberOfDevices) + " devices.");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {      
      //Serial.println("Found device " + String(i) + " with address: " );
      //printAddress(tempDeviceAddress);
      //Serial.println();
    } else {
      Serial.print("Found ghost device at " + String(i) + " but could not detect address. Check power and cabling");
    }
  }

  sensors.requestTemperatures(); //Send the command to get temperatures

  for(int i=0;i<numberOfDevices; i++) {
    T[i] = sensors.getTempCByIndex(i); //Write referance temperatures in T
  }
}

void loop(void) {
  alarm = false;

  Serial.print(String(millis()/100) + ",");
  
  GetT(0); //Get temperature of sensor 0 and 2
  GetdifdT(0); //Get temperature differance of sensor 0 and 2
  TurnMotor(0); //Turn motor if necessary
  
  GetT(1); //Get temperature of sensor 1 and 3
  GetdifdT(1); //Get temperature of sensor 1 and 3
  TurnMotor(1); //Turn motor if necessary
  /*
  if (turnTime[0] < 0.1 && turnTime[0] < 0.1){
    LaserCentered();
  }*/
  //Let the plates cool down a short while so the dT is amplified
  delay(750);
  digitalWrite (Led_Laser, LOW); // Turn red led off
  digitalWrite (Pulse, LOW); // Turn laser off
  delay(2000);
  digitalWrite (Pulse, HIGH); // Turn laser on
  digitalWrite (Led_Laser, HIGH); // Turn red led on
  
  Serial.println();
}

void GetT(int place0){
  sensors.requestTemperatures(); // Send the command to get temperatures

  //Save temperature in T[] and change dT[]
  for(int i=place0;i<numberOfDevices; i+=2) {
    float Ttemp = sensors.getTempCByIndex(i);
    if(Ttemp > 10){
      //Serial.println("Temp " + String(i) + ": " + String(Ttemp) + "°C");
      dT[i] = Ttemp - T[i]; //Save temperature differance (current temperature - previous temperature)
      T[i] = Ttemp; //Save current temperature

      //Safety: connection error
      while(Ttemp == 85.00){
        Serial.println("Connection error (+ wire) of sensor " + String(i));
        delay(250);
        Ttemp = sensors.getTempCByIndex(i);
      }
      //Safety: Sensor is above 60°C => alarm and turn off laser
      if(T[i] >= 60){
        while(T[i] >= 50){
          digitalWrite (Led_Laser, LOW); // Turn red led off
          digitalWrite (Pulse, LOW); // Turn laser off
          alarm = true;
          digitalWrite (Buzzer, HIGH);
          //Serial.println("ERROR!!!: Plate " + String(pos[i]) + " overheating with " + String(T[i]) + "°C");
          sensors.requestTemperatures(); // Send the command to get temperatures
          T[i] = sensors.getTempCByIndex(i);
          delay(250);
  
          if(T[i] < 50){
            alarm = false;
            digitalWrite (Buzzer, LOW); // Tunr alarm off
            //Serial.println("\t\t\t START PULSE");
            digitalWrite (Led_Laser, HIGH); // Turn red led on
            digitalWrite (Pulse, HIGH); // Turn laser on
          }
        }
      }

      //Serial.println("\t\t\t START PULSE");
      digitalWrite (Led_Laser, HIGH); // Turn red led on
      digitalWrite (Pulse, HIGH); // Turn laser on after safety checks
    }
      
    else {
      digitalWrite (Led_Laser, LOW); // Turn red led off
      digitalWrite (Pulse, LOW); // Turn off laser
      alarm = true; //Connection error
      Serial.println("Connection error (temperature to low) sensor " + String(i));
    }
  }
  //Print temperature / change of temperature for each sensor
  for(int i=place0;i<numberOfDevices;i+=2){
    //Serial.print(String(pos[i]) + ": " + String(T[i]) + "°C, " + String(dT[i]) + "\t");
    Serial.print(String(T[i]) + ",");
    //Serial.print(String(dT[i]) + ",");
  }
  //Serial.print(String(abs(dT[place0] - dT[place0+2])) + ",");
}

void GetdifdT(int place0){
  //Save the place of the fastest heating plate in maxPos
  if(dT[place0] >= dT[place0+2]) { maxPos = place0; }
  else { maxPos = place0+2; }
  //If both plats are cooling set difdT 0 else take the absolute value of the difference
  if(dT[maxPos] >= 0){ difdT = abs(dT[place0] - dT[place0+2]); } //If maxPos gets hotter or stays the same
  else{ difdT = -10;} //Make sure the mirror does not turn if both are cooling down
}


void LowMotors(){
  digitalWrite (MotorR, LOW);
  digitalWrite (MotorL, LOW);
  digitalWrite (MotorU, LOW);
  digitalWrite (MotorD, LOW);
}

void TurnMotor(int place0){
  //When a difdT (= abs(dT[one plate] - dT[opposing plate])) is bigger than a set value and T[one plate] is more than T[opposing plate]+0.5 °C
  if(difdT >= turnDifdT && T[maxPos] > T[(maxPos+2)%4]+turnTthreshold){
    //digitalWrite (Buzzer, HIGH);
    
    //if (turnedLastIteration[place0] == false){
        if(lastTurnDir[place0] != maxPos){ //Devide turnTime if fastest heating plate is differant from last time
          turnTime[place0] /= 2;
          //Serial.println("\t\t TurnTime " + String(place0) + ": " + String(turnTime[place0]));
          counterSameTurnDir[place0] = 0;
        }
        else{
          counterSameTurnDir[place0]++;

          if (counterSameTurnDir[place0] > 2){
            //Serial.println("Plate " + String(maxPos) + " has been the fastest heating for " + String(counterSameTurnDir[place0]) + " itterations!!!");
          }
          if (counterSameTurnDir[place0] > 4){
            digitalWrite (Led_Laser, LOW); // Turn red led off
            digitalWrite (Pulse, LOW); // Turn off laser
            Serial.println("PLATE " + String(maxPos) + " KEEPS HEATING THE FASTEST!!!");
            Serial.println("SHUTTING DOWN FOR SAFETY");
            delay(100);
            digitalWrite (Buzzer, LOW);
            for (int i=0; i<15; i++){
              digitalWrite (Buzzer, HIGH);
              delay(50);
              digitalWrite (Buzzer, LOW);
              delay(50);
            }
            delay(5000);
            sleep_mode();
          }
        }
        lastTurnDir[place0] = maxPos;
        //Serial.println();
        if(maxPos == 0){
          //Serial.println("Turn_L, T: " + String(T[0]));
          turnMotor = MotorL;
          digitalWrite (Led_Left, HIGH);
        }
        else if(maxPos == 1){
          //Serial.println("Turn_D, T: " + String(T[1]));
          turnMotor = MotorD;
          digitalWrite (Led_Down, HIGH);
        }
        else if(maxPos == 2){
          //Serial.println("Turn_R, T: " + String(T[2]));
          turnMotor = MotorR;
          digitalWrite (Led_Right, HIGH);
       }
       else if(maxPos == 3){
          //Serial.println("Turn_U, T: " + String(T[3]));
          turnMotor = MotorU;
          digitalWrite (Led_Up, HIGH);
        }

      digitalWrite (turnMotor, HIGH);
      delay(turnTime[place0]); //Turn motor on for turnTime ms
      LowMotors();
      //Serial.println("Stop motor turning");
      turnedLastIteration[place0] = true;
      delay(1000);
      LowLeds();
    //}
  }
  else {
    turnedLastIteration[place0] = false;
    if (alarm == false){
      //Serial.print("Alarm: " + String(alarm));
      digitalWrite (Buzzer, LOW);
    }
  }
}


void LowLeds(){
  digitalWrite (Led_Left, LOW);
  digitalWrite (Led_Down, LOW);
  digitalWrite (Led_Right, LOW);
  digitalWrite (Led_Up, LOW);
}


// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
      //Serial.print(deviceAddress[i], HEX);
  }
}

void LaserCentered(){
  digitalWrite (Led_Laser, LOW); // Turn red led off
  digitalWrite (Pulse, LOW); // Turn off laser
  Serial.println("LASER IS CENTERED");
  delay(100);
  digitalWrite (Buzzer, LOW);
  for (int i=0; i<15; i++){
    digitalWrite (Buzzer, HIGH);
    delay(100);
    digitalWrite (Buzzer, LOW);
    delay(100);
  }
  delay(5000);
  sleep_mode();
}
