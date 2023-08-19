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

float T[4] = {0}; // Save current temperature
float dT[4] = {0}; // Save change in T

int ExpectedNumberOfDevices = 4; // If not equal to actual amout of found devices turn on buzzer
int turnTime = 300;
float turnDifdT = 0.15; // Difine the minimum difdT (threshold) to turn a motor
float difdT[2];  // Difference between diagonal dT

char turnMotor; //Define the direction the motor has to turn

void setup(void) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //Def outputs
  pinMode (Buzzer, OUTPUT);
  pinMode (Pulse, OUTPUT);
  pinMode (MotorR, OUTPUT);
  pinMode (MotorL, OUTPUT);
  pinMode (MotorU, OUTPUT);
  pinMode (MotorD, OUTPUT);
  pinMode (A5, OUTPUT);
  pinMode (A4, OUTPUT);
  pinMode (A3, OUTPUT);
  pinMode (A2, OUTPUT);
  LowMotors();
  
  // start serial port
  Serial.begin(9600);
  
  // Start up the library
  sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  //Display that there are not 4 sensors connected
  while (numberOfDevices != ExpectedNumberOfDevices) 
  {
    Serial.println("Only found " + String(numberOfDevices) + " sensors, expeced " + String(ExpectedNumberOfDevices) + "!!!!!");
    delay(250);
    sensors.begin();
    numberOfDevices = sensors.getDeviceCount();
  }
  
  // Locate devices on the bus
  Serial.print("Locating devices...");
  Serial.println("Found " + String(numberOfDevices) + " devices.");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {      
      Serial.println("Found device " + String(i) + " with address: " );
      printAddress(tempDeviceAddress);
      Serial.println();
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
  GetT(0); //Get temperature of sensor 0 and 2
  GetdifdT(0); //Get temperature differance of sensor 0 and 2
  TurnMotor(0); //Turn motor if necessary
  
  if (difdT[0] > turnDifdT){
    LaserOffset();
  }
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
      while(T[i] >= 60){
        digitalWrite (Pulse, LOW); // Turn laser off
        digitalWrite (Buzzer, HIGH);
        Serial.println("ERROR!!!: Plate overheating with " + String(T[i]) + "°C");
        sensors.requestTemperatures(); // Send the command to get temperatures
        T[i] = sensors.getTempCByIndex(i);
        delay(250);

        if(T[i] < 60){
          digitalWrite (Buzzer, LOW); // Tunr alarm off
          //Serial.println("\t\t\t START PULSE");
          digitalWrite (Pulse, HIGH); // Turn laser on
        }
      }

      //Serial.println("\t\t\t START PULSE");
      digitalWrite (Pulse, HIGH); // Turn laser on after safety checks
    }
      
    else {
      digitalWrite (Pulse, LOW); // Turn off laser
      Serial.println("Connection error (temperature to low) sensor " + String(i));
    }
  }

  Serial.print("difdT: " + String(abs(dT[place0] - dT[place0+2])));
  Serial.println();
}

void GetdifdT(int place0){
  difdT[place0] = abs(dT[place0] - dT[place0+2]);
}


void LowMotors(){
  digitalWrite (MotorR, LOW);
  digitalWrite (MotorL, LOW);
  digitalWrite (MotorU, LOW);
  digitalWrite (MotorD, LOW);
}

void TurnMotor(int place0){
    digitalWrite (MotorR, HIGH);
    digitalWrite (A3, HIGH);
    delay(turnTime); //Turn motor on for turnTime ms
    LowMotors();
    Serial.println("Stop motor turning");
    digitalWrite (A3, LOW);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}

void LaserOffset(){
  digitalWrite (Pulse, LOW); // Turn off laser
  Serial.println("LASER IS OFFSET");
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
