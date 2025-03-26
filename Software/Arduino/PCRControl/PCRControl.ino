#include "TemperatureSensor.hpp"
#include "PID.hpp"
#include <math.h>

// version is year.month.date.revision
#define SOFTWARE_VERSION "2021.3.5.4"

const int inA = 13; // pin connected to INA on VHN5019
const int inB = 12; // pin connected to INB on VHN5019
const int ppwm = 11; // pin connected to PWM on VHN5019
const int fpwm = 10; // pin connected to PWM on fan
const int thermP = A0; // pin connected to block thermal resistor network
const int LidP = A1; // pin for thermal resistor connected to lid
const int cPin = A2; // for current recording
const int ssr = 9; // solid state relay signal

bool pPower = false; // software peltier on/off
bool lPower = false; // software lid on/off

bool verboseState = false; // spam serial with state every loop?
bool verbosePID = false;  // spam serial with target and current temperature?

double peltierPWM = 0; // the PWM signal to be sent to current drivers for peltier
int limitPWMH = 255;
int limitPWMC = 255;

float avgPTemp = 0; // last average for peltier temperature
int avgPTempSampleSize = 100; // sample size for peltier temperature moving average

float avgPPWM = 0; // last average for peltier PWM
int avgPPWMSampleSize = 2; // sample size for peltier PWM moving average

double targetPeltierTemp = 29; // the temperature the system will try to move to, in degrees C
double currentPeltierTemp; // the temperature currently read from the thermistor connected to thermP, in degrees C

double currentLidTemp; 
double LastLidTemp;

// setup peltier temperature sensor
TemperatureSensor peltierT(thermP);
TemperatureSensor LidT(LidP); // setup for thermistor temp

// setup peltier PID
PID peltierPID(10, 0.01, 1000000);

void setup() {
  // setup serial
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // setup pins
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(ppwm, OUTPUT);
  pinMode(fpwm, OUTPUT);
  pinMode(thermP, INPUT);
  pinMode(LidP, INPUT);
  pinMode(ssr, OUTPUT);

  // set initial pin state to off
  digitalWrite(inA, LOW);
  digitalWrite(inB, LOW);
  analogWrite(ppwm, 0);
  analogWrite(fpwm, 0);
  digitalWrite(ssr, LOW);
}

// checks UART serial for any commands and executes them
void handleSerialInput() {
  if (Serial.available() > 0) {
    String incomingCommand = Serial.readString();
    incomingCommand.trim();  // remove any extra spaces or newlines

    if (incomingCommand == "whoami") { // print out software ID
      Serial.print("FLC-PCR software version: ");
      Serial.print(SOFTWARE_VERSION);
      Serial.print("\n");
    }
    if (incomingCommand == "verbose") { // toggle sending current temp, pwm and current lid temp every loop
      verboseState = !verboseState;
    }
    if (incomingCommand == "pid") { // toggle sending target temp, current temp and pwm every loop
      verbosePID = !verbosePID;
    }
    if (incomingCommand == "d") { // request a single sample of the current temp, pwm and lid temp
      Serial.print(avgPTemp);
      Serial.print(" ");
      Serial.print(avgPPWM);
      Serial.print(" ");
      Serial.print(currentLidTemp);
      Serial.print("\n");
    }
    if (incomingCommand == "state") { // check whether the peltier is on or off
      Serial.print(pPower);
      Serial.print("\n");
    }
    if (incomingCommand == "offl") { // turn off lid
      lPower = false;
    } else if (incomingCommand == "offp") { // turn off peltier
      pPower = false;
    } else if (incomingCommand == "off") { // turn off both lid and peltier
      pPower = false;
      lPower = false;
    }
    if (incomingCommand == "onl") { // turn on the lid
      lPower = true;
    } else if (incomingCommand == "onp") { // turn on the peltier
      pPower = true;
      peltierPID.reset();
    } else if (incomingCommand == "on") { // turn on both lid and peltier
      peltierPID.reset();
      pPower = true;
      lPower = true;
    }
    if (incomingCommand.startsWith("kp")) { // set proportional gain
      peltierPID.setKp(incomingCommand.substring(2).toFloat());
    }
    if (incomingCommand.startsWith("ki")) { // set integral gain
      peltierPID.setKi(incomingCommand.substring(2).toFloat());
    }
    if (incomingCommand.startsWith("kd")) { // set derivative gain
      peltierPID.setKd(incomingCommand.substring(2).toFloat());
    }
    if (incomingCommand.startsWith("pt")) { // set peltier temperature
      peltierPID.reset();
      targetPeltierTemp = incomingCommand.substring(2).toFloat();
    }
    if (incomingCommand.startsWith("pia")) { // set size of peltier temperature low pass filter
      avgPTempSampleSize = incomingCommand.substring(3).toInt();
    }
    if (incomingCommand.startsWith("poa")) { // set size of pwm low pass filter
      avgPPWMSampleSize = incomingCommand.substring(3).toInt();
    }
    if (incomingCommand.startsWith("plc")) { // set the max pwm for cooling
      limitPWMC = incomingCommand.substring(3).toInt();
    }
    if (incomingCommand.startsWith("plh")) { // set the max pwm for heating
      limitPWMH = incomingCommand.substring(3).toInt();
    }
  }
}

void loop() {
  handleSerialInput();

  currentLidTemp = LidT.getTemp(); // read lid temp
  currentPeltierTemp = peltierT.getTemp(); // read peltier temp

  if (isnan(currentPeltierTemp) || isinf(currentPeltierTemp)) { // reset nan and inf values
    currentPeltierTemp = avgPTemp;
  }

  // 3/2/2021 temperature calibration
  currentPeltierTemp = 1.1201 * currentPeltierTemp - 3.32051;

  avgPTemp = ((avgPTempSampleSize - 1) * avgPTemp + currentPeltierTemp) / avgPTempSampleSize; // average
  peltierPWM = peltierPID.calculate(avgPTemp, targetPeltierTemp); // calculate pid and set to output
  peltierPWM = constrain(peltierPWM, -limitPWMC, limitPWMH); // clamp output between -255 and 255

  if (isnan(peltierPWM) || isinf(peltierPWM)) { // reset nan and inf values
    peltierPWM = avgPPWM;
  }

  avgPPWM = ((avgPPWMSampleSize - 1) * avgPPWM + peltierPWM) / avgPPWMSampleSize; // average

  // print out verbose data to serial if set
  if (verboseState) {
    Serial.print(avgPTemp);
    Serial.print(" ");
    Serial.print(avgPPWM);
    Serial.print(" ");
    Serial.print(currentLidTemp);
    Serial.print("\n");
  }
  if (verbosePID) {
    Serial.print(avgPTemp);
    Serial.print(" ");
    Serial.print(targetPeltierTemp);
    Serial.print(" ");
    Serial.print(avgPPWM);
    Serial.print("\n");
  }

  // lid control
  if (lPower) {
    if (currentLidTemp < 70) { 
      digitalWrite(ssr, HIGH);
    } else {
      digitalWrite(ssr, LOW);
    }
  } else {
    digitalWrite(ssr, LOW);
  }

  // peltier control
  if (!pPower || currentPeltierTemp > 150) { // peltier on, shut down if over 150C
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(fpwm, 225);
    analogWrite(ppwm, 0);

    peltierPID.reset();
    return;
  } else { // peltier on
    analogWrite(ppwm, constrain(abs(peltierPWM), 0, 255)); // Ensure PWM is within 0-255
    analogWrite(fpwm, constrain(255, 0, 255)); // Ensure fpwm is in range 0-255

    if (peltierPWM > 0) {
      digitalWrite(inA, HIGH);
      digitalWrite(inB, LOW);
    } else {
      digitalWrite(inA, LOW);
      digitalWrite(inB, HIGH);
    }
  }
}

