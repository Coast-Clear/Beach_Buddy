

#include <timer.h>

#include <AutoPID.h>
#include <FastLED.h>


#define ENCODER_PIN_A   2
#define ENCODER_PIN_B   3
#define LED_PIN         4
#define LIMIT_PINS      5
#define PWM_A           6
#define DIRECTION_PIN   9
#define MOTOR_FAULT_A   10
#define REVERSE_DRIVE   11
#define MOTOR_FAULT_B   12

#define NUM_LEDS       4

//Create and initialize variables
int encoder0Pos = 0;
int encoder0PosLast = 0;
double pwmValue;
float voltage = 0.0;
int sum = 0;
int sample_count = 0;
int NUM_SAMPLES = 10;
CRGB leds[NUM_LEDS];
int prog1 = 0;
int prog2 = 0;
unsigned long previousMillis = 0;
int blinking = 0;
long lastTime = millis();
double RPM =0;
double setPoint;
double powerOutput = 0;
char lSig[2] = "00";

AutoPID myPID(&RPM, &setPoint, &pwmValue, -2, 2, 0.2, 0, .7);

void setup() {
  //Assign pinmodes to digital pins
  pinMode (ENCODER_PIN_A, INPUT);
  pinMode (ENCODER_PIN_B, INPUT);
  pinMode (LIMIT_PINS, INPUT);
  pinMode (REVERSE_DRIVE, INPUT_PULLUP);
  pinMode(PWM_A, OUTPUT);

  //Begin serial communications
  Serial.begin (9600);

  //Start motor at 0 power
  writeMotor(0, true);

  //Assign RPM setpoint based on potentiometer reading
  setPoint = map(analogRead(A1), 0, 1023, 40, 180);

  //Initialize LED strip object
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  //Set PID update rate (0 for default)
  myPID.setTimeStep(0);

  //Set encoder pin to run on interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateRPM, RISING);

  //Set analog references to external
  analogReference(EXTERNAL);
}

void loop() {

  //=======
  //MOTOR CONTROl
  //=======

  //Read potentiometer setpoint
  setPoint = map(analogRead(A1), 0, 1023, 40, 180);

  //Run motor in reverse if reverse button is held
  if(digitalRead(REVERSE_DRIVE) == HIGH){
    writeMotor(127, false);
    lSig[0] = '1';
  }
  //Run motor using PID control if limit switches are pressed and reverse button is not pressed
  else if(digitalRead(LIMIT_PINS) == HIGH){
    //update PID every 10ms
    if(millis()-lastTime > 10){
      RPM = encoder0Pos - encoder0PosLast;
      RPM *= 60000/256;
      double x = (millis() - lastTime);
      RPM /= -x;

      myPID.run();
      powerOutput += pwmValue;
      powerOutput = constrain(powerOutput, 0, 255);
      
      Serial.print(RPM);
      Serial.print("\t");
      Serial.print(setPoint);
      Serial.print("\t");
      Serial.println(setPoint);
      
      encoder0PosLast = encoder0Pos;

      lastTime = millis();
    }

    //Let display know the motor is running in forward
    lSig[0] = '0';
    writeMotor(constrain((int)powerOutput, -255, 255), false);
    
  } 
  //If nothing is pressed, turn motor off
  else {
    writeMotor(0, true);
    
    encoder0PosLast = encoder0Pos;
  
    lastTime = millis();

    lSig[0] = '2';
  }

  //Send error codes to display
  if(RPM < 2 && pwmValue > 200){
   
    lSig[1] = '0';
  }
  else{
    lSig[1] = '1';
  }
  
  //=======
  //DISPLAY
  //=======

  //blinking for low battery
  if ((millis() - previousMillis) > 500){
    previousMillis = millis();
    blinking++;
  }

  //Get multiple battery voltage samples
  sample_count = 0;
  sum = 0;
  while (sample_count < NUM_SAMPLES) {
    sum += analogRead(A0);
    sample_count++;
  }
  
  //Interpret voltage from battery
  voltage = ((float)sum)/((float)NUM_SAMPLES);
  voltage = voltage * (12.27 / 730);
  //Serial.println(voltage);

  //Shut off if battery dangerously low
  if (voltage < 10.5){
    leds[3] = CRGB(0, 0, 0);
    FastLED.show();
    leds[2] = CRGB(0, 0, 0);
    FastLED.show();
    leds[1] = CRGB(0, 0, 0);
    FastLED.show();
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    prog1++;
  }

  //Green if battery high
  if ((voltage >= 11.9) && (prog2 < 1) && (prog1 == 0)){
    leds[1] = CRGB(0, 255, 0);
    FastLED.show();
  }

  //Yellow if battery mid
  if ((voltage >= 11.31) && (voltage < 11.9) && (prog2 < 2) && (prog1 == 0)){
    leds[1] = CRGB(125, 125, 0);
    prog2++;
  }

  //Red if battery low
  if ((voltage >= 10.9) && (voltage < 11.31) && (prog2 < 3) && (prog1 == 0)){
    leds[1] = CRGB(255, 0, 0);
    FastLED.show();
    prog2++;
  }
  
  //blinking if battery really low
  if ((voltage < 10.9) && (voltage >= 10.5) && (prog1 == 0) && (blinking % 2 == 0)){
    leds[1] = CRGB(255, 0, 0);
    FastLED.show();
  }
  if ((voltage < 10.9) && (voltage >= 10.5) && (prog1 == 0) && (blinking % 2 == 1)){
    leds[1] = CRGB(0, 0, 0);
    FastLED.show();
  }

  //Motor in reverse
  if (lSig[0] == '1'){
    leds[3] = CRGB(0,0, 255);
    leds[2] = CRGB(0, 0, 0);
    FastLED.show();
  }
  //Motor in forward
  if (lSig[0] == '0'){
    leds[2] = CRGB(0, 0, 255);
    leds[3] = CRGB(0,0, 0);
    FastLED.show();
  }
  //Motor no signal
  if (lSig[0] == '2'){
    leds[3] = CRGB(0, 0, 0);
    leds[2] = CRGB(0, 0, 0);
    FastLED.show();
  }
  
  //No errors
  if (lSig[1] == '0'){
    leds[0] = CRGB(125, 125, 0);
    FastLED.show();
  }
  //Stalling
  if (lSig[1] == '1'){
    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
  }

  
  
}

void writeMotor(int power, boolean forward){
  if(forward){
    analogWrite(PWM_A, power);
    digitalWrite(DIRECTION_PIN, LOW);
  } else {
    analogWrite(PWM_A, power);
    digitalWrite(DIRECTION_PIN, HIGH);
  }
  
}


//If encoder wheel returns new value, update encoder reading
void updateRPM(){
  if (digitalRead(ENCODER_PIN_B) == LOW) {
    encoder0Pos--;
  } else {
    encoder0Pos--;
  }
}
