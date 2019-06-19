#include <NewPing.h>
/**********************Definations:*******************/
//1. H-Bridge
#define INPUT1 22
#define INPUT2 24
#define INPUT3 26
#define INPUT4 28
#define ENA 6
#define ENB 7
//2.RotatingBrush_MotorRelay
#define BRUSH_RELAY 48
//3.Wheels_MotorRelay
#define WHEELRELAY_FRD 50
#define WHEELRELAY_BKD 52
//4.BluetoohModule
#define rx 0
#define tx 1
//5.UltraSonics
#define TRIG_FORWARD 44
#define ECHO_FORWARD 45
#define TRIG_BACKWARD 46
#define ECHO_BACKWARD 47
//6.Encoders
#define ENCODER_RIGHT 30
#define ENCODER_LEFT 32

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(TRIG_FORWARD, ECHO_FORWARD, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG_BACKWARD, ECHO_BACKWARD, MAX_DISTANCE)
};

/*****************************************************/
/******************Global_Variables:*****************/
//1-PID_Parameters:
double kp = 0.085;
double ki = 5.5;
double kd = 0.00002;
//2-Speed_parameters:
int sv_speed = 100;     //this value is 0~255
double pv_speed = 0;
double set_speed = 0;
double e_speed = 0; //error of speed = set_speed - pv_speed
double e_speed_pre = 0;  //last error of speed
double e_speed_sum = 0;  //sum error of speed
double pwm_pulse = 0;     //this value is 0~255
//3-Encoder:
int rcount = 0;
int lcount = 0;
//4-Bluetooth_recieved_Charachters:
String mySt = "";
char myChar;
boolean stringComplete = false;  // whether the string is complete
boolean motor_start = false;
//5- Timer:
int timer1_counter; //for timer
int i = 0;
/*****************************************************/
/*******************Functions decleration:***********/
void roboForward_H (int);
void roboBackward_H (int);
void roboForward_Relay (void);
void roboBackward_Relay (void);
void roboStop (void);
void brushRotate(void);
void serialEvent(void);
void rencoder(void);
void lecnoder(void);
/*****************************************************/
void setup() {

  //PIN_MODE_SETUP:
  //1. H-Bridge
  pinMode (INPUT1, OUTPUT);
  pinMode (INPUT2, OUTPUT);
  pinMode (INPUT3, OUTPUT);
  pinMode (INPUT4, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  //2.RotatingBrush_MotorRelay
  pinMode (BRUSH_RELAY, OUTPUT);
  //3.Wheels_MotorRelay
  pinMode (WHEELRELAY_FRD, OUTPUT);
  pinMode (WHEELRELAY_BKD, OUTPUT);
  //4.BluetoohModule
  pinMode (tx, OUTPUT);
  pinMode (rx, INPUT);
  //5.UltraSonics
  pinMode(ECHO_FORWARD, INPUT);
  pinMode(TRIG_FORWARD, OUTPUT);
  pinMode(ECHO_BACKWARD, INPUT);
  pinMode(TRIG_BACKWARD, OUTPUT);
  //6.Encoders
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  /*****************************************************/
  /***************SETUP&Initialization:*****************/
  digitalWrite(ENCODER_RIGHT, HIGH);
  digitalWrite(ENCODER_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rencoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT) , lencoder, RISING);
  Serial.begin(115200);
  /*
    //--------------------------timer setup
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    timer1_counter = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
    TCNT1 = timer1_counter;   // preload timer
    TCCR1B |= (1 << CS12);    // 256 prescaler
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts
    //--------------------------timer setup
  */
  brushStop();
  roboStop();

}
/*****************************************************/
void loop() {
  //mySt='0';
  //serialEvent();
  //if (mySt == '1') { //Run the Robot autonomuslly
  brushRotate();
  while (1) {
    double d_Forward = sonar[0].ping_cm();
    Serial.print("forward is");
    Serial.println(d_Forward);
    double d_Backward = sonar[1].ping_cm();
    Serial.print("backward is");
    Serial.println(d_Backward);

    if (d_Forward > 30) {
      //roboForward_Relay();
      Serial.println("Forward");
    }
    else if (d_Backward > 30) {
      //roboBackward_Relay();
      Serial.println("Backward");
    }
    else {
      //roboStop();
      Serial.println("STOP");
    }


  }
  /*
    else if (mySt == '0') {
    brushStop();
    roboStop();
    }
    else {
    roboStop();
    }
  */
}
void roboForward_H (int s) {
  digitalWrite(INPUT1, HIGH);
  digitalWrite(INPUT2, LOW);
  digitalWrite(INPUT3, HIGH);
  digitalWrite(INPUT4, LOW);
  analogWrite(ENA, s);
  analogWrite(ENB, s);
}
void roboBackward_H (int s) {
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, HIGH);
  digitalWrite(INPUT3, LOW);
  digitalWrite(INPUT4, HIGH);
  analogWrite(ENA, s);
  analogWrite(ENB, s);
}
void roboForward_Relay (void) {
  digitalWrite(WHEELRELAY_FRD, HIGH);
  digitalWrite(WHEELRELAY_BKD, LOW);
}
void roboBackward_Relay (void) {
  digitalWrite(WHEELRELAY_BKD, HIGH);
  digitalWrite(WHEELRELAY_FRD, LOW);
}
void roboStop (void) {
  digitalWrite(WHEELRELAY_FRD, LOW);
  digitalWrite(WHEELRELAY_BKD, LOW);
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, LOW);
  digitalWrite(INPUT3, LOW);
  digitalWrite(INPUT4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void brushRotate(void) {
  digitalWrite(BRUSH_RELAY, HIGH);
}
void brushStop(void) {
  digitalWrite(BRUSH_RELAY, LOW);
}
void rencoder()  {// pulse and direction, direct port reading to save cycles
  rcount++;
}
void lencoder()  {// pulse and direction, direct port reading to save cycles
  lcount++;
}
void serialEvent(void) {
  while (Serial.available()) {
    char inChar = (char)Serial.read();// get the new byte
    if (inChar != '\n') {// add it to the inputString
      mySt += inChar;
    }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
/*ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
  {
  TCNT1 = timer1_counter;   // set timer
  pv_speed = 60.0*(rcount/12.0)/0.1;  //calculate motor speed, unit is rpm
  rcount=0;
  //print out speed
  if (Serial.available() <= 0) {
    Serial.print("speed");
    Serial.println(pv_speed);         //Print speed (rpm)
    }
  //PID program
  if (motor_start){
    e_speed = set_speed - pv_speed;
    pwm_pulse = e_speed*kp + e_speed_sum*ki + (e_speed - e_speed_pre)*kd;
    e_speed_pre = e_speed;  //save last (previous) error
    e_speed_sum += e_speed; //sum of error
    if (e_speed_sum >4000) e_speed_sum = 4000;
    if (e_speed_sum <-4000) e_speed_sum = -4000;
  }
  else{
    e_speed = 0;
    e_speed_pre = 0;
    e_speed_sum = 0;
    pwm_pulse = 0;
  }


  //update new speed
  if (pwm_pulse <255 & pwm_pulse >0){
    analogWrite(ENA,pwm_pulse);  //set motor speed
  }
  else{
    if (pwm_pulse>255){
      analogWrite(ENA,255);
    }
    else{
      analogWrite(ENA,0);
    }
  }
  }
*/
double distanceCalc(int sensorNum) {
  double duration, cm;
  if (sensorNum == 1) { // forwardSensor
    digitalWrite(TRIG_FORWARD, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_FORWARD, HIGH);
    delayMicroseconds(2);
    duration = pulseIn(ECHO_FORWARD, HIGH, 15000);
    cm = (duration) * (0.034 / 2.0); // Divide by 29.1 or multiply by 0.0343
    digitalWrite(TRIG_FORWARD, LOW);
    digitalWrite(ECHO_FORWARD, LOW);
    Serial.println(cm);


  }
  else if (sensorNum == 2) { // backwardSensor
    digitalWrite(TRIG_BACKWARD, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_BACKWARD, HIGH);
    delayMicroseconds(10);
    duration = pulseIn(ECHO_BACKWARD, HIGH);
    cm = (duration) * (0.034 / 2.0); // Divide by 29.1 or multiply by 0.0343
    digitalWrite(TRIG_BACKWARD, LOW);

  }
  else {
    //do nothing
  }

  return cm + 1;

}


