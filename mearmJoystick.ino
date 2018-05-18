/*

web site : http://www.7gp.cn

pins: 
11/10/9/5 : Servo base/left/right/gripper
*/
#include <Servo.h> 
#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13);     // hc06 bluetooth rx/tx
const int SERVOS = 4, MAXSPEED = 10; // servo pin 11/10/9/5 base/left/right/gripper
int PIN[SERVOS], value[SERVOS], currentAngle[SERVOS], MIN[SERVOS], MAX[SERVOS], INITANGLE[SERVOS];
Servo myservo[SERVOS];
int aOrg[] = {90, 90, 90};  // initial angle for base/left/right arm arm
int demoActions[9][4] = { { 90,  55, 165, 30},    // demo mode by pressing right button when boot 
                          { 45, 145,  90,  0},
                          { 90,  55, 165,  0},
                          {135, 145,  90, 30},
                          { 90,  55, 165, 30},
                          {135, 145,  90,  0},
                          { 90,  55, 165,  0},
                          { 45, 145,  90, 30},
                          { 90,  55, 165, 30}                          
                       };
                      
int DELAYTIME = 200;
int servo_moving[SERVOS] = {0,0,0,0};
boolean stringComplete = false;
int bt_servo = 0;
int bt_move = 0;
int idle[] = {0,0,0,0};
boolean learningMode = false, autoPlayMode = false, autoDemoMode = false;
boolean clawClosed = false, bootbuttonPressed = false;
int const maxAutoActions = 100;
int autoAction[maxAutoActions][SERVOS];  // can record up to 20 actions
int actionIndex = 0;  // total steps of an movement
int buttonPreState = 0;
int buttonL = 2, buttonR = 4;
int threshL = 300, threshR = 700;
int clawOpenangle = 10, clawCloseangle = 102;
int demospeed = 1;  //1 for 1x, 2 for 2x...

void setup() {
  Serial.begin(9600);    // RX/TX = D0/D1
  pinMode(2, INPUT_PULLUP);  //2 to memorize 
  pinMode(4, INPUT_PULLUP);  //4 to start actions
  pinMode(3, OUTPUT);  
  digitalWrite(3, HIGH);
  Serial.print(digitalRead(buttonL));
  Serial.print(" , ");
  Serial.println(digitalRead(buttonR));
  
  mySerial.begin(38400); // RX/TX = D12/D13 for bluetooth 
  
  init_Pins();
  cutcut();              

  // long press left button for recording mode
  if (!digitalRead(buttonL)) {
    learningMode = true;
    buttonPreState = 1;
    delay(1000);
    digitalWrite(3, LOW);
    Serial.println("learningMode!!");
  }
  
  // long press right button for demo mode
  if (!digitalRead(buttonR)) {
    autoDemoMode = true;
    buttonPreState = 1;
    auto_mode();
  }
}

void loop() {
  armfromto(autoAction[actionIndex], autoAction[0]);
}

void init_Pins(){
  PIN[0] = 11;          // pin to attach servo
  MIN[0] = 10;           // minimal angle of this servo based on mechanic structure
  MAX[0] = 179;         // maximum angle of this servo based on mechanic structure
  INITANGLE[0] = 90;    // initial angle at start up
  PIN[1] = 10;
  MIN[1] = 10;
  MAX[1] = 140;
  INITANGLE[1] = 90;
  PIN[2] = 9;
  MIN[2] = 40;
  MAX[2] = 170;
  INITANGLE[2] = 90;
  PIN[3] = 5;
  MIN[3] = 10;
  MAX[3] = 102;
  INITANGLE[3] = 60;
  
  //set motors to initial angles
  initMotors();
  
  // reset autoaction angles to record
  for (int i = 0 ; i < maxAutoActions; i++){
    for (int j = 0 ; j < SERVOS; j++){
      autoAction[i][j] = 0;
    }
  }
  // set autoAction[0][] = aOrg[];
  for (int i = 0 ; i < SERVOS-1; i++){
    autoAction[0][i] = aOrg[i];
  }
  autoAction[0][SERVOS-1] = clawOpenangle;
}

void move_bt(){ 
  checkSoftSerial();  // read bluetooth and set servo_moving[i] for each servo, format +1/0/-1

  for (int i = 0; i < SERVOS; i++){
    currentAngle[i] = myservo[i].read();  // get existing serveo angle
    
    if (servo_moving[i] != 0){
      currentAngle[i] += servo_moving[i];  // +/- angles based on the info from bluetooth
      currentAngle[i] = currentAngle[i] > MAX[i] ? --currentAngle[i] : currentAngle[i];  // ensure target angle within safe range 
      currentAngle[i] = currentAngle[i] < MIN[i] ? ++currentAngle[i] : currentAngle[i];
      myservo[i].write(currentAngle[i]);  // rotate each servo
      delay(20);
    }
  }
}

// read from bluetooth and illustrate into angle change
// the format is 2 digit like "31", the first digit is servo , the 2nd digit is 1 or 2 for +/- angle
void checkSoftSerial() {
  String str = "";
  
  if (mySerial.available()){
    for (int i = 0 ; i < 2; i++){
      str += (char)mySerial.read(); 
    }
    // the servo to move
    int value = str.toInt();
    bt_servo = value / 10;
    
    // the direction to move
    int angle = value % 10;
    if (angle == 2) bt_move = 1;
    else if (angle == 1) bt_move = -1;
    else bt_move = 0;

    servo_moving[bt_servo] = bt_move;  // so we set +1/0/-1 into array to be used in move_bt() 
  }
}

void move_joy(){
  boolean joyChanged = false;
  
  for (int i = 0; i < SERVOS; i++){
    value[i] = analogRead(i);              // read the joystick value from analog port 0~3
    delay(2);
    currentAngle[i] = myservo[i].read();   // get existing angles    
       
    if (value[i] > threshR) {
      idle[i] = 0;      // reset idle because someone touch the joystick
      joyChanged = true;
      if (currentAngle[i] > MIN[i]) --currentAngle[i]; // adjust the angle according to joystick direction
      if (i == SERVOS-1) currentAngle[i] = clawCloseangle;            // for claw, only open/close mode
    } else if (value[i] < threshL) {
      idle[i] = 0;
      joyChanged = true;
      if (currentAngle[i] < MAX[i]) ++currentAngle[i]; 
      if (i == SERVOS-1) currentAngle[i] = clawOpenangle;
    } else {
      ++idle[i];
    }
    
    if (idle[i] >= 100){
      //myservo[i].detach();  // optional: detach servo after idle for a while
      idle[i] = 0;
    }
  }      
  if (joyChanged){  
    for (int i = 0 ; i < SERVOS; i++){
      // before write angle , let's attach servo if not
      if (!myservo[i].attached()) myservo[i].attach(PIN[i]);      
      myservo[i].write(currentAngle[i]);
    }  
  }
  delay(20);
}

// memorize the actions
void move_mem(){
  if (!autoPlayMode){
    if (learningMode){
      //Serial.println("learningMode");
      // if one of any button pressed, memorize the angles
      if (!digitalRead(buttonR)) {
        // done recoding, going to auto play
        autoPlayMode = true;
        return;
      }
      int buttonPressed = !digitalRead(buttonL);
      //Serial.print("buttonPressed = ");
      //if (buttonPressed) Serial.println("buttonPressed");
      delay(1);
      
      
      if (buttonPreState && buttonPressed) return; // repeated count of the same press
      else if (!buttonPreState && buttonPressed){  // Left button pressed to record
      actionIndex++;
        buttonPreState = true;
        if (actionIndex < maxAutoActions){
          Serial.print("actionIndex = ");
          Serial.println(actionIndex);
          for(int i = 0; i < SERVOS; i++){
            int tmp = myservo[i].read();
            Serial.print(tmp);
            Serial.print(" , ");
            autoAction[actionIndex][i] = tmp;
          }
          Serial.println("");
          
          
          if (actionIndex == maxAutoActions) autoPlayMode = true;
        }
      } else buttonPreState = buttonPressed;
      
      if (!digitalRead(buttonR)) { // Right button pressed
        autoPlayMode = true;
      }
    }
  } else {
    // auto Playing Mode
    autoPlay();
  }
  //delay(100);
}

// open and close the claw 2 times
void cutcut(){
  delay(1000);
  for (int i = 0; i < 2; i++){
    closeclaw(true);
    delay(150);
    closeclaw(false);
    delay(150);
  }
}

// auto mode for demo purpose, can be paused any time if joystick is touched
void auto_mode(){
  int elements = sizeof(demoActions) / (4*sizeof(int));
  Serial.print("elements = ");
  Serial.println(elements);
  cutcut();
  while(1){
    for (int i = 0; i < elements-1; i++){
      if (buttonPressed()) {
        learningMode = false;
        autoPlayMode = false;
        autoDemoMode = false;
        return;  
      }
      armfromto(demoActions[i], demoActions[i+1]);
    }
  } 
}

//play recorded mode
void autoPlay(){
  digitalWrite(3, HIGH);
  // moving from current to the first position
  //Serial.print("autoPlay: last one to 0 : total ");
  //Serial.println(actionIndex);
  //armfromto(autoAction[actionIndex], autoAction[0]);
  initMotors();
  delay(1000);
  cutcut();  
  
  while(1){    
    for (int i = 0; i < actionIndex; i++){     
      Serial.println("start playing...");
      
      // pause if joystick touched, resume if touch again
      if (buttonPressed()) {
        learningMode = false;
        autoPlayMode = false;
        autoDemoMode = false;
        return;
      }
      Serial.print("autoplay : ============="); 
      Serial.println(i);
      armfromto(autoAction[i], autoAction[i+1]);
    }
    delay(500/demospeed);
    armfromto(autoAction[actionIndex], autoAction[0]);  //back to the first action
    delay(500/demospeed);
  }
}

// rotate all servos at a time smoothly by using "for" loop and delay(30) 
void armfromto(int *arrf, int *arrt){
  int maxAngles = 0;
  float lp[4];

  // adjust speed
  adjustSpeed();
      
  maxAngles = max(max(abs(arrt[0] - arrf[0]), abs(arrt[1] - arrf[1])), abs(arrt[2] - arrf[2]));
  maxAngles /= demospeed;
  maxAngles = maxAngles < 1 ? 1 : maxAngles;  
  Serial.print("speed = ");
  Serial.println(demospeed); 
  
  for (int i = 0; i < SERVOS-1; i++){  
   // Serial.print(arrt[i] - arrf[i]);
   // Serial.print(" "); 
  }
  //Serial.println(" === Delta");
  
  for (int i = 0; i < SERVOS-1; i++){    
    lp[i] = float(arrt[i] - arrf[i])/float(maxAngles);
    //Serial.print(lp[i]);
    //Serial.print(" "); 
    //Serial.println();
  }
  
  for (int j = 1; j <= maxAngles; j++){

    for (int i = 0; i < SERVOS-1; i++){
      
      //Serial.print(arrf[i]+j*lp[i]); 
      //Serial.print(" "); 
      myservo[i].write(arrf[i]+j*lp[i]);      
    }
   // Serial.print(" === "); 
   // Serial.println(j); 
    
    delay(20);
  } 
  // the claw
  myservo[SERVOS-1].write(arrt[SERVOS-1]);
  delay(20);  
    
  if (autoPlayMode || autoDemoMode){
    //Serial.print("claw angle: ");
    //Serial.println(clawCloseangle);
    //myservo[SERVOS-1].write(arrt[SERVOS-1]);
    //delay(20);  
  }
}

// close the claw
void closeclaw(boolean op){
  if (op){
    myservo[SERVOS-1].write(clawCloseangle);
  } else {
    myservo[SERVOS-1].write(clawOpenangle);
  }
}

// check if joystick touched to stop the auto mode
boolean joystickTouched(){
  int tmp = 500;
  
  for (int i = 0; i < SERVOS; i++){
    tmp = analogRead(i);
    delay(1);
    if (tmp > threshR || tmp < threshL) return true;
  }
  
  return false;
}

void initMotors(){
  // attach servos to pins
  for (int i = 0; i < SERVOS; i++){
    myservo[i].attach(PIN[i]);
    myservo[i].write(INITANGLE[i]);
    value[i] = 0;
    idle[i] = 0;
  }  
}

void adjustSpeed(){
  //if (learningMode) {
    for (int i = 0; i < SERVOS; i++){
      if (analogRead(i) > threshR) demospeed++;
      if (analogRead(i) < threshL) demospeed--;
    }
    demospeed = demospeed < 1 ? 1 : demospeed;
    demospeed = demospeed > MAXSPEED ? MAXSPEED : demospeed;
  //}
}

boolean buttonPressed(){
  //return true when any button pressed and not previously pressed
  if(!buttonPreState && (!digitalRead(buttonR) || !digitalRead(buttonL))) {
    buttonPreState = 1;
    return true;
  } else if (digitalRead(buttonR) && digitalRead(buttonL)){
    buttonPreState = 0;
  }
  return false;  
}
