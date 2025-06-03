#include <SevSeg.h>

SevSeg sevseg;

#define MAX_NUMBER_STRINGS 5
#define MAX_STRING_SIZE 8
char testStrings[MAX_NUMBER_STRINGS][MAX_STRING_SIZE];

#define PATTERN_CHANGE_TIME 1000
unsigned long timer = millis() - PATTERN_CHANGE_TIME;
byte testStringsPos = 0;

const int fold_pin = 14;
const int call_pin = 15;
const int bet_pin = 16;
const int done_pin = 17;
const int size_pin = A4;
float size;
float rounded;
float maxBet = 20.0;

// 0 waiting, 1 fold, 2 call, 3 bet, 4 done
int state = 0;


unsigned long old_time;
unsigned long new_time;

unsigned long period = 1000;


void setup() {
  Serial.begin(9600);

  pinMode(fold_pin, INPUT);
  pinMode(call_pin, INPUT);
  pinMode(bet_pin, INPUT);
  pinMode(done_pin, INPUT);

  byte numDigits = 4;
  byte digitPins[] = {10, 11, 12, 13};
  byte segmentPins[] = {9, 2, 3, 5, 6, 8, 7, 4};

  bool resistorsOnSegments = true;
  bool updateWithDelaysIn = true;
  byte hardwareConfig = COMMON_CATHODE;
  bool updateWithDelays = false;
  bool leadingZeros = false;
  bool disableDecPoint = false;
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
  updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(90);

  strcpy(testStrings[0], ". . . .");
  strcpy(testStrings[1], "FOLD");
  strcpy(testStrings[2], "CALL");
  strcpy(testStrings[3], "BET");
  strcpy(testStrings[4], "DONE");
}


void loop() {
  //sevseg.setNumber(4999, 3);
  //sevseg.refreshDisplay();
  /*
    if (millis() > (timer + PATTERN_CHANGE_TIME)) {
    sevseg.setChars(testStrings[testStringsPos]);
    testStringsPos++;
    if (testStringsPos >= MAX_NUMBER_STRINGS) testStringsPos = 0;
    timer = millis();
  }
  */
  // check for max bet info
  if (Serial.available()) {
    String incoming = Serial.readStringUntil('\n');
    float new_max = incoming.toFloat();
    if (new_max > 0.0 && new_max <= 100.0) {  // max bet 100
      max_bet = new_max;
    }
  }
  
  // handle buttons and send user input when done button is pressed
  if (state == 0){
    // check for fold, call, bet
    if (digitalRead(fold_pin)){
      state = 1;
    }
    else if (digitalRead(call_pin)){
      state = 2;
    }
    else if (digitalRead(bet_pin)){
      old_time = millis();
      state = 3;
    }
  }
  else if (state == 4){
    // reset to waiting
    new_time = millis();
    if ((new_time - old_time) > 1000)
    {
      //Serial.println(new_time-old_time);
      state = 0;
    }
  }
  else if (state == 3){
    // change to bet sizing display
    new_time = millis();
    if ((new_time - old_time) > 1000)
    {
      int raw = analogRead(size_pin);  // 0 to 1023
      size = (raw / 1023.0) * maxBet;
      rounded = round(size * 10.0) / 10.0;
      //size = analogRead(size_pin);
    }
  }
  if (digitalRead(done_pin)){
    // send data to ROS
    state = 4;
    old_time = millis();

    Serial.print("state:");
    Serial.print(state);
    Serial.print(",size:");
    Serial.println(size, 2);  // 2 decimal places
  }

  // write to 7 segment display
  if (state != 3){
    sevseg.setChars(testStrings[state]);
  }
  else{
    new_time = millis();
    if ((new_time - old_time) > 1000)
    {
      sevseg.setNumber(rounded * 100, 2);
    }
    else {
      sevseg.setChars(testStrings[state]);
    }
  }
  sevseg.refreshDisplay();
}

