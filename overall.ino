#include<Servo.h>
#include<SoftwareSerial.h>
#include<IRremote.h>

/* pins setup:
    Sensors             Arduino
    ------------        -------
    VCC                 5V
    GND                 GND
    S0(Color Sensor)    13
    S1(Color Sensor)    12
    S2(Color Sensor)    8
    S3(Color Sensor)    7
    OUT(Color Sensor)   2
    LED                 5V
    RFID                4
    IR left             A0
    IR Right            A1
    obstacle left       A2
    obstacle right      A3
    motor driver pin2   11
    motor driver pin7   6
    motor driver pin10  9
    motor driver pin15  10
    servo for flag      5
    IR emitter          3
*/

// define pins for infared obstable detector
#define left_sensor A0
#define right_sensor A1

//define pins for rfid and emitter
#define RFID 4
IRsend irsend;
SoftwareSerial Rfid = SoftwareSerial(RFID, 0);

bool follow_line = true;
// AnalogIn
#define IR_l A2
#define IR_r A3
bool read_l = LOW;
bool read_r = LOW;
bool RIGHT = false;
bool LEFT = false;
bool GO = false;
bool tmp = false;

//`Motor left
const int motorPin3  = 11;  // Pin 2 of L293
const int motorPin4  = 6;  // Pin 4 of L293
//Motor right
const int motorPin1  = 10; // Pin  15 of L293
const int motorPin2  = 9;  // Pin  10 of L293


const int s0 = 13;
const int s1 = 12;
const int s2 = 8;
const int s3 = 7;
const int out = 2;
Servo myservo;

int red = 0;
int green = 0;
int blue = 0;
int need_change = 0;
typedef enum Flag_state {
  up, down
} flag_state;

int cur_state = 0;
// to track ur current task and save time

void turnleft() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  //delay(400);
}

void turnright() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  //delay(400);
}

void carstop() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

void backward() {
  analogWrite(motorPin1, 200);
  digitalWrite(motorPin2, LOW);
  analogWrite(motorPin3, 200);
  digitalWrite(motorPin4, LOW);
}

void forward() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  //analogWrite(motorPin2, 180);
  digitalWrite(motorPin3, LOW);
  //analogWrite(motorPin4, 180);
  digitalWrite(motorPin4, HIGH);
}

void detect_color()
{
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  //count OUT, pRed, RED
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  //count OUT, pBLUE, BLUE
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  //count OUT, pGreen, GREEN
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}

void flag_down() {
  int pos = 0; //servo position variable
  for (pos = 0; pos <= 80; pos += 1) {
    myservo.write(pos);
    delay(5);
  }
  Flag_state flag_state = up;
}
void flag_up() {
  int pos; //servo position variable
  for (pos = 80; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(5);
  }
  Flag_state flag_state = down;
}
void fuck_turnright() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin4, HIGH);
  digitalWrite(motorPin3, LOW);
  analogWrite(motorPin2, 100);
}
void fuck_turnleft() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  analogWrite(motorPin4, 100);
}
void setup() {
  //IR setup
  Serial.begin(9600);
  Serial.println("Begin!");
  pinMode(IR_l, INPUT);
  pinMode(IR_r, INPUT);
  // Motor control
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  // infared obstacle detector setup
  pinMode(left_sensor, INPUT_PULLUP);
  pinMode(right_sensor, INPUT_PULLUP);

  // Serial Monitor to see results on the computer
  // Communication to the RFID reader
  Rfid.begin(9600);

  //setup for color sensor
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  myservo.attach(5);
  flag_up();
  //leave pin 5 for servo
}

void loop() {
  Serial.println(cur_state);

  //get readings from ir
  int i;
  int line_l = 0;
  int line_r = 0;
  for (i = 0; i <= 3; i++) {
    line_l += digitalRead(IR_l);
    line_r += digitalRead(IR_r);
    delay(10);
  }
  read_l = (line_l > 2);
  read_r = (line_r > 2);

  // rfid part
  if (cur_state == 2 and (read_l and read_r)) {
    carstop();
    if (Rfid.available() > 0) {
      // as long as there is data available... && cur_state == 1
      follow_line = false;
      cur_state ++;
      Serial.println("RFID if");
      while (Rfid.available() > 0 ) {
        carstop();
        unsigned long r = Rfid.read();
        delay(50);
        Serial.print("RFID detect!");
        Serial.println(r);
        irsend.sendRC5(r, 6); //send 0x0 code (8 bits)
        delay(200);
      }
      forward();
      delay(500);
      follow_line = true;
    }
    delay(1000);
  }


  // IR sensor
  // Reading from two IR sensor
  // When black return HIGH, else return LOW
  if (follow_line) {
    //    read_l = digitalRead(IR_l);
    //    read_r = digitalRead(IR_r);
    // Check the direction
    if (!read_l && !read_r) {
      GO = true;
      LEFT = false;
      RIGHT = false;
    }
    else if (read_l && !read_r) {
      GO = false;
      LEFT = true;
      RIGHT = false;
    }
    else if (!read_l && read_r) {
      GO = false;
      LEFT = false;
      RIGHT = true;
    }
    else if (cur_state == 1) {
      // this need to change according to your strategy
      GO = false;
      LEFT = false;
      RIGHT = false;
    }
    else {
      GO = GO;
      LEFT = LEFT;
      RIGHT = RIGHT;
    }
    if (GO) {
      forward();
      //      Serial.print(GO);
      //      Serial.print(LEFT);
      //      Serial.print(RIGHT);
      Serial.println("F");
    }
    else if (LEFT) {
      turnleft();
      //      Serial.print(GO);
      //      Serial.print(LEFT);
      //      Serial.print(RIGHT);
      Serial.println("L");
    }
    else if (RIGHT) {
      turnright();
      Serial.println("R");
    }
    else {
      carstop();
      Serial.println("S");
    }
  }

  //obstacle avoidance
  if (cur_state == 0) {
    int L = 0;
    int R = 0;
    for (int i = 0; i <= 3; i++) {
      L += digitalRead(left_sensor);
      R += digitalRead(right_sensor);
      delay(10);
    }
    L = max(L - 3, 0);
    R = max(R - 3, 0);
    if (not L and not R) {
      cur_state ++;
    }
  }
  if (cur_state == 1) {
    int L = 0;
    int R = 0;
    for (int i = 0; i <= 2; i++) {
      L += digitalRead(left_sensor);
      R += digitalRead(right_sensor);
      delay(10);
    }
    L = max(L - 2, 0);
    R = max(R - 2, 0);
    Serial.println(L);
    Serial.println(R);
    //    if (L != 0 && R != 0) {
    //      forward();
    //      Serial.println("f");
    //    }
    // for testing
    if (L == 0 || R == 0) {
      follow_line = false;
      tmp = true;
      //Serial.println(tmp);
      //Serial.println(cur_state);
      if ( L != 0) {
        fuck_turnleft();
        delay(1500);
        Serial.println("Obstacle, L");
      }
      else if (R != 0) {
        fuck_turnright();
        delay(1400);

        Serial.println("Obstacle, R");
      }
      else if (R == 0 && L == 0) {
        forward();
        Serial.println("Obstacle, f");
      }
    }
    else if (tmp == true) {
      carstop();
      for (int i = 0; i <= 9; i++) {
        L += digitalRead(left_sensor);
        R += digitalRead(right_sensor);
        delay(10);
      }
      L = max(L - 8, 0);
      R = max(R - 8, 0);
      if (L != 0 and R != 0) {
        cur_state ++;
        follow_line = true;
      }
    }
  }

  //color detect
  if (cur_state == 3 ) {
    //initialize and flag up!
    Flag_state flag_state = up;
    if (need_change == 0) {
      flag_up();
      Serial.print("Up");
    }
    else if (need_change == 1) {
      Serial.print("down!");
      flag_state = down;
    }
    else if (need_change == 2) {
      Serial.print("Up");
      flag_state = up;
    }
    detect_color();
    Serial.print("R Intensity:");
    Serial.print(red, DEC);
    Serial.print(" G Intensity: ");
    Serial.print(green, DEC);
    Serial.print(" B Intensity : ");
    Serial.print(blue, DEC);
    Serial.print(need_change);
    if (red < blue && red < green && red < 60 && (red < 0.5 * green or red < 0.5 * blue) ) //needs to tune the parameters here of RBG relations, use combination to find the optimal logic
    {
      Serial.println(" - (Red Color)");
      if (flag_state == up) {
        flag_down();
        need_change = 1;
      }
    }
    else if (green < red && green < blue)        ////needs to tune the parameters here of RBG relations
    {
      Serial.println(" - (Green Color)");
      if (flag_state == down) {
        flag_up();
        need_change = 2;
      }
    }
    else {
      Serial.println();
    }
  }
}
