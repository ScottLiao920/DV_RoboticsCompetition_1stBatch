#include<Servo.h>
#include<SoftwareSerial.h>
#include<IRremote.h>

// define pins for infared obstable detector
#define left_sensor A0
#define right_sensor A1

//define pins for rfid and emitter
#define EMITTER 1
IRsend irsend;
SoftwareSerial Rfid = SoftwareSerial(EMITTER, 0);

bool follow_line = true;
// AnalogIn
const int IR_l = 4;
const int IR_r = 2;
bool read_l = LOW;
bool read_r = LOW;
bool RIGHT = false;
bool LEFT = false;
bool GO = false;
bool tmp = true;

//`Motor left
const int motorPin3  = 11;  // Pin 2 of L293
const int motorPin4  = 6;  // Pin 4 of L293
//Motor right
const int motorPin1  = 10; // Pin  15 of L293
const int motorPin2  = 9;  // Pin  10 of L293

/* TCS230 color sensor pins setup:
    Color Sensor  Arduino
    ------------  -------
    VCC           5V
    GND           GND
    S0            13
    S1            12
    S2            6
    S3            7
    OUT           5
    LED           5V
*/
const int s0 = 13;
const int s1 = 12;
const int s2 = 8;
const int s3 = 7;
const int out = 5;
Servo myservo;

int red = 0;
int green = 0;
int blue = 0;
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
  analogWrite(motorPin2, 180);
  digitalWrite(motorPin3, LOW);
  analogWrite(motorPin4, 180);
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

void flag_up() {
  int pos = 0; //servo position variable
  for (pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);
    delay(10);
  }
  Flag_state flag_state = up;
}
void flag_down() {
  int pos = 180; //servo position variable
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(10);
  }
  Flag_state flag_state = down;
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
  myservo.attach(8);
  //leave pin 8 for servo
}

void loop() {
  // IR sensor
  // Reading from two IR sensor
  // When black return HIGH, else return LOW
  // carstop();
  if (follow_line) {
    int i;
    int line_l = 0;
    int line_r = 0;
    for (i = 0; i <= 3; i++) {
      line_l += digitalRead(IR_l);
      line_r += digitalRead(IR_r);
    }
    read_l = (line_l > 2);
    read_r = (line_r > 2);
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

    if (GO) {
      forward();
      Serial.println("F");
    }
    else if (LEFT) {
      turnleft();
      Serial.println("L");
    }
    else if (RIGHT) {
      turnright();
      Serial.println("R");
    }
  }
  if (cur_state == 0) {
    int L = digitalRead(left_sensor);
    int R = digitalRead(right_sensor);
    //    if (L == 0 && R == 0) {
    //      forward();
    //      Serial.println("f");
    //    }
    // for testing
    if (L || R) {
      follow_line = false;
      // cur_state ++;
      if ( L != 0) {
        turnleft();
        Serial.println("L");
      }
      if (R != 0) {
        turnright();
        Serial.println("R");
      }
      if (R != 0 && L != 0) {
        forward();
        Serial.println("f");
      }
    }
  }
  // rfid part

  if (Rfid.available() > 0 && cur_state == 1) {
    // as long as there is data available...
    follow_line = false;
    cur_state ++;
    Serial.println("RFID if");
    while (Rfid.available() > 0 ) {
      carstop();
      Serial.println("RFID detect!");
      unsigned long r = Rfid.read();
      irsend.sendRC5(r, 6); //send 0x0 code (8 bits)
      delay(200);
    }
    forward();
    delay(500);
    follow_line = true;
  }

  //color detect
  if (cur_state == 2) {
    //initialize and flag up!
    Flag_state flag_state = up;
    flag_up();

    detect_color();
    Serial.print("R Intensity:");
    Serial.print(red, DEC);
    Serial.print(" G Intensity: ");
    Serial.print(green, DEC);
    Serial.print(" B Intensity : ");
    Serial.print(blue, DEC);
    //Serial.println();
    if (red < blue && red < green && red < 60 && (red < 0.5 * green or red < 0.5 * blue) ) //needs to tune the parameters here of RBG relations, use combination to find the optimal logic
    {
      follow_line = false;
      Serial.println(" - (Red Color)");
      if (flag_state == up)
        flag_down();
    }

    else if (green < red && green < blue)        ////needs to tune the parameters here of RBG relations
    {
      follow_line = false;
      Serial.println(" - (Green Color)");
      if (flag_state == down )
        flag_up();
    }
    else {
      follow_line = true;
      Serial.println();
    }
  }
}
