#include <Servo.h>
int key;

Servo servo1;
int servo1Pin = 2;
int pos1 = 0;

Servo servo2;
int servo2Pin = 4;
int pos2 = 0;

Servo servo3;
int servo3Pin = 6;
int pos3 = 0;

Servo servo4;
int servo4Pin = 8;
int pos4 = 0;


void setup() {

  Serial.begin(9600);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);

}


void loop() {

if (Serial.available() > 0) {
  String line = Serial.readStringUntil("\n");
  line.trim();

  if (line.length() > 0) {
      // break into tokens
      const int MAXBUF = 64;
      char buf[MAXBUF];
      line.toCharArray(buf, MAXBUF);
      char* token = strtok(buf, ",");
      int vals[3] = {0,0,0};
      int i = 0;

      while (token != NULL && i < 3) {
        vals[i++] = atoi(token);
        token = strtok(NULL, ",");
      }

      if (i == 4) {
        pos1 = constrain(vals[0], 0, 180);
        pos2 = constrain(vals[1], 0, 180);
        pos3 = constrain(vals[2], 0, 180);
  
      }
  }
}

// move servos
        servo1.write(pos1);
        servo2.write(pos2);
        servo3.write(pos3);
        servo4.write(pos4);

}