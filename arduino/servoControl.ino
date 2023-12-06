#include<Servo.h>

Servo m1,m2,m3,m4,m5;
int prevpos[5] = {90,90,180,90,90};
int pos[5] = {90,90,180,90,90};

void driveServo(Servo s, int prev, int newone){
  if(prev>newone){
    for(int i=prev;i>=newone;i--){
      s.write(i);
      delay(25);
      }
    }

  else{
    for(int i=prev;i<=newone;i++){
      s.write(i);
      delay(25);
      }
    }
  
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  m1.attach(8);
  m2.attach(9);
  m3.attach(11);
  m4.attach(12);
  m5.attach(13);

}

void loop() {
  if(Serial.available()>=10){
    for(int i=0;i<5;i++){
    pos[i] = Serial.parseInt();
      }
    }
    

    driveServo(m1,prevpos[0],pos[0]);
    prevpos[0] = pos[0];
    delay(5);

    driveServo(m2,prevpos[1],pos[1]);
    prevpos[1] = pos[1];
    delay(5);

    driveServo(m3,prevpos[2],pos[2]);
    prevpos[2] = pos[2];
    delay(5);

    driveServo(m4,prevpos[3],pos[3]);
    prevpos[3] = pos[3];
    delay(5);

    driveServo(m5,prevpos[4],pos[4]);
    prevpos[4] = pos[4];
    delay(5);

    }