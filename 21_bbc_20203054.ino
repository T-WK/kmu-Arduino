// Arduino pin assignment
#include <Servo.h>

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define Horizontal_Angle 2160
#define Max_Variable_Angle 100

Servo myservo;

int a, b, angle; // unit: mm

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);
  
  myservo.attach(PIN_SERVO);
  myservo.write(Horizontal_Angle);

  a = 68;
  b = 245;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);

  if (dist_cali > 255) {
//    for (angle = 0; angle <= Max_Variable_Angle; angle++){
      myservo.write(Horizontal_Angle - Max_Variable_Angle);
//      if (angle > -100)
//        angle--;
//      Serial.println(angle);
//    }
  }
  else if (dist_cali < 255) {
//    for (angle = 0; angle <= Max_Variable_Angle; angle++){
      myservo.write(Horizontal_Angle + Max_Variable_Angle/2);
//      if (angle < 100 )
//        angle++;
//    }
  }
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);
}
