// Arduino pin assignment
#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

//filter
#define LENGTH 90
#define k_LENGTH 20

// configurable parameters
#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)
#define Horizontal_Angle 1800 // 작아지면 올라감
#define Max_Variable_Angle 30

#define _POS_START (Horizontal_Angle + Max_Variable_Angle)
#define _POS_END (Horizontal_Angle - Max_Variable_Angle)

#define _SERVO_SPEED 30 // servo speed limit (unit: degree/second)
#define INTERVAL 100  // servo update interval


//ema
#define _DIST_ALPHA 0.42 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.



// global variables
int a, b; // unit: mm
int correction_dist, cnt;
float dist_list[LENGTH], sum, dist_ema, alpha;
Servo myservo;

int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
int inval = INTERVAL;
int duty_target, duty_value;


//ir_cali arr
const float coE[] = {-0.0000078, 0.0037895, 0.7732333, 37.6782211};

void setup() {
    // initialize GPIO pins
    pinMode(PIN_LED,OUTPUT);
    digitalWrite(PIN_LED, 1);

    myservo.attach(PIN_SERVO);
    duty_value = Horizontal_Angle;
    duty_target = _POS_END;
    myservo.writeMicroseconds(duty_value);
    
    // initialize serial port
    Serial.begin(57600);

    // convert angle speed into duty change per interval.
    duty_chg_per_interval = (int)((_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0));
    
    // initialize variables for servo update.
    pause_time = 1;
    toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
    toggle_interval_cnt = toggle_interval;


    // a = 107;
    // b = 247;
    a = 64;
    b = 298;
    correction_dist = 0;
    cnt = 0; sum = 0;
    alpha = _DIST_ALPHA;
    dist_ema = 0;
}

float ir_distance(void){ // return value unit: mm
    float val;
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0/(volt-9.0))-4.0) * 10.0;
    return val;
}

float makeCorrections() {
    cnt = 0;
    sum = 0;
    while (cnt < LENGTH)
    {
        dist_list[cnt] = 100 + 300.0 / (b - a) * (ir_distance() - a);
        sum += dist_list[cnt];
        cnt++;
    }

    for (int i = 0; i < LENGTH-1; i++){
        for (int j = i+1; j < LENGTH; j++){
            if (dist_list[i] > dist_list[j]) {
                float tmp = dist_list[i];
                dist_list[i] = dist_list[j];
                dist_list[j] = tmp;
            }
        }
    }
    
    for (int i = 0; i < k_LENGTH; i++) {
        sum -= dist_list[i];
    }
    for (int i = 1; i <= k_LENGTH; i++) {
        sum -= dist_list[LENGTH-i];
    }

    return sum/(LENGTH-2*k_LENGTH);
}


void loop() {
    // ir_dist filter
    float dist_cali = makeCorrections();
    dist_ema = alpha*dist_cali + (1-alpha)*dist_ema;

    
    
    // print
    Serial.print(",dist_ema:");
    Serial.print(dist_ema);
    
    Serial.print(",duty_value:");
    Serial.println(duty_value);

    
    // adjust duty_value toward duty_target by duty_chg_per_interval
    if(dist_ema < 260) {
        if (duty_value <= Horizontal_Angle+Max_Variable_Angle) {
            duty_value += duty_chg_per_interval * ((460-(int)dist_ema) / 1000.0);
        }   
        // if(dist_ema < 260) duty_value = duty_target;
    }
    else if(dist_ema > 240) {
        if (duty_value >= Horizontal_Angle-Max_Variable_Angle) {
            duty_value -= duty_chg_per_interval * (((int)dist_ema-60) / 1000.0);
        }
        // if(dist_ema > 250) duty_value = duty_target;
    }
  
    // update servo position
   myservo.writeMicroseconds(duty_value);

     //toggle duty_target between _DUTY_MIN and _DUTY_MAX.
    // if(toggle_interval_cnt >= toggle_interval) {
    //     toggle_interval_cnt = 0;
       
    //     if(duty_target == _POS_START) duty_target = _POS_END;
    //     else duty_target = _POS_START;
    // }
    // else {
    //     toggle_interval_cnt++;
    // }
    
    delay(20);
}
