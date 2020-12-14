#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9   			// [1234] LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10		// [3069] 서보모터를 아두이노 GPIO 10번 핀에 연결
#define PIN_IR A0			// [3070] 적외선 센서를 아두이노 아날로그 A0핀에 연결

// Framework setting
#define _DIST_TARGET 255		// 목표하는 위치
#define _DIST_MIN 100		//[3050] 거리의 최솟값이 100mm
#define _DIST_MAX 410                   //[3068] 거리의 최대값 410mm

// Distance sensor
#define _DIST_ALPHA 0.5		
#define a 64
#define b 298

// Servo range
#define _DUTY_MIN 1280 		// [3052] 서보의 최소각도를 microseconds로 표현
#define _DUTY_NEU 1480 		// [3070] 레일플레이트 중립위치를 만드는 서보 duty값
#define _DUTY_MAX 1680                //[3062] 서보의 최대각도의 microseconds의 값

// Servo speed control
#define _SERVO_ANGLE 25 	// [3131] 최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 80 	//[3064]서보 속도를 30으로

// Event periods
#define _INTERVAL_DIST 10 	// 적외선센서 업데이트 주기
#define _INTERVAL_SERVO 10 	// 서보 업데이트 주기
#define _INTERVAL_SERIAL 100 	// [3070] 시리얼 플로터 갱신 속도

// PID parameters
#define _KP 0.75  // P 이득 비율
#define _KI 17.5   // I 이득 비율
#define _KD 175.0 // D 이득 비율
#define _ITERM_MAX 200

// ir filter
#define LENGTH 70
#define k_LENGTH 15


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo; // [3070] 서보 객체 생성

// Distance sensor
float dist_target; // [1234] location to send the ball
float dist_raw, dist_ema; // [3070] 거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,last_sampling_time_serial;
// 각 이벤트별 업데이트를 위해 측정하는 시간
//[3054] 마지막으로 측정한 거리, 마지막으로 측정한 서보 각도 (unsigned long : 부호없는 정수형)
bool event_dist, event_servo, event_serial;	// 이벤트별 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_per_interval; // [3055] 서보속도 제어를 위한 변수 선언
int duty_target, duty_curr, duty_neutral;	// [3131] 목표duty, 현재duty 중앙duty

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; // [3070] PID 제어를 위한 현재 오차, 이전오차, 컨트롤(?), p 값, d 값, i 값 변수 선언


void setup() {
// initialize GPIO pins for LED and attach servo 
    pinMode(PIN_LED, OUTPUT); //[3062] 핀 LED 활성화
    myservo.attach(PIN_SERVO); // [3070] 서보 구동을 위한 서보 초기화

    // initialize global variables
    duty_curr = _DUTY_MIN; // [3055] duty_target, duty_curr 초기화
    duty_target = _DIST_TARGET;
    duty_neutral = _DUTY_NEU;
    last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;
    // [3055] 샘플링 타임 변수 초기화
    dist_raw = dist_ema = 0; // [3055] dist 변수 초기화
    pterm = iterm = dterm = 0; // pid 제어값에서 우선 p 만 사용하도록

    error_prev = 0;

    // move servo to neutral position
    myservo.writeMicroseconds(_DUTY_NEU); 
    //while(1) {}

    // initialize serial port
    Serial.begin(57600); 

    // convert angle speed into duty change per interval.
    duty_chg_per_interval = int((_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED /  _SERVO_ANGLE) * (float(_INTERVAL_DIST) / 1000.0));

}
  
void loop() {
    /////////////////////
    // Event generator //
    /////////////////////
    unsigned long time_curr = millis(); // [3070] 이벤트 업데이트 주기 계산을 위한 현재 시간
    // [3070] 이벤트 주기가 돌아올때까지 현재시간과 비교하며 기다리도록 함.
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    ////////////////////
    // Event handlers //
    ////////////////////

    if(event_dist) {
        event_dist = false; // 업데이트 대기
        // get a distance reading from the distance sensor
        dist_raw = ir_distance_filtered(); // 센서 값 받아와서 변수에 저장
        dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema; 
        // dist_ema 설정

        // PID control logic
        error_curr = _DIST_TARGET - dist_raw; // 오차 계산
        
        pterm = _KP * error_curr;
        dterm = _KD * (error_curr - error_prev);
        iterm += _KI * error_curr;
        control = pterm + dterm;

        // Limit iterm
        if (abs(iterm) > _ITERM_MAX) iterm = 0;
        if (iterm > _ITERM_MAX) iterm = _ITERM_MAX;
        if (iterm < -_ITERM_MAX) iterm = -_ITERM_MAX;
        
        duty_target = _DUTY_NEU + control;

        // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
        if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
        if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

        // update error_prev
        error_prev = error_curr;

        // duyy_target = _DUTY_NEU + (control - X) * (control > X ? _DUTY_MAX - _DUTY_NEU : _DUTY_NEU - _DUTY_MIN);
        // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    }
  
    if(event_servo) {
        event_servo = false; // 업데이트 대기
        // adjust duty_curr toward duty_target by duty_chg_per_interval
        if(duty_target > duty_curr) { // [3070]
            duty_curr += duty_chg_per_interval;
            if(duty_curr > duty_target) duty_curr = duty_target;
        }
        else {
            duty_curr -= duty_chg_per_interval;
            if(duty_curr < duty_target) duty_curr = duty_target;
        }

        // update servo position
        myservo.writeMicroseconds((int)duty_curr); // [3070]

        event_servo = false; // [3055] 모든 작업이 끝나면 이벤트를 다시 false로
    }
  
    if(event_serial) {
        event_serial = false;
        Serial.print("IR:");
        Serial.print(dist_ema);
        Serial.print(",T:");
        Serial.print(dist_target);
        Serial.print(",P:");
        Serial.print(map(pterm,-1000,1000,510,610));
        Serial.print(",D:");
        Serial.print(map(dterm,-1000,1000,510,610));
        Serial.print(",I:");
        Serial.print(map(iterm,-1000,1000,510,610));
        Serial.print(",DTT:");
        Serial.print(map(duty_target,1000,2000,410,510));
        Serial.print(",DTC:");
        Serial.print(map(duty_curr,1000,2000,410,510));
        Serial.println(",-G:245,+G:265,m:255,M:800");
    }
}

float ir_distance(void){ // return value unit: mm
    float val; // [3055] 변수 val 선언
    float volt = float(analogRead(PIN_IR)); // [3055] volt변수에 적외선 센서 측정값 입력
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0; // [3055] volt 값 보정
    return val; // [3055] val 리턴
}

float ir_distance_filtered(void){ // return value unit: mm
    int cnt = 0;
    int sum = 0;
    int dist_list[LENGTH+1];
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
