#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  
  Serial.println("Hello World!");
  count = 0;
  toggle = HIGH;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);

  if (count == 1){
      toggle = toggle_state(toggle); //toggle LED value.
      digitalWrite(PIN_LED, toggle); //update LED status.
  }
  else if (count == 2){
    while(1){
      if (count-1 == 11){
        toggle = toggle_state(toggle); //toggle LED value.
        digitalWrite(PIN_LED, toggle);
        break;
      }
        
      
      toggle = toggle_state(toggle); //toggle LED value.
      digitalWrite(PIN_LED, toggle); //update LED status.
      delay(100);
      
      ++count;
    }
  }
  delay(1000); // wait for 1,000 milliseconds
}

int toggle_state(int toggle) {
  if (toggle == LOW) {
    return HIGH;
  }
  else {
    return LOW;
  }
}
