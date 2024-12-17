#include <Arduino.h>

#define LED_OVERRIDE A2
#define LED_BUZZER A5
#define LED_RADIOTEST A4
#define LED_RADIOPYRO A3
#define LED_MCUARMED 6
#define LED_MCURDY 7

#define SW_ARMED 10
#define SW_BUZZER 9
#define SW_OVERRIDE 8

#define CHANNEL2 3
#define CHANNEL1 2

#define BUZZER A1
#define PYRO 11


// ++++++++++ SETTINGS ----------- //
#define LOOP_DEALY 20

#define PWM_ACTIVE 2000
#define PWM_THRESHOLD 50

#define SW_ARMED_DB_DELAY 50
#define SW_BUZZER_DB_DELAY 50
#define SW_OVERRIDE_DB_DELAY 50

#define TIMER_PERIOD1 1000000 // 1 second

// ************ globals ********** //
volatile long ch1_time[2] = { 0, 0};
volatile long ch2_time[2] = {0, 0};

bool ch1_active = false;
bool ch2_active = false;

bool mcu_armed = false;
bool buzzer = false;
bool override = false;

long sw_armed_debounce = 0;
long sw_buzzer_debounce = 0;
long sw_override_debounce = 0;

volatile bool led_blink = false;

bool led_override = false;
bool led_radiotest = false;
bool led_radiopyro = false;
bool led_mcurdy = false;

bool launching = false;

void channel1ISR(){
  ch1_time[0] = ch1_time[1];
  ch1_time[1] = micros();
}

void channel2ISR(){
  ch2_time[0] = ch2_time[1];
  ch2_time[1] = micros();
}

ISR(TIMER1_COMPA_vect){
  led_blink = !led_blink;
}

//TODO: add max on timer!

void debounce(int pin){

  bool present_state = !digitalRead(pin);
  long new_time = millis();

  switch(pin){
    case SW_ARMED:
      //if state is changed and debounce elapsed 
      if(present_state != mcu_armed && new_time - sw_armed_debounce > SW_ARMED_DB_DELAY){
        mcu_armed = present_state;
        sw_armed_debounce = new_time;
      }
      return;
    case SW_BUZZER:
      if(present_state != buzzer && new_time - sw_buzzer_debounce > SW_BUZZER_DB_DELAY){
        buzzer = present_state;
        sw_buzzer_debounce = new_time;
      }
      return;
    case SW_OVERRIDE:
      if(present_state != override && new_time - sw_override_debounce > SW_OVERRIDE_DB_DELAY){
        override = present_state;
        sw_override_debounce = new_time;
      }
      return;
    default:
      break;
  }
}

void blink(int pin, bool led_bool){
  if(led_blink && led_bool){
    digitalWrite(pin, HIGH);
  }else{
    digitalWrite(pin, LOW);
  }
}

void steady(int pin, bool led_bool){
  if(led_bool){
    digitalWrite(pin, HIGH);
  }else{
    digitalWrite(pin, LOW);
  }
}

void checkRadio(){
  //stat each channel to get pulse duration
  long tmp_ch1_time1 = 0;
  long tmp_ch1_time2 = 0;
  noInterrupts();
  tmp_ch1_time1 = ch1_time[0];
  tmp_ch1_time2 = ch1_time[1];
  interrupts();
  long tmp_ch1_time = tmp_ch1_time2 - tmp_ch1_time1;

  long tmp_ch2_time1 = 0;
  long tmp_ch2_time2 = 0;
  noInterrupts();
  tmp_ch2_time1 = ch2_time[0];
  tmp_ch2_time2 = ch2_time[1];
  interrupts();
  long tmp_ch2_time = tmp_ch2_time2 - tmp_ch2_time1;

  //check the channels
  if( tmp_ch1_time >= PWM_ACTIVE - PWM_THRESHOLD && tmp_ch1_time <= PWM_ACTIVE + PWM_THRESHOLD){
    if(!ch1_active){
      ch1_active = true;
      led_radiotest = true;
    }
  }else{
    ch1_active = false;
    led_radiotest = false;
  }

  if( tmp_ch2_time >= PWM_ACTIVE - PWM_THRESHOLD && tmp_ch2_time <= PWM_ACTIVE + PWM_THRESHOLD){
    if(!ch2_active){
      ch2_active = true;
      led_radiopyro = true;
    }
  }else{
    ch2_active = false;
    led_radiopyro = false;
  }
}

void setup() {

  //leds
  pinMode(LED_BUZZER, OUTPUT);
  pinMode(LED_RADIOTEST, OUTPUT);
  pinMode(LED_RADIOPYRO, OUTPUT);
  pinMode(LED_MCUARMED, OUTPUT);
  pinMode(LED_MCURDY, OUTPUT);

  //btns
  pinMode(SW_ARMED, INPUT);
  pinMode(SW_BUZZER, INPUT);
  pinMode(SW_OVERRIDE, INPUT);

  //buzzer
  pinMode(BUZZER, OUTPUT);

  //pyro
  pinMode(PYRO, OUTPUT);

  //pwm
  pinMode(CHANNEL1, INPUT);
  pinMode(CHANNEL2, INPUT);

  //Serial.begin(9600);

  noInterrupts();

  TCCR1A =0; //timer settings register 
  TCCR1B = 0; //timer settings register
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); //set prescaler to 64
  OCR1A = TIMER_PERIOD1 / 64 - 1; //compare match value; 16MHz clock, 64 prescaler
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare match interrupt

  attachInterrupt(digitalPinToInterrupt(3), channel2ISR, CHANGE); //pyro channel
  attachInterrupt(digitalPinToInterrupt(2), channel1ISR, CHANGE); //test channel

  //MCU ready
  ch1_time[1] = micros();
  ch2_time[1] = micros();

  interrupts();

  steady(LED_MCURDY, true);
}


void loop() {

  long loop_start = millis();

  delay(LOOP_DEALY);

  checkRadio();

  //pull the btns values and set bools with debouncing
  debounce(SW_ARMED);
  debounce(SW_BUZZER);
  debounce(SW_OVERRIDE);

  steady(LED_RADIOTEST, led_radiotest);
  blink(BUZZER, buzzer);
  blink(LED_BUZZER, buzzer);
  steady(LED_MCUARMED, mcu_armed);
  steady(LED_OVERRIDE, mcu_armed && buzzer);
  blink(LED_RADIOPYRO, led_radiopyro);

  //determine if it should launch
  if(mcu_armed && buzzer && (ch2_active || override)){
    digitalWrite(PYRO, HIGH);
  }else{
     digitalWrite(PYRO, LOW);
  }

  //Serial.print("loop Freq: ");
  long loop_end = millis();
  long loop_time = loop_end - loop_start;
  double loop_freq = 1.0 / (loop_time / 1000.0);
  //Serial.println(loop_freq);
}