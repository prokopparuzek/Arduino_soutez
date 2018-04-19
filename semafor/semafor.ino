#include<Arduino.h>
#include<Fsm.h>
#include<Bounce2.h>
#define SERIAL_DEBUG true
#if SERIAL_DEBUG
#include<HardwareSerial.h>
#include<SerialDebug.h>
#endif
enum {RED1 = 22, RED2, YEL1, YEL2, GRE1, GRE2};
#define BUT1 42
#define HIT 1
#define LEFT 2

void green(void);
void reset(void);
void walker(void);


Bounce debouncer1 = Bounce();

State state_green(&reset,&green,NULL);
State state_walker(NULL,&walker,NULL);

Fsm fsm(&state_green);

void setup() {
int i;
for(i = 22; i <= 27; i++)
    pinMode(i, OUTPUT);
pinMode(BUT1,INPUT_PULLUP);
debouncer1.attach(BUT1);
debouncer1.interval(10);
fsm.add_transition(&state_green,&state_walker,HIT,NULL);
fsm.add_transition(&state_walker,&state_green,LEFT,NULL);
SERIAL_DEBUG_SETUP(9600);
}

void loop() {
fsm.run_machine();
}

void green() {
    debouncer1.update();
    DEBUG("green",debouncer1.read());
    if (debouncer1.fell()) {
        fsm.trigger(HIT);
    }
}

void reset() {
    digitalWrite(RED1,LOW);
    digitalWrite(RED2,HIGH);
    digitalWrite(YEL1,LOW);
    digitalWrite(YEL2,LOW);
    digitalWrite(GRE1,HIGH);
    digitalWrite(GRE2,LOW);
}

void walker() {
DEBUG("walker");
delay(10000);
digitalWrite(GRE1,LOW);
digitalWrite(YEL1,HIGH);
delay(1000);
digitalWrite(YEL1,LOW);
digitalWrite(RED1,HIGH);
delay(1500);
digitalWrite(RED2,LOW);
digitalWrite(GRE2,HIGH);
delay(2500);
digitalWrite(GRE2,LOW);
digitalWrite(RED2,HIGH);
delay(4000);
digitalWrite(YEL1,HIGH);
delay(500);
digitalWrite(RED1,LOW);
digitalWrite(YEL1,LOW);
digitalWrite(GRE1,HIGH);
fsm.trigger(LEFT);
}