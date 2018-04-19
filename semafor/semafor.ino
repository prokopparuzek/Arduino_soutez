#include<Arduino.h>
#include<Fsm.h>
#include<Bounce2.h>
#include<MPU6050_tockn.h>
#include<Wire.h>
#define SERIAL_DEBUG true
#if SERIAL_DEBUG
#include<HardwareSerial.h>
#endif
#include<SerialDebug.h>
 #define TIME(last,interval) (millis() - (last)) > (interval) // makro na zjisteni doby behu
enum {RED1 = 22, RED2, YEL1, YEL2, GRE1, GRE2};
#define BUT1 42
#define POT A0
#define SPIN1 52
#define SPIN2 53
#define HIT 1
#define LEFT 2
#define SWITCH 5
#define POTSET 40
#define MPU 6

void green(void);
void reset(void);
void walker(void);
void set(void);
void potenciometr(void);
void fless2(void);
void f2(void);
void f4(void);
void f6(void);
void f8(void);
void mpuset(void);
void mpu30(void);
void mpu30_45(void);
void mpu45_60(void);
void mpu60_90(void);
void mpu90more(void);


Bounce debouncer1 = Bounce();
Bounce debouncer2 = Bounce();
Bounce debouncer3 = Bounce();

State state_green(&reset,&green,NULL);
State state_walker(NULL,&walker,NULL);
State state_switch(NULL,&set,NULL);
State state_potenciometr(NULL,&potenciometr,NULL);
State state_mpuSet(NULL,&mpuset,NULL);

Fsm fsm(&state_switch);

MPU6050 gyro(Wire);

void setup() {
int i;
for(i = 22; i <= 27; i++)
    pinMode(i, OUTPUT);
pinMode(BUT1,INPUT_PULLUP);
debouncer1.attach(BUT1);
debouncer1.interval(5);
pinMode(SPIN1,INPUT_PULLUP);
debouncer2.attach(SPIN1);
debouncer2.interval(10);
pinMode(SPIN2,INPUT_PULLUP);
debouncer3.attach(SPIN2);
debouncer3.interval(10);
pinMode(POT,INPUT);
fsm.add_transition(&state_green,&state_walker,HIT,NULL);
fsm.add_transition(&state_walker,&state_green,LEFT,NULL);
fsm.add_transition(&state_switch,&state_green,LEFT,NULL);
fsm.add_transition(&state_green,&state_switch,SWITCH,NULL);
fsm.add_transition(&state_switch,&state_potenciometr,POTSET,NULL);
fsm.add_transition(&state_potenciometr,&state_switch,SWITCH,NULL);
fsm.add_transition(&state_switch,&state_mpuSet,MPU,NULL);
fsm.add_transition(&state_mpuSet,&state_switch,SWITCH,NULL);
SERIAL_DEBUG_SETUP(9600);
Wire.begin();
gyro.begin();
gyro.calcGyroOffsets(true);
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
    fsm.trigger(SWITCH);
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

void set() {
int spin1, spin2;
debouncer2.update();
debouncer3.update();
//DEBUG("switch", debouncer2.read(),debouncer3.read());
spin1 = !debouncer2.read();
spin2 = !debouncer3.read();
if (spin1 && spin2) {
    fsm.trigger(LEFT);
}
else if (!spin1 && spin2) {
    fsm.trigger(POTSET);
}
else if(spin1 && !spin2) {
    fsm.trigger(MPU);
}
}

void potenciometr() {
int value;
value = analogRead(POT);
DEBUG("potenciometr",value);
if (value >= 0) fless2();
if (value > 205) f2();
if (value > 410) f4();
if (value > 614) f6();
if (value > 819) f8();
fsm.trigger(SWITCH);
}

void fless2() {
static unsigned long lastTime = 0;
DEBUG("fless2");
if (TIME(lastTime,2000)) {
    digitalWrite(RED1,!digitalRead(RED1));
    lastTime = millis();
}
}

void f2() {
static unsigned long lastTime = 0;
DEBUG("f2");
if (TIME(lastTime,1500)) {
    digitalWrite(RED2,!digitalRead(RED2));
    lastTime = millis();
}
}

void f4() {
static unsigned long lastTime = 0;
DEBUG("f4");
if (TIME(lastTime,2000)) {
    digitalWrite(GRE1,!digitalRead(GRE1));
    lastTime = millis();
}
}

void f6() {
static unsigned long lastTime = 0;
DEBUG("f6");
if (TIME(lastTime,1500)) {
    digitalWrite(GRE2,!digitalRead(GRE2));
    lastTime = millis();
}
}

void f8() {
static unsigned long lastTime = 0;
DEBUG("f8");
if (TIME(lastTime,2000)) {
    digitalWrite(YEL1,!digitalRead(YEL1));
    lastTime = millis();
}
}

void mpuset() {
int angle;
gyro.update();
//angle = gyro.getAngleX();
DEBUG("mpuset",gyro.getAccAngleX());
fsm.trigger(SWITCH);
}