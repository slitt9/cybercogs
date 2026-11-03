// Standalone PID right-wall follower: complete single-file sketch.
#include <stdint.h>
#include <math.h>
namespace Tune {
 const float TARGET_CM=15.0f, FRONT_TURN_CM=30.0f, FRONT_CLEAR_CM=34.0f;
 const float FRONT_STOP_CM=12.0f, FRONT_RELEASE_CM=16.0f;
 const float MAX_RELEVANT_SIDE_CM=25.0f, MAX_RELEVANT_FRONT_CM=60.0f;
 const float WALL_LOST_CM=MAX_RELEVANT_SIDE_CM, STABLE_MIN_CM=8.0f, STABLE_MAX_CM=22.0f;
 const float STABLE_CHANGE_CM=1.0f;
 const float KP=2.0f, KI=0.10f, KD=0.5f;
 const float SIDE_FILTER=0.45f, DERIVATIVE_FILTER_SECONDS=0.25f;
 const int CRUISE_PWM=75, SLOW_PWM=70, TURN_PWM=80, MIN_FORWARD_PWM=65;
 const int MAX_CORRECTION=10, ALIGN_CORRECTION=4;
 const uint32_t START_DELAY_MS=3000, HOLD_MS=350, BLIND_MS=800;
 const uint32_t MIN_TURN_MS=150, MANEUVER_MS=12000, RETRY_PAUSE_MS=500;
 const uint32_t ALIGN_MS=600;
 const uint8_t MAX_RETRIES=2;
}
struct MotorPair { int left; int right; };
class SmoothControl {
public:
 enum State {STARTUP,FOLLOW,TURN_LEFT,ALIGN,SEARCH_RIGHT,SENSOR_WAIT,FRONT_HOLD,RECOVERY_PAUSE,BLOCKED,STOPPED};
 void boot(uint32_t now){reset(now);state_=STARTUP;}
 void start(uint32_t now){reset(now);state_=FOLLOW;}
 void stop(){state_=STOPPED;resetPid();}
 void front(float cm,uint32_t now){
   if(isfinite(cm)&&cm>Tune::MAX_RELEVANT_FRONT_CM)cm=Tune::MAX_RELEVANT_FRONT_CM;
   front_.sample(cm,now);
   if(!frontClear(now)){stable_=0;previousStable_=NAN;}
 }
 void side(float cm,uint32_t now){
   right_.sample(cm,now); ++sideSequence_;
   lost_=(!isfinite(cm)||cm>Tune::WALL_LOST_CM)?inc(lost_):0;
   bool usable=isfinite(cm)&&cm>=Tune::STABLE_MIN_CM&&cm<=Tune::STABLE_MAX_CM;
   if(state_==ALIGN||state_==SEARCH_RIGHT){
     if(!usable||!frontClear(now)){stable_=0;previousStable_=NAN;}
     else {
       stable_=isfinite(previousStable_)&&fabsf(cm-previousStable_)<=Tune::STABLE_CHANGE_CM?inc(stable_):1;
       previousStable_=cm;
     }
   }
 }
 State state()const{return state_;}
 uint8_t stableReadings()const{return stable_;}
 MotorPair step(uint32_t now){
   using namespace Tune;
   if(state_==STOPPED||state_==BLOCKED)return {0,0};
   if(state_==STARTUP){if(age(now,entered_)<START_DELAY_MS)return {0,0};state_=FOLLOW;}
   // A good side reading cannot establish that the FRONT is clear.
   if(!front_.latestValid||!front_.recent(now,HOLD_MS)||front_.fault||right_.fault){
     if(state_!=SENSOR_WAIT){suspended_=state_;state_=SENSOR_WAIT;stable_=0;previousStable_=NAN;resetPid();}
     return {0,0};
   }
   if(state_==SENSOR_WAIT)state_=suspended_;
   if(front_.cm<=FRONT_STOP_CM)frontHold_=true;
   if(frontHold_){
     if(front_.cm<FRONT_RELEASE_CM){state_=FRONT_HOLD;return {0,0};}
     frontHold_=false;state_=FOLLOW;resetPid();
   }
   if(state_==RECOVERY_PAUSE){
     if(age(now,entered_)<RETRY_PAUSE_MS)return {0,0};
     maneuver_=false;state_=FOLLOW;
   }
   if(maneuver_&&age(now,attemptStarted_)>=MANEUVER_MS){
     if(retries_>=MAX_RETRIES){state_=BLOCKED;return {0,0};}
     ++retries_;state_=RECOVERY_PAUSE;entered_=now;resetPid();return {0,0};
   }
   bool near=front_.recent(now,HOLD_MS)&&front_.cm<=FRONT_TURN_CM;
   if(near&&state_!=TURN_LEFT)enter(TURN_LEFT,now);
   if(state_==TURN_LEFT){
     if(age(now,entered_)<MIN_TURN_MS||!frontClear(now))return {-TURN_PWM,TURN_PWM};
     // Stable readings while stopped do not establish heading. Keep moving
     // slowly and use the distance trend for PID alignment instead.
     enter(ALIGN,now);
   }
   if(lost_>=2&&(state_==FOLLOW||(state_==ALIGN&&age(now,entered_)>=ALIGN_MS)))
     enter(SEARCH_RIGHT,now);
   if(state_==SEARCH_RIGHT){
     if(age(now,entered_)>=MIN_TURN_MS&&sideCandidate(now)&&stable_>=3&&frontClear(now))
       enter(ALIGN,now);
     // A stopped inner wheel gives a definite slow right turn. The former
     // 75/65 command was almost straight on this chassis.
     else return {TURN_PWM,0};
   }
   if(state_==ALIGN&&age(now,entered_)>=ALIGN_MS&&stable_>=3&&frontClear(now))finishCorner();
   int base=state_==ALIGN?SLOW_PWM:CRUISE_PWM;
   // Slow down before the turn threshold rather than braking at it.
   if(!front_.recent(now,HOLD_MS))base=SLOW_PWM;
   else if(front_.cm<40){
     float factor=bound((front_.cm-FRONT_TURN_CM)/(40-FRONT_TURN_CM),0,1);
     int approach=(int)roundf(SLOW_PWM+factor*(CRUISE_PWM-SLOW_PWM));
     if(base>approach)base=approach;
   }
   return pidDrive(now,base);
 }
private:
 struct Range{
   float cm=0;uint32_t at=0;bool seen=false,fault=false,latestValid=false;
   void sample(float x,uint32_t t){fault=isfinite(x)&&x<0;latestValid=isfinite(x)&&x>0;if(latestValid){cm=x;at=t;seen=true;}}
   bool recent(uint32_t t,uint32_t span)const{return seen&&!fault&&uint32_t(t-at)<span;}
 }front_,right_;
 State state_=STOPPED,suspended_=FOLLOW;
 uint32_t entered_=0,attemptStarted_=0,sideSequence_=0,pidSequence_=0,pidAt_=0;
 uint8_t stable_=0,lost_=0,retries_=0;
 bool maneuver_=false,pidReady_=false,frontHold_=false;
 float previousStable_=NAN,filteredSide_=0,previousError_=0,integral_=0,derivative_=0;
 int correction_=0;
 static uint32_t age(uint32_t a,uint32_t b){return uint32_t(a-b);}
 static uint8_t inc(uint8_t n){return n<3?n+1:3;}
 static float bound(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
 bool frontClear(uint32_t now)const{
   return front_.latestValid&&front_.recent(now,Tune::HOLD_MS)&&front_.cm>=Tune::FRONT_CLEAR_CM;
 }
 bool sideCandidate(uint32_t now)const{
   return right_.latestValid&&right_.recent(now,Tune::HOLD_MS)&&
     right_.cm>=Tune::STABLE_MIN_CM&&right_.cm<=Tune::STABLE_MAX_CM;
 }
 void resetPid(){pidReady_=false;integral_=derivative_=0;correction_=0;}
 void reset(uint32_t now){
   front_=Range();right_=Range();entered_=now;stable_=lost_=retries_=0;
   maneuver_=false;frontHold_=false;sideSequence_=0;previousStable_=NAN;resetPid();
 }
 void enter(State s,uint32_t now){
   state_=s;entered_=now;stable_=0;previousStable_=NAN;resetPid();
   if(!maneuver_){maneuver_=true;attemptStarted_=now;}
 }
 void finishCorner(){state_=FOLLOW;maneuver_=false;retries_=0;resetPid();}
 MotorPair pidDrive(uint32_t now,int base){
   using namespace Tune;
   int limit=state_==ALIGN?ALIGN_CORRECTION:MAX_CORRECTION;
   if(!right_.recent(now,HOLD_MS)||right_.cm>MAX_RELEVANT_SIDE_CM){resetPid();return {base,base};}
   // Only fresh side pings update I and D. Faster loop ticks reuse output.
   if(!pidReady_||pidSequence_!=sideSequence_){
     float dt=pidReady_?age(right_.at,pidAt_)/1000.0f:0;
     filteredSide_=pidReady_?filteredSide_+SIDE_FILTER*(right_.cm-filteredSide_):right_.cm;
     float error=filteredSide_-TARGET_CM;
     if(fabsf(error)<0.75f)error=0;
     if(dt>0&&dt<1){
       float rate=(error-previousError_)/dt;
       derivative_+=dt/(DERIVATIVE_FILTER_SECONDS+dt)*(rate-derivative_);
       float candidate=bound(integral_+error*dt,-30,30);
       float proposed=KP*error+KI*candidate+KD*derivative_;
       if(fabsf(proposed)<=limit||(proposed>limit&&error<0)||
          (proposed<-limit&&error>0))integral_=candidate;
     }else derivative_=0;
     correction_=(int)roundf(bound(KP*error+KI*integral_+KD*derivative_,-limit,limit));
     previousError_=error;pidAt_=right_.at;pidSequence_=sideSequence_;pidReady_=true;
   }
   return {(int)bound(base+correction_,MIN_FORWARD_PWM,110),(int)bound(base-correction_,MIN_FORWARD_PWM,110)};
 }
};


// UNO R4 WiFi + L298N. Motor-wheel end is forward, as in your reference.
const int LEFT_ENABLE=10, LEFT_IN1=A4, LEFT_IN2=A3;
const int RIGHT_ENABLE=9, RIGHT_IN1=A0, RIGHT_IN2=A1;
const int FRONT_TRIG=5, FRONT_ECHO=6, SIDE_TRIG=7, SIDE_ECHO=8;
const int MOTOR_POLARITY=1;
const unsigned long PING_INTERVAL_MS=60, MOTOR_INTERVAL_MS=20;
const int RAMP_PWM_PER_TICK=5; // Five PWM counts per 20 ms, not an instant jump.

SmoothControl controller;
MotorPair applied={0,0};
int previousLeftPWM=0,previousRightPWM=0;
bool frontNext=true;
unsigned long lastPing=0,lastMotor=0,lastTelemetry=0;
float frontCm=NAN,rightCm=NAN;

void setOneMotor(int enable,int in1,int in2,int pwm,int &previous){
  pwm=constrain(pwm*MOTOR_POLARITY,-255,255);
  // Keep PWM running during ordinary speed adjustments. Disable only before
  // changing direction; the old code briefly disabled it on every update.
  if((pwm>0&&previous<0)||(pwm<0&&previous>0))analogWrite(enable,0);
  digitalWrite(in1,pwm<0?HIGH:LOW);
  digitalWrite(in2,pwm>0?HIGH:LOW);
  analogWrite(enable,abs(pwm));
  previous=pwm;
}
void writeMotors(){
  setOneMotor(LEFT_ENABLE,LEFT_IN1,LEFT_IN2,applied.left,previousLeftPWM);
  setOneMotor(RIGHT_ENABLE,RIGHT_IN1,RIGHT_IN2,applied.right,previousRightPWM);
}
void immediateStop(){applied={0,0};writeMotors();}
int approachPwm(int current,int target,int amount){
  if(current<target)return min(current+amount,target);
  return max(current-amount,target);
}
float readSonar(int trig,int echo){
  if(digitalRead(echo)==HIGH)return -1; // Stuck echo is a fault, not open space.
  digitalWrite(trig,LOW);delayMicroseconds(2);
  digitalWrite(trig,HIGH);delayMicroseconds(10);digitalWrite(trig,LOW);
  unsigned long duration=pulseIn(echo,HIGH,25000UL);
  if(!duration)return digitalRead(echo)==HIGH?-1.0f:NAN;
  // Keep very close echoes as obstacles rather than discarding them.
  return constrain(duration*0.0343f/2.0f,1.0f,400.0f);
}
void sampleSensors(){
  unsigned long now=millis();
  if(now-lastPing<PING_INTERVAL_MS)return;
  lastPing=now;
  if(frontNext){frontCm=readSonar(FRONT_TRIG,FRONT_ECHO);controller.front(frontCm,millis());}
  else {rightCm=readSonar(SIDE_TRIG,SIDE_ECHO);controller.side(rightCm,millis());}
  frontNext=!frontNext;
}
const char* stateName(){
  switch(controller.state()){
    case SmoothControl::STARTUP:return "startup";
    case SmoothControl::FOLLOW:return "pid_follow";
    case SmoothControl::TURN_LEFT:return "left_corner";
    case SmoothControl::ALIGN:return "pid_align";
    case SmoothControl::SEARCH_RIGHT:return "right_edge_turn";
    case SmoothControl::SENSOR_WAIT:return "sensor_wait";
    case SmoothControl::FRONT_HOLD:return "front_too_close";
    case SmoothControl::RECOVERY_PAUSE:return "retry_pause";
    case SmoothControl::BLOCKED:return "blocked_check_motors";
    default:return "stopped";
  }
}
void printTelemetry(){
  if(!Serial||millis()-lastTelemetry<500)return;
  lastTelemetry=millis();
  Serial.print(lastTelemetry);Serial.print(',');
  Serial.print(frontCm,1);Serial.print(',');Serial.print(rightCm,1);Serial.print(',');
  Serial.print(stateName());Serial.print(',');
  Serial.print(applied.left);Serial.print(',');Serial.print(applied.right);Serial.print(',');
  Serial.println(controller.stableReadings());
}
void setup(){
  pinMode(LEFT_ENABLE,OUTPUT);pinMode(LEFT_IN1,OUTPUT);pinMode(LEFT_IN2,OUTPUT);
  pinMode(RIGHT_ENABLE,OUTPUT);pinMode(RIGHT_IN1,OUTPUT);pinMode(RIGHT_IN2,OUTPUT);
  pinMode(FRONT_TRIG,OUTPUT);pinMode(FRONT_ECHO,INPUT);
  pinMode(SIDE_TRIG,OUTPUT);pinMode(SIDE_ECHO,INPUT);
  digitalWrite(FRONT_TRIG,LOW);digitalWrite(SIDE_TRIG,LOW);
  immediateStop();Serial.begin(9600); // No while(!Serial): battery operation.
  controller.boot(millis());lastMotor=millis();lastPing=millis()-PING_INTERVAL_MS;
  if(Serial)Serial.println("ms,front_cm,right_cm,state,left_pwm,right_pwm,stable_side_samples");
}
void loop(){
  sampleSensors();
  unsigned long now=millis();
  unsigned long dt=now-lastMotor;
  if(dt>=MOTOR_INTERVAL_MS){
    lastMotor=now;
    SmoothControl::State before=controller.state();
    MotorPair target=controller.step(now);
    // Cancel forward motion BEFORE ramping into a pivot. Otherwise a slow
    // sign reversal keeps driving into the obstacle for several motor ticks.
    // Also cancel rotation on EXIT; do not slew through negative PWM while
    // already trying to confirm a heading or drive forward.
    bool turningBefore=before==SmoothControl::TURN_LEFT||before==SmoothControl::SEARCH_RIGHT;
    bool turningNow=controller.state()==SmoothControl::TURN_LEFT||controller.state()==SmoothControl::SEARCH_RIGHT;
    if(before!=controller.state()&&(turningBefore||turningNow)){
      applied={0,0};writeMotors();
    }
    if(target.left==0&&target.right==0)applied=target;
    else{
      int amount=(int)(min(dt,60UL)*RAMP_PWM_PER_TICK/MOTOR_INTERVAL_MS);
      applied.left=approachPwm(applied.left,target.left,amount);
      applied.right=approachPwm(applied.right,target.right,amount);
    }
    writeMotors();
  }
  printTelemetry();
}
