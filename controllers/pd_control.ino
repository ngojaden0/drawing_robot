#include "readPinFast.h"
#include "enc_1.h"
#include "mot_1.h"

void setup()
{
  Serial.begin(9600);
  enc1Setup();
  motor1Setup(); 
}

double q1 = 0.0; //start from 0, go to desired
double t_old = 0.0, t_new, t_span;
double kp = 50, kv = 2;

double q1_des = M_PI;
double q1d_des = 0.0;
double q1dd_des = 0.0;

double q1d, qprev = getEnc1(), e, edot;
double TQ; 
double ki = (10.52-0.108605472*4.5)/45;
double kt = 0.980665/2.666666666;
double Vs;
double R = 4.5;

void loop() 
{
  q1 = getEnc1();

  t_new = millis()/1000.0; //tf always up to date in seconds
  t_span = t_new - t_old; //dt, tf-to
  t_old = t_new; //update to
  
  q1d = (q1 - qprev)/t_span;
  qprev = q1;

  e = q1_des - q1;
  edot = q1d_des - q1d; 

  TQ = q1dd_des + kv*edot + kp*e;
  
  Vs = TQ/kt*R + ki*-q1d;
    
  if(Vs < 0)
  {
    digitalWrite(motAdir, HIGH);
  }
  else
  {
    digitalWrite(motAdir, LOW);
  }
  
  Vs = abs(Vs);
  Vs = constrain(Vs, 0, 12);
  Vs = map(Vs, 0, 12, 0, 255); 
  analogWrite(motApwm, Vs);
  
  Serial.print(t_new); Serial.print("\t"); Serial.print(q1);  Serial.print("\n");
}
