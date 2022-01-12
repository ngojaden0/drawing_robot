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
double kp = 100, kv = 2*sqrt(kp);

double q1_des = M_PI;
double q1d_des = 0.0;
double q1dd_des = 0.0;

double q1dd;
double q1_old = q1_des - q1, q1_new, q1d;
double TQ;
double ki = (10.52-0.108605472*4.5)/45;
double kt = 0.980665/2.666666666;
double Vs;
double R = 4.5;
double M_LINK = 0.0059147;
double M_ROTOR = 0.1035074*0.7;
double M_GEAR = 0.0532324+0.0088721;
double R_ROTOR = 0.015, R_GEAR = 0.01;
double L = 0.1, W = 0.05;


void loop() 
{
  q1_new = getEnc1();

  t_new = millis()/1000.0; //tf always up to date in seconds
  t_span = t_new - t_old; //dt, tf-to
  t_old = t_new; //update to 
  
  q1d = (q1_new - q1_old)/t_span;
  q1_old = q1_new;

  q1dd = q1dd_des + kv*(q1d_des - q1d) + kp*(q1_des-q1_new);
  TQ = 0.009157509*(3.0*M_LINK*pow(L,2)+6.0*M_GEAR*pow(R_GEAR,2) + 48600.0*M_ROTOR*pow(R_ROTOR,2) + M_LINK*(pow(L,2)+pow(W,2)))*q1dd;
  
  Vs = TQ/kt*R + ki*q1d;

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
  
  Serial.print(t_new); Serial.print("\t"); Serial.print(q1_new); Serial.print("\n");
}
