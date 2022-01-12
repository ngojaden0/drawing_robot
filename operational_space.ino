#include <math.h>
#include "readPinFast.h"
#include "enc_1.h"
#include "mot_1.h"
#include "enc_4.h"
#include "mot_4.h"
#include "parameters.h"
#include "calc_q2_q3.h"

void setup() 
{
  Serial.begin(9600);
  enc1Setup();
  motor1Setup();
  enc4Setup();
  motor4Setup();
  while (!Serial);
}

double GRATIO = 144.0;
double Q1_init = M_PI/2, Q1_old = Q1_init, Q1, U1;
double Q4_init = M_PI/2, Q4_old = Q4_init, Q4, U4;
double Q2,Q3,U2,U3;
double t_old = 0.0, t_new, t_span;
double KP = 4, KV = 2*sqrt(KP);
double f = 0.3, Rc = 5.0/3.0, r = 3.0/3.0, d = 1.7/3.0, x0 = L_GR/2.0, y0 = L_GR, th;
double PXD, VXD=0.0, AXD=0.0;
double PYD, VYD=0.0, AYD=0.0;
double TQ1,TQ4;
double Vs1,Vs4;
double w = 0.0;
double a = 0.0;
double s = 0.0;
double D = 0.0;

void loop() 
{
  
  
  t_new = millis()/1000.0; Q1 = Q1_init+getEnc1(); Q4 = Q4_init+getEnc4();
  t_span = t_new - t_old; 
  U1 = (Q1-Q1_old)/t_span; 
  U4 = (Q4-Q4_old)/t_span;
  t_old = t_new; Q1_old = Q1; Q4_old = Q4;
  
  calc_q2_q3(Q1,Q4,L_S,L_L,L_L,L_S,L_GR,Q2,Q3);
  U2 = -L_S*(sin(Q1-Q3)*U1+sin(Q3-Q4)*U4)/(L_L*sin(Q2-Q3));
  U3 = -L_S*(sin(Q1-Q2)*U1+sin(Q2-Q4)*U4)/(L_L*sin(Q2-Q3));

  th = 2*PI*f*t_new;

  PXD = x0;
  PYD = y0; 
  
  if (Serial.available() > 0) {  
  char str = Serial.read();
  if(str == 'w')
    y0 -= 0.01;
  else if(str == 'a')
    x0 -= 0.01;
  else if(str == 's')
    y0 += 0.01;
  else if(str == 'd')
    x0 += 0.01;
  }
  
  //-------------------------------------------------------------------------------
  TQ1 = 0.08333333333333333*(3*M_S_LINK*pow(L_S,2)+6*M_GEAR*pow(R_GEAR,2)+6*
  M_ROTOR*pow(GRATIO,2)*pow(R_ROTOR,2)+M_S_LINK*(pow(L_S,2)+pow(W_S,2))+6*
  M_L_LINK*pow(L_S,2)*(2-sin(Q1-Q3)*cos(Q1-Q2)/sin(Q2-Q3))+M_L_LINK*pow(L_S,2)*(
  pow(sin(Q1-Q2),2)*(3*L_L/sin(Q2-Q3)+(pow(L_L,2)+pow(W_L,2))/(L_L*sin(Q2-Q3)))+
  sin(Q1-Q3)*((pow(L_L,2)+pow(W_L,2))*sin(Q1-Q3)/(L_L*sin(Q2-Q3))-3*L_L*(2*
  cos(Q1-Q2)-sin(Q1-Q3)/sin(Q2-Q3))))/(L_L*sin(Q2-Q3)))*(cos(Q2)*(AXD+KP*(PXD-
  L_L*cos(Q2)-L_S*cos(Q1))+KV*(VXD+L_L*sin(Q2)*U2+L_S*sin(Q1)*U1)-L_S*(U4*(
  cos(Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2+sin(Q2)*sin(Q2-Q3)*cos(Q3-Q4)*(U3-U4)-sin(
  Q2)*sin(Q3-Q4)*cos(Q2-Q3)*(U2-U3))/pow(sin(Q2-Q3),2)-U1*(cos(Q1)*U1+(sin(Q2)*
  sin(Q1-Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*sin(Q1-Q3)*sin(Q2-Q3)*U2-sin(Q2)*sin(
  Q2-Q3)*cos(Q1-Q3)*(U1-U3))/pow(sin(Q2-Q3),2))))+sin(Q2)*(AYD+KP*(PYD-L_L*
  sin(Q2)-L_S*sin(Q1))+KV*(VYD-L_L*cos(Q2)*U2-L_S*cos(Q1)*U1)+L_S*(U4*(cos(Q2)*
  sin(Q2-Q3)*cos(Q3-Q4)*(U3-U4)-sin(Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2-cos(Q2)*sin(
  Q3-Q4)*cos(Q2-Q3)*(U2-U3))/pow(sin(Q2-Q3),2)+U1*(sin(Q1)*U1-(sin(Q2)*sin(Q1-
  Q3)*sin(Q2-Q3)*U2+cos(Q2)*sin(Q1-Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*sin(Q2-Q3)*
  cos(Q1-Q3)*(U1-U3))/pow(sin(Q2-Q3),2)))))/(L_S*(sin(Q2)*(cos(Q1)-cos(Q2)*
  sin(Q1-Q3)/sin(Q2-Q3))-cos(Q2)*(sin(Q1)-sin(Q2)*sin(Q1-Q3)/sin(Q2-Q3)))) - 
  0.08333333333333333*L_S*M_L_LINK*(6*L_L*sin(Q1-Q2)*pow(U2,2)+6*cos(Q1-Q2)*(
  L_L*(pow(U3,2)-cos(Q2-Q3)*pow(U2,2))-L_S*(cos(Q1-Q3)*pow(U1,2)-cos(Q3-Q4)*
  pow(U4,2)))/sin(Q2-Q3)+(sin(Q1-Q2)*((pow(L_L,2)+pow(W_L,2))*((pow(U2,2)-cos(
  Q2-Q3)*pow(U3,2))/sin(Q2-Q3)+L_S*(cos(Q1-Q2)*pow(U1,2)-cos(Q2-Q4)*pow(U4,2))/(
  L_L*sin(Q2-Q3)))-3*L_L*(2*L_S*sin(Q3-Q4)*pow(U4,2)-(L_L*(pow(U2,2)-cos(Q2-
  Q3)*pow(U3,2))+L_S*(cos(Q1-Q2)*pow(U1,2)-cos(Q2-Q4)*pow(U4,2)))/sin(Q2-Q3)))-
  sin(Q1-Q3)*((pow(L_L,2)+pow(W_L,2))*((pow(U3,2)-cos(Q2-Q3)*pow(U2,2))/sin(
  Q2-Q3)-L_S*(cos(Q1-Q3)*pow(U1,2)-cos(Q3-Q4)*pow(U4,2))/(L_L*sin(Q2-Q3)))-3*
  L_L*(2*L_S*sin(Q1-Q2)*pow(U1,2)-(L_L*(pow(U3,2)-cos(Q2-Q3)*pow(U2,2))-L_S*(
  cos(Q1-Q3)*pow(U1,2)-cos(Q3-Q4)*pow(U4,2)))/sin(Q2-Q3))))/(L_L*sin(Q2-Q3))) - 
  0.08333333333333333*L_S*M_L_LINK*(6*sin(Q3-Q4)*cos(Q1-Q2)/sin(Q2-Q3)-(sin(
  Q1-Q3)*sin(Q3-Q4)*(3*L_L/sin(Q2-Q3)+(pow(L_L,2)+pow(W_L,2))/(L_L*sin(Q2-Q3)))+
  sin(Q1-Q2)*((pow(L_L,2)+pow(W_L,2))*sin(Q2-Q4)/(L_L*sin(Q2-Q3))-3*L_L*(2*
  cos(Q3-Q4)-sin(Q2-Q4)/sin(Q2-Q3))))/(L_L*sin(Q2-Q3)))*((cos(Q1)*sin(Q2-Q3)-
  cos(Q2)*sin(Q1-Q3))*(AXD+KP*(PXD-L_L*cos(Q2)-L_S*cos(Q1))+KV*(VXD+L_L*sin(
  Q2)*U2+L_S*sin(Q1)*U1)-L_S*(U4*(cos(Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2+sin(Q2)*
  sin(Q2-Q3)*cos(Q3-Q4)*(U3-U4)-sin(Q2)*sin(Q3-Q4)*cos(Q2-Q3)*(U2-U3))/pow(
  sin(Q2-Q3),2)-U1*(cos(Q1)*U1+(sin(Q2)*sin(Q1-Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*
  sin(Q1-Q3)*sin(Q2-Q3)*U2-sin(Q2)*sin(Q2-Q3)*cos(Q1-Q3)*(U1-U3))/pow(sin(Q2-
  Q3),2))))+(sin(Q1)*sin(Q2-Q3)-sin(Q2)*sin(Q1-Q3))*(AYD+KP*(PYD-L_L*sin(Q2)-
  L_S*sin(Q1))+KV*(VYD-L_L*cos(Q2)*U2-L_S*cos(Q1)*U1)+L_S*(U4*(cos(Q2)*sin(Q2-
  Q3)*cos(Q3-Q4)*(U3-U4)-sin(Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2-cos(Q2)*sin(Q3-Q4)*
  cos(Q2-Q3)*(U2-U3))/pow(sin(Q2-Q3),2)+U1*(sin(Q1)*U1-(sin(Q2)*sin(Q1-Q3)*
  sin(Q2-Q3)*U2+cos(Q2)*sin(Q1-Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*sin(Q2-Q3)*cos(
  Q1-Q3)*(U1-U3))/pow(sin(Q2-Q3),2)))))/(sin(Q3-Q4)*(sin(Q2)*(cos(Q1)-cos(Q2)*
  sin(Q1-Q3)/sin(Q2-Q3))-cos(Q2)*(sin(Q1)-sin(Q2)*sin(Q1-Q3)/sin(Q2-Q3))));

  //------------------------------------------------------------------------------

  TQ4 = 0.08333333333333333*L_S*M_L_LINK*(6*L_L*sin(Q3-Q4)*pow(U3,2)+6*cos(Q3-
  Q4)*(L_L*(pow(U2,2)-cos(Q2-Q3)*pow(U3,2))+L_S*(cos(Q1-Q2)*pow(U1,2)-cos(Q2-
  Q4)*pow(U4,2)))/sin(Q2-Q3)-(sin(Q2-Q4)*((pow(L_L,2)+pow(W_L,2))*((pow(U2,2)-
  cos(Q2-Q3)*pow(U3,2))/sin(Q2-Q3)+L_S*(cos(Q1-Q2)*pow(U1,2)-cos(Q2-Q4)*pow(
  U4,2))/(L_L*sin(Q2-Q3)))-3*L_L*(2*L_S*sin(Q3-Q4)*pow(U4,2)-(L_L*(pow(U2,2)-
  cos(Q2-Q3)*pow(U3,2))+L_S*(cos(Q1-Q2)*pow(U1,2)-cos(Q2-Q4)*pow(U4,2)))/sin(
  Q2-Q3)))-sin(Q3-Q4)*((pow(L_L,2)+pow(W_L,2))*((pow(U3,2)-cos(Q2-Q3)*pow(U2,
  2))/sin(Q2-Q3)-L_S*(cos(Q1-Q3)*pow(U1,2)-cos(Q3-Q4)*pow(U4,2))/(L_L*sin(Q2-
  Q3)))-3*L_L*(2*L_S*sin(Q1-Q2)*pow(U1,2)-(L_L*(pow(U3,2)-cos(Q2-Q3)*pow(U2,2))-
  L_S*(cos(Q1-Q3)*pow(U1,2)-cos(Q3-Q4)*pow(U4,2)))/sin(Q2-Q3))))/(L_L*sin(Q2-
  Q3))) - 0.08333333333333333*L_S*M_L_LINK*(6*sin(Q1-Q2)*cos(Q3-Q4)/sin(Q2-Q3)-(
  sin(Q1-Q2)*sin(Q2-Q4)*(3*L_L/sin(Q2-Q3)+(pow(L_L,2)+pow(W_L,2))/(L_L*sin(Q2-
  Q3)))+sin(Q3-Q4)*((pow(L_L,2)+pow(W_L,2))*sin(Q1-Q3)/(L_L*sin(Q2-Q3))-3*L_L*(
  2*cos(Q1-Q2)-sin(Q1-Q3)/sin(Q2-Q3))))/(L_L*sin(Q2-Q3)))*(cos(Q2)*(AXD+KP*(
  PXD-L_L*cos(Q2)-L_S*cos(Q1))+KV*(VXD+L_L*sin(Q2)*U2+L_S*sin(Q1)*U1)-L_S*(U4*(
  cos(Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2+sin(Q2)*sin(Q2-Q3)*cos(Q3-Q4)*(U3-U4)-sin(
  Q2)*sin(Q3-Q4)*cos(Q2-Q3)*(U2-U3))/pow(sin(Q2-Q3),2)-U1*(cos(Q1)*U1+(sin(Q2)*
  sin(Q1-Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*sin(Q1-Q3)*sin(Q2-Q3)*U2-sin(Q2)*sin(
  Q2-Q3)*cos(Q1-Q3)*(U1-U3))/pow(sin(Q2-Q3),2))))+sin(Q2)*(AYD+KP*(PYD-L_L*
  sin(Q2)-L_S*sin(Q1))+KV*(VYD-L_L*cos(Q2)*U2-L_S*cos(Q1)*U1)+L_S*(U4*(cos(Q2)*
  sin(Q2-Q3)*cos(Q3-Q4)*(U3-U4)-sin(Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2-cos(Q2)*sin(
  Q3-Q4)*cos(Q2-Q3)*(U2-U3))/pow(sin(Q2-Q3),2)+U1*(sin(Q1)*U1-(sin(Q2)*sin(Q1-
  Q3)*sin(Q2-Q3)*U2+cos(Q2)*sin(Q1-Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*sin(Q2-Q3)*
  cos(Q1-Q3)*(U1-U3))/pow(sin(Q2-Q3),2)))))/(sin(Q2)*(cos(Q1)-cos(Q2)*sin(Q1-
  Q3)/sin(Q2-Q3))-cos(Q2)*(sin(Q1)-sin(Q2)*sin(Q1-Q3)/sin(Q2-Q3))) - 0.08333333333333333*(
  6*M_L_LINK*pow(L_S,2)*(-2+sin(Q2-Q4)*cos(Q3-Q4)/sin(Q2-Q3))-6*M_GEAR*pow(
  R_GEAR,2)-3*M_S_LINK*pow(L_S,2)-6*M_ROTOR*pow(GRATIO,2)*pow(R_ROTOR,2)-
  M_S_LINK*(pow(L_S,2)+pow(W_S,2))-M_L_LINK*pow(L_S,2)*(pow(sin(Q3-Q4),2)*(3*
  L_L/sin(Q2-Q3)+(pow(L_L,2)+pow(W_L,2))/(L_L*sin(Q2-Q3)))+sin(Q2-Q4)*((pow(
  L_L,2)+pow(W_L,2))*sin(Q2-Q4)/(L_L*sin(Q2-Q3))-3*L_L*(2*cos(Q3-Q4)-sin(Q2-
  Q4)/sin(Q2-Q3))))/(L_L*sin(Q2-Q3)))*((cos(Q1)*sin(Q2-Q3)-cos(Q2)*sin(Q1-Q3))*(
  AXD+KP*(PXD-L_L*cos(Q2)-L_S*cos(Q1))+KV*(VXD+L_L*sin(Q2)*U2+L_S*sin(Q1)*U1)-
  L_S*(U4*(cos(Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2+sin(Q2)*sin(Q2-Q3)*cos(Q3-Q4)*(U3-
  U4)-sin(Q2)*sin(Q3-Q4)*cos(Q2-Q3)*(U2-U3))/pow(sin(Q2-Q3),2)-U1*(cos(Q1)*U1+(
  sin(Q2)*sin(Q1-Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*sin(Q1-Q3)*sin(Q2-Q3)*U2-sin(
  Q2)*sin(Q2-Q3)*cos(Q1-Q3)*(U1-U3))/pow(sin(Q2-Q3),2))))+(sin(Q1)*sin(Q2-Q3)-
  sin(Q2)*sin(Q1-Q3))*(AYD+KP*(PYD-L_L*sin(Q2)-L_S*sin(Q1))+KV*(VYD-L_L*cos(
  Q2)*U2-L_S*cos(Q1)*U1)+L_S*(U4*(cos(Q2)*sin(Q2-Q3)*cos(Q3-Q4)*(U3-U4)-sin(
  Q2)*sin(Q2-Q3)*sin(Q3-Q4)*U2-cos(Q2)*sin(Q3-Q4)*cos(Q2-Q3)*(U2-U3))/pow(sin(
  Q2-Q3),2)+U1*(sin(Q1)*U1-(sin(Q2)*sin(Q1-Q3)*sin(Q2-Q3)*U2+cos(Q2)*sin(Q1-
  Q3)*cos(Q2-Q3)*(U2-U3)-cos(Q2)*sin(Q2-Q3)*cos(Q1-Q3)*(U1-U3))/pow(sin(Q2-Q3),
  2)))))/(L_S*sin(Q3-Q4)*(sin(Q2)*(cos(Q1)-cos(Q2)*sin(Q1-Q3)/sin(Q2-Q3))-cos(
  Q2)*(sin(Q1)-sin(Q2)*sin(Q1-Q3)/sin(Q2-Q3))));

  Vs1 = TQ1/kt*R + ki*U1;
  Vs4 = TQ4/kt*R + ki*U4;  
  
  if(Vs1 < 0)
  {
    digitalWrite(motAdir, HIGH);
  }
  else
  {
    digitalWrite(motAdir, LOW);
  }
  if(Vs4 < 0)
  {
    digitalWrite(motBdir, HIGH);
  }
  else
  {
    digitalWrite(motBdir, LOW);
  }
  
  Vs1 = abs(Vs1);
  Vs4 = abs(Vs4);
  Vs1 = constrain(Vs1, 0, 12);
  Vs4 = constrain(Vs4, 0, 12);
  Vs1 = map(Vs1, 0, 12, 0, 255);
  Vs4 = map(Vs4, 0, 12, 0, 255);  
  analogWrite(motApwm, Vs1);
  analogWrite(motBpwm, Vs4);
  
  
  
}
