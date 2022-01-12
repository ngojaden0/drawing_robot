////////////////    Setup     ////////////////////////////////
//       N2>
//       ^
//       |
//       |
//         E
//        /\
//      B/  \C
//      /    \
//     /      \
// J1 O        O J2
//     \      /
//     A\    /D
//       o--o                   ------>N1
//      N    F
// Body A = Lower link on left side
// Body B = Upper link on left side
// Body C = Lower link on right side
// Body D = Upper link on right side
// A1 axis = Direction from point N to point J1
// B1 axis = Direction from point J1 to point E
// C1 axis = Direction from point J2 to point E
// D1 axis = Direction from point F to point J2
// q1 = Angle between N1> and A1>
// q2 = Angle between N1> and B1>
// q3 = Angle between N1> and C1>
// q4 = Angle between N1> and D1>
// l1 = Length of link A
// l2 = Length of link B
// l3 = Length of link C
// l4 = Length of link D
// l5 = Distance between points N and F
//
// Usage: If Q1 through Q4 are the variables that store
// the angles and L1 through L5 are the link lengths as
// specified, then use following line to calculate and
// update the variables Q2 and Q3 which represent the other
// two angles.
// calc_q2_q3(Q1,Q4,L1,L2,L3,L4,L5,Q2,Q3)
//////////////////////////////////////////////////////////////

void calc_q2_q3(double q1, double q4, double l1, double l2, double l3, double l4, double l5, double &q2, double &q3)
{
  double a = l3/l2, b = (l4*cos(q4)+l5-l1*cos(q1))/l2, c = (l4*sin(q4)-l1*sin(q1))/l2;
  q2 = 2*atan((2*c + pow(- pow(a,4) + 2*pow(a,2)*pow(b,2) + 2*pow(a,2)*pow(c,2) + 2*pow(a,2) - pow(b,4) - 2*pow(b,2)*pow(c,2) + 2*pow(b,2) - pow(c,4) + 2*pow(c,2) - 1,0.5))/(- pow(a,2) + pow(b,2) + 2*b + pow(c,2) + 1));
  q3 = -2*atan((2*a*c + pow((pow(a,2) + 2*a - pow(b,2) - pow(c,2) + 1)*(- pow(a,2) + 2*a + pow(b,2) + pow(c,2) - 1),0.5))/(pow(a,2) - 2*a*b + pow(b,2) + pow(c,2) - 1));
}