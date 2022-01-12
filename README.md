# drawing_robot

Arduino code for a 2 DOF drawing robot using two motors. 

Requirements:
- 3D printed linkages. Check lengths of links to avoid constraints
- 2 Motors (Gear ratios calculated)
- Arduino Mega 2560
- Arduino Mega Motor Shield

This Repo includes 3 .ino files for controlling the links
  pd: standard PID controller without the integral term
  joint_space: computed torque method, requires inverse kinematics (calc_q2_q3.h)
  operational_space: torque equations solved taking the inverse of the Jacobian. No need for inverse kinematics
  
//      Setup     
//        N2>
//        ^
//        |
//        |
//         E
//        /\
//      B/  \C
//      /    \
//     /      \
// J1 O        O J2
//     \      /
//     A\    /D
//       o--o          ------>N1>
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
