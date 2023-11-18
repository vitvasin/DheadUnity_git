# DheadUnity_git
 
This Version for 3 DOF (Roll, Pitch,Yaw)
 - Disable motor ID 1,2 (Flexion/Bending)
 - Disable Linear actuator

/*
//25-4-2022 Update Simplify version ->> remove unnecessary version
  // Not test on HW yet!
  
-remove scservo
-remove publisher feedback and  feedback2

*/
//rev3- This version adjust parameter with JOI unity code
//30-7-2021 -rev4- fix reverse pitch angle line 55 >>> swap 0 and 1023
//18-8-2021 -rev5- add manual calibration function >> home_adj();
// Need to use sync write >>tested