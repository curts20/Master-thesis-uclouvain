%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Sat Jan 26 16:49:12 2019
%
%	==> Project name : Lunar_Lander_Base
%	==> using XML input file 
%
%	==> Number of joints : 14
%
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1720
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.030 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,14);
 trq = zeros(3,14);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C7 = cos(q(7));
  S7 = sin(q(7));
  C8 = cos(q(8));
  S8 = sin(q(8));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C9 = cos(q(9));
  S9 = sin(q(9));
  C10 = cos(q(10));
  S10 = sin(q(10));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C11 = cos(q(11));
  S11 = sin(q(11));
  C12 = cos(q(12));
  S12 = sin(q(12));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C13 = cos(q(13));
  S13 = sin(q(13));
  C14 = cos(q(14));
  S14 = sin(q(14));

% = = Block_0_0_1_1_0_1 = = 
 
% Sensor Kinematics 


  ROcp1_25 = S4*S5;
  ROcp1_35 = -C4*S5;
  ROcp1_85 = -S4*C5;
  ROcp1_95 = C4*C5;
  ROcp1_16 = C5*C6;
  ROcp1_26 = ROcp1_25*C6+C4*S6;
  ROcp1_36 = ROcp1_35*C6+S4*S6;
  ROcp1_46 = -C5*S6;
  ROcp1_56 = -(ROcp1_25*S6-C4*C6);
  ROcp1_66 = -(ROcp1_35*S6-S4*C6);
  OMcp1_25 = qd(5)*C4;
  OMcp1_35 = qd(5)*S4;
  PxF1(1) = q(1);
  PxF1(2) = q(2);
  PxF1(3) = q(3);
  RxF1(1,1) = ROcp1_16;
  RxF1(1,2) = ROcp1_26;
  RxF1(1,3) = ROcp1_36;
  RxF1(2,1) = ROcp1_46;
  RxF1(2,2) = ROcp1_56;
  RxF1(2,3) = ROcp1_66;
  RxF1(3,1) = S5;
  RxF1(3,2) = ROcp1_85;
  RxF1(3,3) = ROcp1_95;
  VxF1(1) = qd(1);
  VxF1(2) = qd(2);
  VxF1(3) = qd(3);
  OMxF1(1) = qd(4)+qd(6)*S5;
  OMxF1(2) = OMcp1_25+qd(6)*ROcp1_85;
  OMxF1(3) = OMcp1_35+qd(6)*ROcp1_95;
  AxF1(1) = qdd(1);
  AxF1(2) = qdd(2);
  AxF1(3) = qdd(3);
  OMPxF1(1) = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OMPxF1(2) = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp1_95-OMcp1_35*S5)-qdd(5)*C4-qdd(6)*ROcp1_85);
  OMPxF1(3) = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp1_85-OMcp1_25*S5)+qdd(5)*S4+qdd(6)*ROcp1_95;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc12 = ROcp1_16*SWr1(1)+ROcp1_26*SWr1(2)+ROcp1_36*SWr1(3);
  xfrc22 = ROcp1_46*SWr1(1)+ROcp1_56*SWr1(2)+ROcp1_66*SWr1(3);
  xfrc32 = ROcp1_85*SWr1(2)+ROcp1_95*SWr1(3)+SWr1(1)*S5;
  s.frc(1,6) = s.frc(1,6)+xfrc12;
  s.frc(2,6) = s.frc(2,6)+xfrc22;
  s.frc(3,6) = s.frc(3,6)+xfrc32;
  xtrq12 = ROcp1_16*SWr1(4)+ROcp1_26*SWr1(5)+ROcp1_36*SWr1(6);
  xtrq22 = ROcp1_46*SWr1(4)+ROcp1_56*SWr1(5)+ROcp1_66*SWr1(6);
  xtrq32 = ROcp1_85*SWr1(5)+ROcp1_95*SWr1(6)+SWr1(4)*S5;
  s.trq(1,6) = s.trq(1,6)+xtrq12-xfrc22*(SWr1(9)-s.l(3,6))+xfrc32*SWr1(8);
  s.trq(2,6) = s.trq(2,6)+xtrq22+xfrc12*(SWr1(9)-s.l(3,6))-xfrc32*SWr1(7);
  s.trq(3,6) = s.trq(3,6)+xtrq32-xfrc12*SWr1(8)+xfrc22*SWr1(7);

% = = Block_0_0_1_2_0_1 = = 
 
% Sensor Kinematics 


  ROcp2_25 = S4*S5;
  ROcp2_35 = -C4*S5;
  ROcp2_85 = -S4*C5;
  ROcp2_95 = C4*C5;
  ROcp2_16 = C5*C6;
  ROcp2_26 = ROcp2_25*C6+C4*S6;
  ROcp2_36 = ROcp2_35*C6+S4*S6;
  ROcp2_46 = -C5*S6;
  ROcp2_56 = -(ROcp2_25*S6-C4*C6);
  ROcp2_66 = -(ROcp2_35*S6-S4*C6);
  OMcp2_25 = qd(5)*C4;
  OMcp2_35 = qd(5)*S4;
  OMcp2_16 = qd(4)+qd(6)*S5;
  OMcp2_26 = OMcp2_25+qd(6)*ROcp2_85;
  OMcp2_36 = OMcp2_35+qd(6)*ROcp2_95;
  OPcp2_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp2_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp2_95-OMcp2_35*S5)-qdd(5)*C4-qdd(6)*ROcp2_85);
  OPcp2_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp2_85-OMcp2_25*S5)+qdd(5)*S4+qdd(6)*ROcp2_95;
  RLcp2_117 = ROcp2_16*s.dpt(1,7);
  RLcp2_217 = ROcp2_26*s.dpt(1,7);
  RLcp2_317 = ROcp2_36*s.dpt(1,7);
  ORcp2_117 = OMcp2_26*RLcp2_317-OMcp2_36*RLcp2_217;
  ORcp2_217 = -(OMcp2_16*RLcp2_317-OMcp2_36*RLcp2_117);
  ORcp2_317 = OMcp2_16*RLcp2_217-OMcp2_26*RLcp2_117;
  PxF2(1) = q(1)+RLcp2_117;
  PxF2(2) = q(2)+RLcp2_217;
  PxF2(3) = q(3)+RLcp2_317;
  RxF2(1,1) = ROcp2_16;
  RxF2(1,2) = ROcp2_26;
  RxF2(1,3) = ROcp2_36;
  RxF2(2,1) = ROcp2_46;
  RxF2(2,2) = ROcp2_56;
  RxF2(2,3) = ROcp2_66;
  RxF2(3,1) = S5;
  RxF2(3,2) = ROcp2_85;
  RxF2(3,3) = ROcp2_95;
  VxF2(1) = qd(1)+ORcp2_117;
  VxF2(2) = qd(2)+ORcp2_217;
  VxF2(3) = qd(3)+ORcp2_317;
  OMxF2(1) = OMcp2_16;
  OMxF2(2) = OMcp2_26;
  OMxF2(3) = OMcp2_36;
  AxF2(1) = qdd(1)+OMcp2_26*ORcp2_317-OMcp2_36*ORcp2_217+OPcp2_26*RLcp2_317-OPcp2_36*RLcp2_217;
  AxF2(2) = qdd(2)-OMcp2_16*ORcp2_317+OMcp2_36*ORcp2_117-OPcp2_16*RLcp2_317+OPcp2_36*RLcp2_117;
  AxF2(3) = qdd(3)+OMcp2_16*ORcp2_217-OMcp2_26*ORcp2_117+OPcp2_16*RLcp2_217-OPcp2_26*RLcp2_117;
  OMPxF2(1) = OPcp2_16;
  OMPxF2(2) = OPcp2_26;
  OMPxF2(3) = OPcp2_36;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc13 = ROcp2_16*SWr2(1)+ROcp2_26*SWr2(2)+ROcp2_36*SWr2(3);
  xfrc23 = ROcp2_46*SWr2(1)+ROcp2_56*SWr2(2)+ROcp2_66*SWr2(3);
  xfrc33 = ROcp2_85*SWr2(2)+ROcp2_95*SWr2(3)+SWr2(1)*S5;
  s.frc(1,6) = s.frc(1,6)+xfrc13;
  s.frc(2,6) = s.frc(2,6)+xfrc23;
  s.frc(3,6) = s.frc(3,6)+xfrc33;
  xtrq13 = ROcp2_16*SWr2(4)+ROcp2_26*SWr2(5)+ROcp2_36*SWr2(6);
  xtrq23 = ROcp2_46*SWr2(4)+ROcp2_56*SWr2(5)+ROcp2_66*SWr2(6);
  xtrq33 = ROcp2_85*SWr2(5)+ROcp2_95*SWr2(6)+SWr2(4)*S5;
  s.trq(1,6) = s.trq(1,6)+xtrq13-xfrc23*(SWr2(9)-s.l(3,6))+xfrc33*SWr2(8);
  s.trq(2,6) = s.trq(2,6)+xtrq23+xfrc13*(SWr2(9)-s.l(3,6))-xfrc33*SWr2(7);
  s.trq(3,6) = s.trq(3,6)+xtrq33-xfrc13*SWr2(8)+xfrc23*SWr2(7);

% = = Block_0_0_1_3_0_1 = = 
 
% Sensor Kinematics 


  ROcp3_25 = S4*S5;
  ROcp3_35 = -C4*S5;
  ROcp3_85 = -S4*C5;
  ROcp3_95 = C4*C5;
  ROcp3_16 = C5*C6;
  ROcp3_26 = ROcp3_25*C6+C4*S6;
  ROcp3_36 = ROcp3_35*C6+S4*S6;
  ROcp3_46 = -C5*S6;
  ROcp3_56 = -(ROcp3_25*S6-C4*C6);
  ROcp3_66 = -(ROcp3_35*S6-S4*C6);
  OMcp3_25 = qd(5)*C4;
  OMcp3_35 = qd(5)*S4;
  OMcp3_16 = qd(4)+qd(6)*S5;
  OMcp3_26 = OMcp3_25+qd(6)*ROcp3_85;
  OMcp3_36 = OMcp3_35+qd(6)*ROcp3_95;
  OPcp3_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp3_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp3_95-OMcp3_35*S5)-qdd(5)*C4-qdd(6)*ROcp3_85);
  OPcp3_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp3_85-OMcp3_25*S5)+qdd(5)*S4+qdd(6)*ROcp3_95;
  RLcp3_118 = ROcp3_16*s.dpt(1,8);
  RLcp3_218 = ROcp3_26*s.dpt(1,8);
  RLcp3_318 = ROcp3_36*s.dpt(1,8);
  ORcp3_118 = OMcp3_26*RLcp3_318-OMcp3_36*RLcp3_218;
  ORcp3_218 = -(OMcp3_16*RLcp3_318-OMcp3_36*RLcp3_118);
  ORcp3_318 = OMcp3_16*RLcp3_218-OMcp3_26*RLcp3_118;
  PxF3(1) = q(1)+RLcp3_118;
  PxF3(2) = q(2)+RLcp3_218;
  PxF3(3) = q(3)+RLcp3_318;
  RxF3(1,1) = ROcp3_16;
  RxF3(1,2) = ROcp3_26;
  RxF3(1,3) = ROcp3_36;
  RxF3(2,1) = ROcp3_46;
  RxF3(2,2) = ROcp3_56;
  RxF3(2,3) = ROcp3_66;
  RxF3(3,1) = S5;
  RxF3(3,2) = ROcp3_85;
  RxF3(3,3) = ROcp3_95;
  VxF3(1) = qd(1)+ORcp3_118;
  VxF3(2) = qd(2)+ORcp3_218;
  VxF3(3) = qd(3)+ORcp3_318;
  OMxF3(1) = OMcp3_16;
  OMxF3(2) = OMcp3_26;
  OMxF3(3) = OMcp3_36;
  AxF3(1) = qdd(1)+OMcp3_26*ORcp3_318-OMcp3_36*ORcp3_218+OPcp3_26*RLcp3_318-OPcp3_36*RLcp3_218;
  AxF3(2) = qdd(2)-OMcp3_16*ORcp3_318+OMcp3_36*ORcp3_118-OPcp3_16*RLcp3_318+OPcp3_36*RLcp3_118;
  AxF3(3) = qdd(3)+OMcp3_16*ORcp3_218-OMcp3_26*ORcp3_118+OPcp3_16*RLcp3_218-OPcp3_26*RLcp3_118;
  OMPxF3(1) = OPcp3_16;
  OMPxF3(2) = OPcp3_26;
  OMPxF3(3) = OPcp3_36;
 
% Sensor Forces Computation 

  SWr3 = usrfun.fext(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_16*SWr3(1)+ROcp3_26*SWr3(2)+ROcp3_36*SWr3(3);
  xfrc24 = ROcp3_46*SWr3(1)+ROcp3_56*SWr3(2)+ROcp3_66*SWr3(3);
  xfrc34 = ROcp3_85*SWr3(2)+ROcp3_95*SWr3(3)+SWr3(1)*S5;
  s.frc(1,6) = s.frc(1,6)+xfrc14;
  s.frc(2,6) = s.frc(2,6)+xfrc24;
  s.frc(3,6) = s.frc(3,6)+xfrc34;
  xtrq14 = ROcp3_16*SWr3(4)+ROcp3_26*SWr3(5)+ROcp3_36*SWr3(6);
  xtrq24 = ROcp3_46*SWr3(4)+ROcp3_56*SWr3(5)+ROcp3_66*SWr3(6);
  xtrq34 = ROcp3_85*SWr3(5)+ROcp3_95*SWr3(6)+SWr3(4)*S5;
  s.trq(1,6) = s.trq(1,6)+xtrq14-xfrc24*(SWr3(9)-s.l(3,6))+xfrc34*SWr3(8);
  s.trq(2,6) = s.trq(2,6)+xtrq24+xfrc14*(SWr3(9)-s.l(3,6))-xfrc34*SWr3(7);
  s.trq(3,6) = s.trq(3,6)+xtrq34-xfrc14*SWr3(8)+xfrc24*SWr3(7);

% = = Block_0_0_1_4_0_1 = = 
 
% Sensor Kinematics 


  ROcp4_25 = S4*S5;
  ROcp4_35 = -C4*S5;
  ROcp4_85 = -S4*C5;
  ROcp4_95 = C4*C5;
  ROcp4_16 = C5*C6;
  ROcp4_26 = ROcp4_25*C6+C4*S6;
  ROcp4_36 = ROcp4_35*C6+S4*S6;
  ROcp4_46 = -C5*S6;
  ROcp4_56 = -(ROcp4_25*S6-C4*C6);
  ROcp4_66 = -(ROcp4_35*S6-S4*C6);
  OMcp4_25 = qd(5)*C4;
  OMcp4_35 = qd(5)*S4;
  OMcp4_16 = qd(4)+qd(6)*S5;
  OMcp4_26 = OMcp4_25+qd(6)*ROcp4_85;
  OMcp4_36 = OMcp4_35+qd(6)*ROcp4_95;
  OPcp4_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp4_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp4_95-OMcp4_35*S5)-qdd(5)*C4-qdd(6)*ROcp4_85);
  OPcp4_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp4_85-OMcp4_25*S5)+qdd(5)*S4+qdd(6)*ROcp4_95;
  RLcp4_119 = ROcp4_46*s.dpt(2,9);
  RLcp4_219 = ROcp4_56*s.dpt(2,9);
  RLcp4_319 = ROcp4_66*s.dpt(2,9);
  ORcp4_119 = OMcp4_26*RLcp4_319-OMcp4_36*RLcp4_219;
  ORcp4_219 = -(OMcp4_16*RLcp4_319-OMcp4_36*RLcp4_119);
  ORcp4_319 = OMcp4_16*RLcp4_219-OMcp4_26*RLcp4_119;
  PxF4(1) = q(1)+RLcp4_119;
  PxF4(2) = q(2)+RLcp4_219;
  PxF4(3) = q(3)+RLcp4_319;
  RxF4(1,1) = ROcp4_16;
  RxF4(1,2) = ROcp4_26;
  RxF4(1,3) = ROcp4_36;
  RxF4(2,1) = ROcp4_46;
  RxF4(2,2) = ROcp4_56;
  RxF4(2,3) = ROcp4_66;
  RxF4(3,1) = S5;
  RxF4(3,2) = ROcp4_85;
  RxF4(3,3) = ROcp4_95;
  VxF4(1) = qd(1)+ORcp4_119;
  VxF4(2) = qd(2)+ORcp4_219;
  VxF4(3) = qd(3)+ORcp4_319;
  OMxF4(1) = OMcp4_16;
  OMxF4(2) = OMcp4_26;
  OMxF4(3) = OMcp4_36;
  AxF4(1) = qdd(1)+OMcp4_26*ORcp4_319-OMcp4_36*ORcp4_219+OPcp4_26*RLcp4_319-OPcp4_36*RLcp4_219;
  AxF4(2) = qdd(2)-OMcp4_16*ORcp4_319+OMcp4_36*ORcp4_119-OPcp4_16*RLcp4_319+OPcp4_36*RLcp4_119;
  AxF4(3) = qdd(3)+OMcp4_16*ORcp4_219-OMcp4_26*ORcp4_119+OPcp4_16*RLcp4_219-OPcp4_26*RLcp4_119;
  OMPxF4(1) = OPcp4_16;
  OMPxF4(2) = OPcp4_26;
  OMPxF4(3) = OPcp4_36;
 
% Sensor Forces Computation 

  SWr4 = usrfun.fext(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_16*SWr4(1)+ROcp4_26*SWr4(2)+ROcp4_36*SWr4(3);
  xfrc25 = ROcp4_46*SWr4(1)+ROcp4_56*SWr4(2)+ROcp4_66*SWr4(3);
  xfrc35 = ROcp4_85*SWr4(2)+ROcp4_95*SWr4(3)+SWr4(1)*S5;
  s.frc(1,6) = s.frc(1,6)+xfrc15;
  s.frc(2,6) = s.frc(2,6)+xfrc25;
  s.frc(3,6) = s.frc(3,6)+xfrc35;
  xtrq15 = ROcp4_16*SWr4(4)+ROcp4_26*SWr4(5)+ROcp4_36*SWr4(6);
  xtrq25 = ROcp4_46*SWr4(4)+ROcp4_56*SWr4(5)+ROcp4_66*SWr4(6);
  xtrq35 = ROcp4_85*SWr4(5)+ROcp4_95*SWr4(6)+SWr4(4)*S5;
  s.trq(1,6) = s.trq(1,6)+xtrq15-xfrc25*(SWr4(9)-s.l(3,6))+xfrc35*SWr4(8);
  s.trq(2,6) = s.trq(2,6)+xtrq25+xfrc15*(SWr4(9)-s.l(3,6))-xfrc35*SWr4(7);
  s.trq(3,6) = s.trq(3,6)+xtrq35-xfrc15*SWr4(8)+xfrc25*SWr4(7);

% = = Block_0_0_1_5_0_1 = = 
 
% Sensor Kinematics 


  ROcp5_25 = S4*S5;
  ROcp5_35 = -C4*S5;
  ROcp5_85 = -S4*C5;
  ROcp5_95 = C4*C5;
  ROcp5_16 = C5*C6;
  ROcp5_26 = ROcp5_25*C6+C4*S6;
  ROcp5_36 = ROcp5_35*C6+S4*S6;
  ROcp5_46 = -C5*S6;
  ROcp5_56 = -(ROcp5_25*S6-C4*C6);
  ROcp5_66 = -(ROcp5_35*S6-S4*C6);
  OMcp5_25 = qd(5)*C4;
  OMcp5_35 = qd(5)*S4;
  OMcp5_16 = qd(4)+qd(6)*S5;
  OMcp5_26 = OMcp5_25+qd(6)*ROcp5_85;
  OMcp5_36 = OMcp5_35+qd(6)*ROcp5_95;
  OPcp5_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp5_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp5_95-OMcp5_35*S5)-qdd(5)*C4-qdd(6)*ROcp5_85);
  OPcp5_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp5_85-OMcp5_25*S5)+qdd(5)*S4+qdd(6)*ROcp5_95;
  RLcp5_120 = ROcp5_46*s.dpt(2,10);
  RLcp5_220 = ROcp5_56*s.dpt(2,10);
  RLcp5_320 = ROcp5_66*s.dpt(2,10);
  ORcp5_120 = OMcp5_26*RLcp5_320-OMcp5_36*RLcp5_220;
  ORcp5_220 = -(OMcp5_16*RLcp5_320-OMcp5_36*RLcp5_120);
  ORcp5_320 = OMcp5_16*RLcp5_220-OMcp5_26*RLcp5_120;
  PxF5(1) = q(1)+RLcp5_120;
  PxF5(2) = q(2)+RLcp5_220;
  PxF5(3) = q(3)+RLcp5_320;
  RxF5(1,1) = ROcp5_16;
  RxF5(1,2) = ROcp5_26;
  RxF5(1,3) = ROcp5_36;
  RxF5(2,1) = ROcp5_46;
  RxF5(2,2) = ROcp5_56;
  RxF5(2,3) = ROcp5_66;
  RxF5(3,1) = S5;
  RxF5(3,2) = ROcp5_85;
  RxF5(3,3) = ROcp5_95;
  VxF5(1) = qd(1)+ORcp5_120;
  VxF5(2) = qd(2)+ORcp5_220;
  VxF5(3) = qd(3)+ORcp5_320;
  OMxF5(1) = OMcp5_16;
  OMxF5(2) = OMcp5_26;
  OMxF5(3) = OMcp5_36;
  AxF5(1) = qdd(1)+OMcp5_26*ORcp5_320-OMcp5_36*ORcp5_220+OPcp5_26*RLcp5_320-OPcp5_36*RLcp5_220;
  AxF5(2) = qdd(2)-OMcp5_16*ORcp5_320+OMcp5_36*ORcp5_120-OPcp5_16*RLcp5_320+OPcp5_36*RLcp5_120;
  AxF5(3) = qdd(3)+OMcp5_16*ORcp5_220-OMcp5_26*ORcp5_120+OPcp5_16*RLcp5_220-OPcp5_26*RLcp5_120;
  OMPxF5(1) = OPcp5_16;
  OMPxF5(2) = OPcp5_26;
  OMPxF5(3) = OPcp5_36;
 
% Sensor Forces Computation 

  SWr5 = usrfun.fext(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_16*SWr5(1)+ROcp5_26*SWr5(2)+ROcp5_36*SWr5(3);
  xfrc26 = ROcp5_46*SWr5(1)+ROcp5_56*SWr5(2)+ROcp5_66*SWr5(3);
  xfrc36 = ROcp5_85*SWr5(2)+ROcp5_95*SWr5(3)+SWr5(1)*S5;
  frc(1,6) = s.frc(1,6)+xfrc16;
  frc(2,6) = s.frc(2,6)+xfrc26;
  frc(3,6) = s.frc(3,6)+xfrc36;
  xtrq16 = ROcp5_16*SWr5(4)+ROcp5_26*SWr5(5)+ROcp5_36*SWr5(6);
  xtrq26 = ROcp5_46*SWr5(4)+ROcp5_56*SWr5(5)+ROcp5_66*SWr5(6);
  xtrq36 = ROcp5_85*SWr5(5)+ROcp5_95*SWr5(6)+SWr5(4)*S5;
  trq(1,6) = s.trq(1,6)+xtrq16-xfrc26*(SWr5(9)-s.l(3,6))+xfrc36*SWr5(8);
  trq(2,6) = s.trq(2,6)+xtrq26+xfrc16*(SWr5(9)-s.l(3,6))-xfrc36*SWr5(7);
  trq(3,6) = s.trq(3,6)+xtrq36-xfrc16*SWr5(8)+xfrc26*SWr5(7);

% = = Block_0_0_1_6_0_1 = = 
 
% Sensor Kinematics 


  ROcp6_25 = S4*S5;
  ROcp6_35 = -C4*S5;
  ROcp6_85 = -S4*C5;
  ROcp6_95 = C4*C5;
  ROcp6_16 = C5*C6;
  ROcp6_26 = ROcp6_25*C6+C4*S6;
  ROcp6_36 = ROcp6_35*C6+S4*S6;
  ROcp6_46 = -C5*S6;
  ROcp6_56 = -(ROcp6_25*S6-C4*C6);
  ROcp6_66 = -(ROcp6_35*S6-S4*C6);
  OMcp6_25 = qd(5)*C4;
  OMcp6_35 = qd(5)*S4;
  OMcp6_16 = qd(4)+qd(6)*S5;
  OMcp6_26 = OMcp6_25+qd(6)*ROcp6_85;
  OMcp6_36 = OMcp6_35+qd(6)*ROcp6_95;
  OPcp6_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp6_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp6_95-OMcp6_35*S5)-qdd(5)*C4-qdd(6)*ROcp6_85);
  OPcp6_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp6_85-OMcp6_25*S5)+qdd(5)*S4+qdd(6)*ROcp6_95;

% = = Block_0_0_1_6_0_2 = = 
 
% Sensor Kinematics 


  ROcp6_17 = ROcp6_16*C7-S5*S7;
  ROcp6_27 = ROcp6_26*C7-ROcp6_85*S7;
  ROcp6_37 = ROcp6_36*C7-ROcp6_95*S7;
  ROcp6_77 = ROcp6_16*S7+S5*C7;
  ROcp6_87 = ROcp6_26*S7+ROcp6_85*C7;
  ROcp6_97 = ROcp6_36*S7+ROcp6_95*C7;
  ROcp6_48 = ROcp6_46*C8+ROcp6_77*S8;
  ROcp6_58 = ROcp6_56*C8+ROcp6_87*S8;
  ROcp6_68 = ROcp6_66*C8+ROcp6_97*S8;
  ROcp6_78 = -(ROcp6_46*S8-ROcp6_77*C8);
  ROcp6_88 = -(ROcp6_56*S8-ROcp6_87*C8);
  ROcp6_98 = -(ROcp6_66*S8-ROcp6_97*C8);
  RLcp6_17 = ROcp6_16*s.dpt(1,2);
  RLcp6_27 = ROcp6_26*s.dpt(1,2);
  RLcp6_37 = ROcp6_36*s.dpt(1,2);
  OMcp6_17 = OMcp6_16+qd(7)*ROcp6_46;
  OMcp6_27 = OMcp6_26+qd(7)*ROcp6_56;
  OMcp6_37 = OMcp6_36+qd(7)*ROcp6_66;
  ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
  ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
  ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
  OMcp6_18 = OMcp6_17+qd(8)*ROcp6_17;
  OMcp6_28 = OMcp6_27+qd(8)*ROcp6_27;
  OMcp6_38 = OMcp6_37+qd(8)*ROcp6_37;
  OPcp6_18 = OPcp6_16+qd(7)*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56)+qd(8)*(OMcp6_27*ROcp6_37-OMcp6_37*ROcp6_27)+qdd(7)*ROcp6_46+qdd(8)*ROcp6_17;
  OPcp6_28 = OPcp6_26-qd(7)*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46)-qd(8)*(OMcp6_17*ROcp6_37-OMcp6_37*ROcp6_17)+qdd(7)*ROcp6_56+qdd(8)*ROcp6_27;
  OPcp6_38 = OPcp6_36+qd(7)*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46)+qd(8)*(OMcp6_17*ROcp6_27-OMcp6_27*ROcp6_17)+qdd(7)*ROcp6_66+qdd(8)*ROcp6_37;
  RLcp6_121 = ROcp6_78*s.dpt(3,11);
  RLcp6_221 = ROcp6_88*s.dpt(3,11);
  RLcp6_321 = ROcp6_98*s.dpt(3,11);
  ORcp6_121 = OMcp6_28*RLcp6_321-OMcp6_38*RLcp6_221;
  ORcp6_221 = -(OMcp6_18*RLcp6_321-OMcp6_38*RLcp6_121);
  ORcp6_321 = OMcp6_18*RLcp6_221-OMcp6_28*RLcp6_121;
  PxF6(1) = q(1)+RLcp6_121+RLcp6_17;
  PxF6(2) = q(2)+RLcp6_221+RLcp6_27;
  PxF6(3) = q(3)+RLcp6_321+RLcp6_37;
  RxF6(1,1) = ROcp6_17;
  RxF6(1,2) = ROcp6_27;
  RxF6(1,3) = ROcp6_37;
  RxF6(2,1) = ROcp6_48;
  RxF6(2,2) = ROcp6_58;
  RxF6(2,3) = ROcp6_68;
  RxF6(3,1) = ROcp6_78;
  RxF6(3,2) = ROcp6_88;
  RxF6(3,3) = ROcp6_98;
  VxF6(1) = qd(1)+ORcp6_121+ORcp6_17;
  VxF6(2) = qd(2)+ORcp6_221+ORcp6_27;
  VxF6(3) = qd(3)+ORcp6_321+ORcp6_37;
  OMxF6(1) = OMcp6_18;
  OMxF6(2) = OMcp6_28;
  OMxF6(3) = OMcp6_38;
  AxF6(1) = qdd(1)+OMcp6_26*ORcp6_37+OMcp6_28*ORcp6_321-OMcp6_36*ORcp6_27-OMcp6_38*ORcp6_221+OPcp6_26*RLcp6_37+OPcp6_28*RLcp6_321-OPcp6_36*...
 RLcp6_27-OPcp6_38*RLcp6_221;
  AxF6(2) = qdd(2)-OMcp6_16*ORcp6_37-OMcp6_18*ORcp6_321+OMcp6_36*ORcp6_17+OMcp6_38*ORcp6_121-OPcp6_16*RLcp6_37-OPcp6_18*RLcp6_321+OPcp6_36*...
 RLcp6_17+OPcp6_38*RLcp6_121;
  AxF6(3) = qdd(3)+OMcp6_16*ORcp6_27+OMcp6_18*ORcp6_221-OMcp6_26*ORcp6_17-OMcp6_28*ORcp6_121+OPcp6_16*RLcp6_27+OPcp6_18*RLcp6_221-OPcp6_26*...
 RLcp6_17-OPcp6_28*RLcp6_121;
  OMPxF6(1) = OPcp6_18;
  OMPxF6(2) = OPcp6_28;
  OMPxF6(3) = OPcp6_38;
 
% Sensor Forces Computation 

  SWr6 = usrfun.fext(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc17 = ROcp6_17*SWr6(1)+ROcp6_27*SWr6(2)+ROcp6_37*SWr6(3);
  xfrc27 = ROcp6_48*SWr6(1)+ROcp6_58*SWr6(2)+ROcp6_68*SWr6(3);
  xfrc37 = ROcp6_78*SWr6(1)+ROcp6_88*SWr6(2)+ROcp6_98*SWr6(3);
  frc(1,8) = s.frc(1,8)+xfrc17;
  frc(2,8) = s.frc(2,8)+xfrc27;
  frc(3,8) = s.frc(3,8)+xfrc37;
  xtrq17 = ROcp6_17*SWr6(4)+ROcp6_27*SWr6(5)+ROcp6_37*SWr6(6);
  xtrq27 = ROcp6_48*SWr6(4)+ROcp6_58*SWr6(5)+ROcp6_68*SWr6(6);
  xtrq37 = ROcp6_78*SWr6(4)+ROcp6_88*SWr6(5)+ROcp6_98*SWr6(6);
  trq(1,8) = s.trq(1,8)+xtrq17-xfrc27*(SWr6(9)-s.l(3,8))+xfrc37*SWr6(8);
  trq(2,8) = s.trq(2,8)+xtrq27+xfrc17*(SWr6(9)-s.l(3,8))-xfrc37*SWr6(7);
  trq(3,8) = s.trq(3,8)+xtrq37-xfrc17*SWr6(8)+xfrc27*SWr6(7);

% = = Block_0_0_1_7_0_1 = = 
 
% Sensor Kinematics 


  ROcp7_25 = S4*S5;
  ROcp7_35 = -C4*S5;
  ROcp7_85 = -S4*C5;
  ROcp7_95 = C4*C5;
  ROcp7_16 = C5*C6;
  ROcp7_26 = ROcp7_25*C6+C4*S6;
  ROcp7_36 = ROcp7_35*C6+S4*S6;
  ROcp7_46 = -C5*S6;
  ROcp7_56 = -(ROcp7_25*S6-C4*C6);
  ROcp7_66 = -(ROcp7_35*S6-S4*C6);
  OMcp7_25 = qd(5)*C4;
  OMcp7_35 = qd(5)*S4;
  OMcp7_16 = qd(4)+qd(6)*S5;
  OMcp7_26 = OMcp7_25+qd(6)*ROcp7_85;
  OMcp7_36 = OMcp7_35+qd(6)*ROcp7_95;
  OPcp7_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp7_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp7_95-OMcp7_35*S5)-qdd(5)*C4-qdd(6)*ROcp7_85);
  OPcp7_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp7_85-OMcp7_25*S5)+qdd(5)*S4+qdd(6)*ROcp7_95;

% = = Block_0_0_1_7_0_3 = = 
 
% Sensor Kinematics 


  ROcp7_19 = ROcp7_16*C9-S5*S9;
  ROcp7_29 = ROcp7_26*C9-ROcp7_85*S9;
  ROcp7_39 = ROcp7_36*C9-ROcp7_95*S9;
  ROcp7_79 = ROcp7_16*S9+S5*C9;
  ROcp7_89 = ROcp7_26*S9+ROcp7_85*C9;
  ROcp7_99 = ROcp7_36*S9+ROcp7_95*C9;
  ROcp7_410 = ROcp7_46*C10+ROcp7_79*S10;
  ROcp7_510 = ROcp7_56*C10+ROcp7_89*S10;
  ROcp7_610 = ROcp7_66*C10+ROcp7_99*S10;
  ROcp7_710 = -(ROcp7_46*S10-ROcp7_79*C10);
  ROcp7_810 = -(ROcp7_56*S10-ROcp7_89*C10);
  ROcp7_910 = -(ROcp7_66*S10-ROcp7_99*C10);
  RLcp7_19 = ROcp7_46*s.dpt(2,3);
  RLcp7_29 = ROcp7_56*s.dpt(2,3);
  RLcp7_39 = ROcp7_66*s.dpt(2,3);
  OMcp7_19 = OMcp7_16+qd(9)*ROcp7_46;
  OMcp7_29 = OMcp7_26+qd(9)*ROcp7_56;
  OMcp7_39 = OMcp7_36+qd(9)*ROcp7_66;
  ORcp7_19 = OMcp7_26*RLcp7_39-OMcp7_36*RLcp7_29;
  ORcp7_29 = -(OMcp7_16*RLcp7_39-OMcp7_36*RLcp7_19);
  ORcp7_39 = OMcp7_16*RLcp7_29-OMcp7_26*RLcp7_19;
  OMcp7_110 = OMcp7_19+qd(10)*ROcp7_19;
  OMcp7_210 = OMcp7_29+qd(10)*ROcp7_29;
  OMcp7_310 = OMcp7_39+qd(10)*ROcp7_39;
  OPcp7_110 = OPcp7_16+qd(10)*(OMcp7_29*ROcp7_39-OMcp7_39*ROcp7_29)+qd(9)*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56)+qdd(10)*ROcp7_19+qdd(9)*ROcp7_46;
  OPcp7_210 = OPcp7_26-qd(10)*(OMcp7_19*ROcp7_39-OMcp7_39*ROcp7_19)-qd(9)*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46)+qdd(10)*ROcp7_29+qdd(9)*ROcp7_56;
  OPcp7_310 = OPcp7_36+qd(10)*(OMcp7_19*ROcp7_29-OMcp7_29*ROcp7_19)+qd(9)*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46)+qdd(10)*ROcp7_39+qdd(9)*ROcp7_66;
  RLcp7_122 = ROcp7_710*s.dpt(3,12);
  RLcp7_222 = ROcp7_810*s.dpt(3,12);
  RLcp7_322 = ROcp7_910*s.dpt(3,12);
  ORcp7_122 = OMcp7_210*RLcp7_322-OMcp7_310*RLcp7_222;
  ORcp7_222 = -(OMcp7_110*RLcp7_322-OMcp7_310*RLcp7_122);
  ORcp7_322 = OMcp7_110*RLcp7_222-OMcp7_210*RLcp7_122;
  PxF7(1) = q(1)+RLcp7_122+RLcp7_19;
  PxF7(2) = q(2)+RLcp7_222+RLcp7_29;
  PxF7(3) = q(3)+RLcp7_322+RLcp7_39;
  RxF7(1,1) = ROcp7_19;
  RxF7(1,2) = ROcp7_29;
  RxF7(1,3) = ROcp7_39;
  RxF7(2,1) = ROcp7_410;
  RxF7(2,2) = ROcp7_510;
  RxF7(2,3) = ROcp7_610;
  RxF7(3,1) = ROcp7_710;
  RxF7(3,2) = ROcp7_810;
  RxF7(3,3) = ROcp7_910;
  VxF7(1) = qd(1)+ORcp7_122+ORcp7_19;
  VxF7(2) = qd(2)+ORcp7_222+ORcp7_29;
  VxF7(3) = qd(3)+ORcp7_322+ORcp7_39;
  OMxF7(1) = OMcp7_110;
  OMxF7(2) = OMcp7_210;
  OMxF7(3) = OMcp7_310;
  AxF7(1) = qdd(1)+OMcp7_210*ORcp7_322+OMcp7_26*ORcp7_39-OMcp7_310*ORcp7_222-OMcp7_36*ORcp7_29+OPcp7_210*RLcp7_322+OPcp7_26*RLcp7_39-OPcp7_310*...
 RLcp7_222-OPcp7_36*RLcp7_29;
  AxF7(2) = qdd(2)-OMcp7_110*ORcp7_322-OMcp7_16*ORcp7_39+OMcp7_310*ORcp7_122+OMcp7_36*ORcp7_19-OPcp7_110*RLcp7_322-OPcp7_16*RLcp7_39+OPcp7_310*...
 RLcp7_122+OPcp7_36*RLcp7_19;
  AxF7(3) = qdd(3)+OMcp7_110*ORcp7_222+OMcp7_16*ORcp7_29-OMcp7_210*ORcp7_122-OMcp7_26*ORcp7_19+OPcp7_110*RLcp7_222+OPcp7_16*RLcp7_29-OPcp7_210*...
 RLcp7_122-OPcp7_26*RLcp7_19;
  OMPxF7(1) = OPcp7_110;
  OMPxF7(2) = OPcp7_210;
  OMPxF7(3) = OPcp7_310;
 
% Sensor Forces Computation 

  SWr7 = usrfun.fext(PxF7,RxF7,VxF7,OMxF7,AxF7,OMPxF7,s,tsim,7);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc18 = ROcp7_19*SWr7(1)+ROcp7_29*SWr7(2)+ROcp7_39*SWr7(3);
  xfrc28 = ROcp7_410*SWr7(1)+ROcp7_510*SWr7(2)+ROcp7_610*SWr7(3);
  xfrc38 = ROcp7_710*SWr7(1)+ROcp7_810*SWr7(2)+ROcp7_910*SWr7(3);
  frc(1,10) = s.frc(1,10)+xfrc18;
  frc(2,10) = s.frc(2,10)+xfrc28;
  frc(3,10) = s.frc(3,10)+xfrc38;
  xtrq18 = ROcp7_19*SWr7(4)+ROcp7_29*SWr7(5)+ROcp7_39*SWr7(6);
  xtrq28 = ROcp7_410*SWr7(4)+ROcp7_510*SWr7(5)+ROcp7_610*SWr7(6);
  xtrq38 = ROcp7_710*SWr7(4)+ROcp7_810*SWr7(5)+ROcp7_910*SWr7(6);
  trq(1,10) = s.trq(1,10)+xtrq18-xfrc28*(SWr7(9)-s.l(3,10))+xfrc38*SWr7(8);
  trq(2,10) = s.trq(2,10)+xtrq28+xfrc18*(SWr7(9)-s.l(3,10))-xfrc38*SWr7(7);
  trq(3,10) = s.trq(3,10)+xtrq38-xfrc18*SWr7(8)+xfrc28*SWr7(7);

% = = Block_0_0_1_8_0_1 = = 
 
% Sensor Kinematics 


  ROcp8_25 = S4*S5;
  ROcp8_35 = -C4*S5;
  ROcp8_85 = -S4*C5;
  ROcp8_95 = C4*C5;
  ROcp8_16 = C5*C6;
  ROcp8_26 = ROcp8_25*C6+C4*S6;
  ROcp8_36 = ROcp8_35*C6+S4*S6;
  ROcp8_46 = -C5*S6;
  ROcp8_56 = -(ROcp8_25*S6-C4*C6);
  ROcp8_66 = -(ROcp8_35*S6-S4*C6);
  OMcp8_25 = qd(5)*C4;
  OMcp8_35 = qd(5)*S4;
  OMcp8_16 = qd(4)+qd(6)*S5;
  OMcp8_26 = OMcp8_25+qd(6)*ROcp8_85;
  OMcp8_36 = OMcp8_35+qd(6)*ROcp8_95;
  OPcp8_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp8_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp8_95-OMcp8_35*S5)-qdd(5)*C4-qdd(6)*ROcp8_85);
  OPcp8_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp8_85-OMcp8_25*S5)+qdd(5)*S4+qdd(6)*ROcp8_95;

% = = Block_0_0_1_8_0_4 = = 
 
% Sensor Kinematics 


  ROcp8_111 = ROcp8_16*C11-S11*S5;
  ROcp8_211 = ROcp8_26*C11-ROcp8_85*S11;
  ROcp8_311 = ROcp8_36*C11-ROcp8_95*S11;
  ROcp8_711 = ROcp8_16*S11+C11*S5;
  ROcp8_811 = ROcp8_26*S11+ROcp8_85*C11;
  ROcp8_911 = ROcp8_36*S11+ROcp8_95*C11;
  ROcp8_412 = ROcp8_46*C12+ROcp8_711*S12;
  ROcp8_512 = ROcp8_56*C12+ROcp8_811*S12;
  ROcp8_612 = ROcp8_66*C12+ROcp8_911*S12;
  ROcp8_712 = -(ROcp8_46*S12-ROcp8_711*C12);
  ROcp8_812 = -(ROcp8_56*S12-ROcp8_811*C12);
  ROcp8_912 = -(ROcp8_66*S12-ROcp8_911*C12);
  RLcp8_111 = ROcp8_16*s.dpt(1,4);
  RLcp8_211 = ROcp8_26*s.dpt(1,4);
  RLcp8_311 = ROcp8_36*s.dpt(1,4);
  OMcp8_111 = OMcp8_16+qd(11)*ROcp8_46;
  OMcp8_211 = OMcp8_26+qd(11)*ROcp8_56;
  OMcp8_311 = OMcp8_36+qd(11)*ROcp8_66;
  ORcp8_111 = OMcp8_26*RLcp8_311-OMcp8_36*RLcp8_211;
  ORcp8_211 = -(OMcp8_16*RLcp8_311-OMcp8_36*RLcp8_111);
  ORcp8_311 = OMcp8_16*RLcp8_211-OMcp8_26*RLcp8_111;
  OMcp8_112 = OMcp8_111+qd(12)*ROcp8_111;
  OMcp8_212 = OMcp8_211+qd(12)*ROcp8_211;
  OMcp8_312 = OMcp8_311+qd(12)*ROcp8_311;
  OPcp8_112 = OPcp8_16+qd(11)*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56)+qd(12)*(OMcp8_211*ROcp8_311-OMcp8_311*ROcp8_211)+qdd(11)*ROcp8_46+qdd(12)*...
 ROcp8_111;
  OPcp8_212 = OPcp8_26-qd(11)*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46)-qd(12)*(OMcp8_111*ROcp8_311-OMcp8_311*ROcp8_111)+qdd(11)*ROcp8_56+qdd(12)*...
 ROcp8_211;
  OPcp8_312 = OPcp8_36+qd(11)*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46)+qd(12)*(OMcp8_111*ROcp8_211-OMcp8_211*ROcp8_111)+qdd(11)*ROcp8_66+qdd(12)*...
 ROcp8_311;
  RLcp8_123 = ROcp8_712*s.dpt(3,13);
  RLcp8_223 = ROcp8_812*s.dpt(3,13);
  RLcp8_323 = ROcp8_912*s.dpt(3,13);
  ORcp8_123 = OMcp8_212*RLcp8_323-OMcp8_312*RLcp8_223;
  ORcp8_223 = -(OMcp8_112*RLcp8_323-OMcp8_312*RLcp8_123);
  ORcp8_323 = OMcp8_112*RLcp8_223-OMcp8_212*RLcp8_123;
  PxF8(1) = q(1)+RLcp8_111+RLcp8_123;
  PxF8(2) = q(2)+RLcp8_211+RLcp8_223;
  PxF8(3) = q(3)+RLcp8_311+RLcp8_323;
  RxF8(1,1) = ROcp8_111;
  RxF8(1,2) = ROcp8_211;
  RxF8(1,3) = ROcp8_311;
  RxF8(2,1) = ROcp8_412;
  RxF8(2,2) = ROcp8_512;
  RxF8(2,3) = ROcp8_612;
  RxF8(3,1) = ROcp8_712;
  RxF8(3,2) = ROcp8_812;
  RxF8(3,3) = ROcp8_912;
  VxF8(1) = qd(1)+ORcp8_111+ORcp8_123;
  VxF8(2) = qd(2)+ORcp8_211+ORcp8_223;
  VxF8(3) = qd(3)+ORcp8_311+ORcp8_323;
  OMxF8(1) = OMcp8_112;
  OMxF8(2) = OMcp8_212;
  OMxF8(3) = OMcp8_312;
  AxF8(1) = qdd(1)+OMcp8_212*ORcp8_323+OMcp8_26*ORcp8_311-OMcp8_312*ORcp8_223-OMcp8_36*ORcp8_211+OPcp8_212*RLcp8_323+OPcp8_26*RLcp8_311-OPcp8_312...
 *RLcp8_223-OPcp8_36*RLcp8_211;
  AxF8(2) = qdd(2)-OMcp8_112*ORcp8_323-OMcp8_16*ORcp8_311+OMcp8_312*ORcp8_123+OMcp8_36*ORcp8_111-OPcp8_112*RLcp8_323-OPcp8_16*RLcp8_311+OPcp8_312...
 *RLcp8_123+OPcp8_36*RLcp8_111;
  AxF8(3) = qdd(3)+OMcp8_112*ORcp8_223+OMcp8_16*ORcp8_211-OMcp8_212*ORcp8_123-OMcp8_26*ORcp8_111+OPcp8_112*RLcp8_223+OPcp8_16*RLcp8_211-OPcp8_212...
 *RLcp8_123-OPcp8_26*RLcp8_111;
  OMPxF8(1) = OPcp8_112;
  OMPxF8(2) = OPcp8_212;
  OMPxF8(3) = OPcp8_312;
 
% Sensor Forces Computation 

  SWr8 = usrfun.fext(PxF8,RxF8,VxF8,OMxF8,AxF8,OMPxF8,s,tsim,8);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc19 = ROcp8_111*SWr8(1)+ROcp8_211*SWr8(2)+ROcp8_311*SWr8(3);
  xfrc29 = ROcp8_412*SWr8(1)+ROcp8_512*SWr8(2)+ROcp8_612*SWr8(3);
  xfrc39 = ROcp8_712*SWr8(1)+ROcp8_812*SWr8(2)+ROcp8_912*SWr8(3);
  frc(1,12) = s.frc(1,12)+xfrc19;
  frc(2,12) = s.frc(2,12)+xfrc29;
  frc(3,12) = s.frc(3,12)+xfrc39;
  xtrq19 = ROcp8_111*SWr8(4)+ROcp8_211*SWr8(5)+ROcp8_311*SWr8(6);
  xtrq29 = ROcp8_412*SWr8(4)+ROcp8_512*SWr8(5)+ROcp8_612*SWr8(6);
  xtrq39 = ROcp8_712*SWr8(4)+ROcp8_812*SWr8(5)+ROcp8_912*SWr8(6);
  trq(1,12) = s.trq(1,12)+xtrq19-xfrc29*(SWr8(9)-s.l(3,12))+xfrc39*SWr8(8);
  trq(2,12) = s.trq(2,12)+xtrq29+xfrc19*(SWr8(9)-s.l(3,12))-xfrc39*SWr8(7);
  trq(3,12) = s.trq(3,12)+xtrq39-xfrc19*SWr8(8)+xfrc29*SWr8(7);

% = = Block_0_0_1_9_0_1 = = 
 
% Sensor Kinematics 


  ROcp9_25 = S4*S5;
  ROcp9_35 = -C4*S5;
  ROcp9_85 = -S4*C5;
  ROcp9_95 = C4*C5;
  ROcp9_16 = C5*C6;
  ROcp9_26 = ROcp9_25*C6+C4*S6;
  ROcp9_36 = ROcp9_35*C6+S4*S6;
  ROcp9_46 = -C5*S6;
  ROcp9_56 = -(ROcp9_25*S6-C4*C6);
  ROcp9_66 = -(ROcp9_35*S6-S4*C6);
  OMcp9_25 = qd(5)*C4;
  OMcp9_35 = qd(5)*S4;
  OMcp9_16 = qd(4)+qd(6)*S5;
  OMcp9_26 = OMcp9_25+qd(6)*ROcp9_85;
  OMcp9_36 = OMcp9_35+qd(6)*ROcp9_95;
  OPcp9_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp9_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp9_95-OMcp9_35*S5)-qdd(5)*C4-qdd(6)*ROcp9_85);
  OPcp9_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp9_85-OMcp9_25*S5)+qdd(5)*S4+qdd(6)*ROcp9_95;

% = = Block_0_0_1_9_0_5 = = 
 
% Sensor Kinematics 


  ROcp9_113 = ROcp9_16*C13-S13*S5;
  ROcp9_213 = ROcp9_26*C13-ROcp9_85*S13;
  ROcp9_313 = ROcp9_36*C13-ROcp9_95*S13;
  ROcp9_713 = ROcp9_16*S13+C13*S5;
  ROcp9_813 = ROcp9_26*S13+ROcp9_85*C13;
  ROcp9_913 = ROcp9_36*S13+ROcp9_95*C13;
  ROcp9_414 = ROcp9_46*C14+ROcp9_713*S14;
  ROcp9_514 = ROcp9_56*C14+ROcp9_813*S14;
  ROcp9_614 = ROcp9_66*C14+ROcp9_913*S14;
  ROcp9_714 = -(ROcp9_46*S14-ROcp9_713*C14);
  ROcp9_814 = -(ROcp9_56*S14-ROcp9_813*C14);
  ROcp9_914 = -(ROcp9_66*S14-ROcp9_913*C14);
  RLcp9_113 = ROcp9_46*s.dpt(2,5);
  RLcp9_213 = ROcp9_56*s.dpt(2,5);
  RLcp9_313 = ROcp9_66*s.dpt(2,5);
  OMcp9_113 = OMcp9_16+qd(13)*ROcp9_46;
  OMcp9_213 = OMcp9_26+qd(13)*ROcp9_56;
  OMcp9_313 = OMcp9_36+qd(13)*ROcp9_66;
  ORcp9_113 = OMcp9_26*RLcp9_313-OMcp9_36*RLcp9_213;
  ORcp9_213 = -(OMcp9_16*RLcp9_313-OMcp9_36*RLcp9_113);
  ORcp9_313 = OMcp9_16*RLcp9_213-OMcp9_26*RLcp9_113;
  OMcp9_114 = OMcp9_113+qd(14)*ROcp9_113;
  OMcp9_214 = OMcp9_213+qd(14)*ROcp9_213;
  OMcp9_314 = OMcp9_313+qd(14)*ROcp9_313;
  OPcp9_114 = OPcp9_16+qd(13)*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56)+qd(14)*(OMcp9_213*ROcp9_313-OMcp9_313*ROcp9_213)+qdd(13)*ROcp9_46+qdd(14)*...
 ROcp9_113;
  OPcp9_214 = OPcp9_26-qd(13)*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46)-qd(14)*(OMcp9_113*ROcp9_313-OMcp9_313*ROcp9_113)+qdd(13)*ROcp9_56+qdd(14)*...
 ROcp9_213;
  OPcp9_314 = OPcp9_36+qd(13)*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46)+qd(14)*(OMcp9_113*ROcp9_213-OMcp9_213*ROcp9_113)+qdd(13)*ROcp9_66+qdd(14)*...
 ROcp9_313;
  RLcp9_124 = ROcp9_714*s.dpt(3,14);
  RLcp9_224 = ROcp9_814*s.dpt(3,14);
  RLcp9_324 = ROcp9_914*s.dpt(3,14);
  ORcp9_124 = OMcp9_214*RLcp9_324-OMcp9_314*RLcp9_224;
  ORcp9_224 = -(OMcp9_114*RLcp9_324-OMcp9_314*RLcp9_124);
  ORcp9_324 = OMcp9_114*RLcp9_224-OMcp9_214*RLcp9_124;
  PxF9(1) = q(1)+RLcp9_113+RLcp9_124;
  PxF9(2) = q(2)+RLcp9_213+RLcp9_224;
  PxF9(3) = q(3)+RLcp9_313+RLcp9_324;
  RxF9(1,1) = ROcp9_113;
  RxF9(1,2) = ROcp9_213;
  RxF9(1,3) = ROcp9_313;
  RxF9(2,1) = ROcp9_414;
  RxF9(2,2) = ROcp9_514;
  RxF9(2,3) = ROcp9_614;
  RxF9(3,1) = ROcp9_714;
  RxF9(3,2) = ROcp9_814;
  RxF9(3,3) = ROcp9_914;
  VxF9(1) = qd(1)+ORcp9_113+ORcp9_124;
  VxF9(2) = qd(2)+ORcp9_213+ORcp9_224;
  VxF9(3) = qd(3)+ORcp9_313+ORcp9_324;
  OMxF9(1) = OMcp9_114;
  OMxF9(2) = OMcp9_214;
  OMxF9(3) = OMcp9_314;
  AxF9(1) = qdd(1)+OMcp9_214*ORcp9_324+OMcp9_26*ORcp9_313-OMcp9_314*ORcp9_224-OMcp9_36*ORcp9_213+OPcp9_214*RLcp9_324+OPcp9_26*RLcp9_313-OPcp9_314...
 *RLcp9_224-OPcp9_36*RLcp9_213;
  AxF9(2) = qdd(2)-OMcp9_114*ORcp9_324-OMcp9_16*ORcp9_313+OMcp9_314*ORcp9_124+OMcp9_36*ORcp9_113-OPcp9_114*RLcp9_324-OPcp9_16*RLcp9_313+OPcp9_314...
 *RLcp9_124+OPcp9_36*RLcp9_113;
  AxF9(3) = qdd(3)+OMcp9_114*ORcp9_224+OMcp9_16*ORcp9_213-OMcp9_214*ORcp9_124-OMcp9_26*ORcp9_113+OPcp9_114*RLcp9_224+OPcp9_16*RLcp9_213-OPcp9_214...
 *RLcp9_124-OPcp9_26*RLcp9_113;
  OMPxF9(1) = OPcp9_114;
  OMPxF9(2) = OPcp9_214;
  OMPxF9(3) = OPcp9_314;
 
% Sensor Forces Computation 

  SWr9 = usrfun.fext(PxF9,RxF9,VxF9,OMxF9,AxF9,OMPxF9,s,tsim,9);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc110 = ROcp9_113*SWr9(1)+ROcp9_213*SWr9(2)+ROcp9_313*SWr9(3);
  xfrc210 = ROcp9_414*SWr9(1)+ROcp9_514*SWr9(2)+ROcp9_614*SWr9(3);
  xfrc310 = ROcp9_714*SWr9(1)+ROcp9_814*SWr9(2)+ROcp9_914*SWr9(3);
  frc(1,14) = s.frc(1,14)+xfrc110;
  frc(2,14) = s.frc(2,14)+xfrc210;
  frc(3,14) = s.frc(3,14)+xfrc310;
  xtrq110 = ROcp9_113*SWr9(4)+ROcp9_213*SWr9(5)+ROcp9_313*SWr9(6);
  xtrq210 = ROcp9_414*SWr9(4)+ROcp9_514*SWr9(5)+ROcp9_614*SWr9(6);
  xtrq310 = ROcp9_714*SWr9(4)+ROcp9_814*SWr9(5)+ROcp9_914*SWr9(6);
  trq(1,14) = s.trq(1,14)+xtrq110-xfrc210*(SWr9(9)-s.l(3,14))+xfrc310*SWr9(8);
  trq(2,14) = s.trq(2,14)+xtrq210+xfrc110*(SWr9(9)-s.l(3,14))-xfrc310*SWr9(7);
  trq(3,14) = s.trq(3,14)+xtrq310-xfrc110*SWr9(8)+xfrc210*SWr9(7);

% ====== END Task 0 ====== 

  

