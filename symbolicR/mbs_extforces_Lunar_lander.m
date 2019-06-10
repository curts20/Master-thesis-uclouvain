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
%	==> Generation Date : Wed Nov  7 19:28:00 2018
%
%	==> Project name : Lunar_lander
%	==> using XML input file 
%
%	==> Number of joints : 14
%
%	==> Function : F19 : External Forces
%	==> Flops complexity : 1098
%
%	==> Generation Time :  0.010 seconds
%	==> Post-Processing :  0.020 seconds
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


  ROcp0_25 = S4*S5;
  ROcp0_35 = -C4*S5;
  ROcp0_85 = -S4*C5;
  ROcp0_95 = C4*C5;
  ROcp0_16 = C5*C6;
  ROcp0_26 = ROcp0_25*C6+C4*S6;
  ROcp0_36 = ROcp0_35*C6+S4*S6;
  ROcp0_46 = -C5*S6;
  ROcp0_56 = -(ROcp0_25*S6-C4*C6);
  ROcp0_66 = -(ROcp0_35*S6-S4*C6);
  OMcp0_25 = qd(5)*C4;
  OMcp0_35 = qd(5)*S4;
  OMcp0_16 = qd(4)+qd(6)*S5;
  OMcp0_26 = OMcp0_25+qd(6)*ROcp0_85;
  OMcp0_36 = OMcp0_35+qd(6)*ROcp0_95;
  OPcp0_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp0_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp0_95-OMcp0_35*S5)-qdd(5)*C4-qdd(6)*ROcp0_85);
  OPcp0_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp0_85-OMcp0_25*S5)+qdd(5)*S4+qdd(6)*ROcp0_95;

% = = Block_0_0_1_1_0_2 = = 
 
% Sensor Kinematics 


  ROcp0_17 = ROcp0_16*C7-S5*S7;
  ROcp0_27 = ROcp0_26*C7-ROcp0_85*S7;
  ROcp0_37 = ROcp0_36*C7-ROcp0_95*S7;
  ROcp0_77 = ROcp0_16*S7+S5*C7;
  ROcp0_87 = ROcp0_26*S7+ROcp0_85*C7;
  ROcp0_97 = ROcp0_36*S7+ROcp0_95*C7;
  ROcp0_48 = ROcp0_46*C8+ROcp0_77*S8;
  ROcp0_58 = ROcp0_56*C8+ROcp0_87*S8;
  ROcp0_68 = ROcp0_66*C8+ROcp0_97*S8;
  ROcp0_78 = -(ROcp0_46*S8-ROcp0_77*C8);
  ROcp0_88 = -(ROcp0_56*S8-ROcp0_87*C8);
  ROcp0_98 = -(ROcp0_66*S8-ROcp0_97*C8);
  RLcp0_17 = ROcp0_16*s.dpt(1,1);
  RLcp0_27 = ROcp0_26*s.dpt(1,1);
  RLcp0_37 = ROcp0_36*s.dpt(1,1);
  OMcp0_17 = OMcp0_16+qd(7)*ROcp0_46;
  OMcp0_27 = OMcp0_26+qd(7)*ROcp0_56;
  OMcp0_37 = OMcp0_36+qd(7)*ROcp0_66;
  ORcp0_17 = OMcp0_26*RLcp0_37-OMcp0_36*RLcp0_27;
  ORcp0_27 = -(OMcp0_16*RLcp0_37-OMcp0_36*RLcp0_17);
  ORcp0_37 = OMcp0_16*RLcp0_27-OMcp0_26*RLcp0_17;
  OMcp0_18 = OMcp0_17+qd(8)*ROcp0_17;
  OMcp0_28 = OMcp0_27+qd(8)*ROcp0_27;
  OMcp0_38 = OMcp0_37+qd(8)*ROcp0_37;
  OPcp0_18 = OPcp0_16+qd(7)*(OMcp0_26*ROcp0_66-OMcp0_36*ROcp0_56)+qd(8)*(OMcp0_27*ROcp0_37-OMcp0_37*ROcp0_27)+qdd(7)*ROcp0_46+qdd(8)*ROcp0_17;
  OPcp0_28 = OPcp0_26-qd(7)*(OMcp0_16*ROcp0_66-OMcp0_36*ROcp0_46)-qd(8)*(OMcp0_17*ROcp0_37-OMcp0_37*ROcp0_17)+qdd(7)*ROcp0_56+qdd(8)*ROcp0_27;
  OPcp0_38 = OPcp0_36+qd(7)*(OMcp0_16*ROcp0_56-OMcp0_26*ROcp0_46)+qd(8)*(OMcp0_17*ROcp0_27-OMcp0_27*ROcp0_17)+qdd(7)*ROcp0_66+qdd(8)*ROcp0_37;
  RLcp0_131 = ROcp0_78*s.dpt(3,13);
  RLcp0_231 = ROcp0_88*s.dpt(3,13);
  RLcp0_331 = ROcp0_98*s.dpt(3,13);
  ORcp0_131 = OMcp0_28*RLcp0_331-OMcp0_38*RLcp0_231;
  ORcp0_231 = -(OMcp0_18*RLcp0_331-OMcp0_38*RLcp0_131);
  ORcp0_331 = OMcp0_18*RLcp0_231-OMcp0_28*RLcp0_131;
  PxF1(1) = q(1)+RLcp0_131+RLcp0_17;
  PxF1(2) = q(2)+RLcp0_231+RLcp0_27;
  PxF1(3) = q(3)+RLcp0_331+RLcp0_37;
  RxF1(1,1) = ROcp0_17;
  RxF1(1,2) = ROcp0_27;
  RxF1(1,3) = ROcp0_37;
  RxF1(2,1) = ROcp0_48;
  RxF1(2,2) = ROcp0_58;
  RxF1(2,3) = ROcp0_68;
  RxF1(3,1) = ROcp0_78;
  RxF1(3,2) = ROcp0_88;
  RxF1(3,3) = ROcp0_98;
  VxF1(1) = qd(1)+ORcp0_131+ORcp0_17;
  VxF1(2) = qd(2)+ORcp0_231+ORcp0_27;
  VxF1(3) = qd(3)+ORcp0_331+ORcp0_37;
  OMxF1(1) = OMcp0_18;
  OMxF1(2) = OMcp0_28;
  OMxF1(3) = OMcp0_38;
  AxF1(1) = qdd(1)+OMcp0_26*ORcp0_37+OMcp0_28*ORcp0_331-OMcp0_36*ORcp0_27-OMcp0_38*ORcp0_231+OPcp0_26*RLcp0_37+OPcp0_28*RLcp0_331-OPcp0_36*...
 RLcp0_27-OPcp0_38*RLcp0_231;
  AxF1(2) = qdd(2)-OMcp0_16*ORcp0_37-OMcp0_18*ORcp0_331+OMcp0_36*ORcp0_17+OMcp0_38*ORcp0_131-OPcp0_16*RLcp0_37-OPcp0_18*RLcp0_331+OPcp0_36*...
 RLcp0_17+OPcp0_38*RLcp0_131;
  AxF1(3) = qdd(3)+OMcp0_16*ORcp0_27+OMcp0_18*ORcp0_231-OMcp0_26*ORcp0_17-OMcp0_28*ORcp0_131+OPcp0_16*RLcp0_27+OPcp0_18*RLcp0_231-OPcp0_26*...
 RLcp0_17-OPcp0_28*RLcp0_131;
  OMPxF1(1) = OPcp0_18;
  OMPxF1(2) = OPcp0_28;
  OMPxF1(3) = OPcp0_38;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc11 = ROcp0_17*SWr1(1)+ROcp0_27*SWr1(2)+ROcp0_37*SWr1(3);
  xfrc21 = ROcp0_48*SWr1(1)+ROcp0_58*SWr1(2)+ROcp0_68*SWr1(3);
  xfrc31 = ROcp0_78*SWr1(1)+ROcp0_88*SWr1(2)+ROcp0_98*SWr1(3);
  frc(1,8) = s.frc(1,8)+xfrc11;
  frc(2,8) = s.frc(2,8)+xfrc21;
  frc(3,8) = s.frc(3,8)+xfrc31;
  xtrq11 = ROcp0_17*SWr1(4)+ROcp0_27*SWr1(5)+ROcp0_37*SWr1(6);
  xtrq21 = ROcp0_48*SWr1(4)+ROcp0_58*SWr1(5)+ROcp0_68*SWr1(6);
  xtrq31 = ROcp0_78*SWr1(4)+ROcp0_88*SWr1(5)+ROcp0_98*SWr1(6);
  trq(1,8) = s.trq(1,8)+xtrq11-xfrc21*(SWr1(9)-s.l(3,8))+xfrc31*SWr1(8);
  trq(2,8) = s.trq(2,8)+xtrq21+xfrc11*(SWr1(9)-s.l(3,8))-xfrc31*SWr1(7);
  trq(3,8) = s.trq(3,8)+xtrq31-xfrc11*SWr1(8)+xfrc21*SWr1(7);

% = = Block_0_0_1_2_0_1 = = 
 
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
  OMcp1_16 = qd(4)+qd(6)*S5;
  OMcp1_26 = OMcp1_25+qd(6)*ROcp1_85;
  OMcp1_36 = OMcp1_35+qd(6)*ROcp1_95;
  OPcp1_16 = qdd(4)+qd(5)*qd(6)*C5+qdd(6)*S5;
  OPcp1_26 = -(qd(4)*qd(5)*S4+qd(6)*(qd(4)*ROcp1_95-OMcp1_35*S5)-qdd(5)*C4-qdd(6)*ROcp1_85);
  OPcp1_36 = qd(4)*qd(5)*C4+qd(6)*(qd(4)*ROcp1_85-OMcp1_25*S5)+qdd(5)*S4+qdd(6)*ROcp1_95;

% = = Block_0_0_1_2_0_3 = = 
 
% Sensor Kinematics 


  ROcp1_19 = ROcp1_16*C9-S5*S9;
  ROcp1_29 = ROcp1_26*C9-ROcp1_85*S9;
  ROcp1_39 = ROcp1_36*C9-ROcp1_95*S9;
  ROcp1_79 = ROcp1_16*S9+S5*C9;
  ROcp1_89 = ROcp1_26*S9+ROcp1_85*C9;
  ROcp1_99 = ROcp1_36*S9+ROcp1_95*C9;
  ROcp1_410 = ROcp1_46*C10+ROcp1_79*S10;
  ROcp1_510 = ROcp1_56*C10+ROcp1_89*S10;
  ROcp1_610 = ROcp1_66*C10+ROcp1_99*S10;
  ROcp1_710 = -(ROcp1_46*S10-ROcp1_79*C10);
  ROcp1_810 = -(ROcp1_56*S10-ROcp1_89*C10);
  ROcp1_910 = -(ROcp1_66*S10-ROcp1_99*C10);
  RLcp1_19 = ROcp1_46*s.dpt(2,2);
  RLcp1_29 = ROcp1_56*s.dpt(2,2);
  RLcp1_39 = ROcp1_66*s.dpt(2,2);
  OMcp1_19 = OMcp1_16+qd(9)*ROcp1_46;
  OMcp1_29 = OMcp1_26+qd(9)*ROcp1_56;
  OMcp1_39 = OMcp1_36+qd(9)*ROcp1_66;
  ORcp1_19 = OMcp1_26*RLcp1_39-OMcp1_36*RLcp1_29;
  ORcp1_29 = -(OMcp1_16*RLcp1_39-OMcp1_36*RLcp1_19);
  ORcp1_39 = OMcp1_16*RLcp1_29-OMcp1_26*RLcp1_19;
  OMcp1_110 = OMcp1_19+qd(10)*ROcp1_19;
  OMcp1_210 = OMcp1_29+qd(10)*ROcp1_29;
  OMcp1_310 = OMcp1_39+qd(10)*ROcp1_39;
  OPcp1_110 = OPcp1_16+qd(10)*(OMcp1_29*ROcp1_39-OMcp1_39*ROcp1_29)+qd(9)*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56)+qdd(10)*ROcp1_19+qdd(9)*ROcp1_46;
  OPcp1_210 = OPcp1_26-qd(10)*(OMcp1_19*ROcp1_39-OMcp1_39*ROcp1_19)-qd(9)*(OMcp1_16*ROcp1_66-OMcp1_36*ROcp1_46)+qdd(10)*ROcp1_29+qdd(9)*ROcp1_56;
  OPcp1_310 = OPcp1_36+qd(10)*(OMcp1_19*ROcp1_29-OMcp1_29*ROcp1_19)+qd(9)*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46)+qdd(10)*ROcp1_39+qdd(9)*ROcp1_66;
  RLcp1_132 = ROcp1_710*s.dpt(3,15);
  RLcp1_232 = ROcp1_810*s.dpt(3,15);
  RLcp1_332 = ROcp1_910*s.dpt(3,15);
  ORcp1_132 = OMcp1_210*RLcp1_332-OMcp1_310*RLcp1_232;
  ORcp1_232 = -(OMcp1_110*RLcp1_332-OMcp1_310*RLcp1_132);
  ORcp1_332 = OMcp1_110*RLcp1_232-OMcp1_210*RLcp1_132;
  PxF2(1) = q(1)+RLcp1_132+RLcp1_19;
  PxF2(2) = q(2)+RLcp1_232+RLcp1_29;
  PxF2(3) = q(3)+RLcp1_332+RLcp1_39;
  RxF2(1,1) = ROcp1_19;
  RxF2(1,2) = ROcp1_29;
  RxF2(1,3) = ROcp1_39;
  RxF2(2,1) = ROcp1_410;
  RxF2(2,2) = ROcp1_510;
  RxF2(2,3) = ROcp1_610;
  RxF2(3,1) = ROcp1_710;
  RxF2(3,2) = ROcp1_810;
  RxF2(3,3) = ROcp1_910;
  VxF2(1) = qd(1)+ORcp1_132+ORcp1_19;
  VxF2(2) = qd(2)+ORcp1_232+ORcp1_29;
  VxF2(3) = qd(3)+ORcp1_332+ORcp1_39;
  OMxF2(1) = OMcp1_110;
  OMxF2(2) = OMcp1_210;
  OMxF2(3) = OMcp1_310;
  AxF2(1) = qdd(1)+OMcp1_210*ORcp1_332+OMcp1_26*ORcp1_39-OMcp1_310*ORcp1_232-OMcp1_36*ORcp1_29+OPcp1_210*RLcp1_332+OPcp1_26*RLcp1_39-OPcp1_310*...
 RLcp1_232-OPcp1_36*RLcp1_29;
  AxF2(2) = qdd(2)-OMcp1_110*ORcp1_332-OMcp1_16*ORcp1_39+OMcp1_310*ORcp1_132+OMcp1_36*ORcp1_19-OPcp1_110*RLcp1_332-OPcp1_16*RLcp1_39+OPcp1_310*...
 RLcp1_132+OPcp1_36*RLcp1_19;
  AxF2(3) = qdd(3)+OMcp1_110*ORcp1_232+OMcp1_16*ORcp1_29-OMcp1_210*ORcp1_132-OMcp1_26*ORcp1_19+OPcp1_110*RLcp1_232+OPcp1_16*RLcp1_29-OPcp1_210*...
 RLcp1_132-OPcp1_26*RLcp1_19;
  OMPxF2(1) = OPcp1_110;
  OMPxF2(2) = OPcp1_210;
  OMPxF2(3) = OPcp1_310;
 
% Sensor Forces Computation 

  SWr2 = usrfun.fext(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc12 = ROcp1_19*SWr2(1)+ROcp1_29*SWr2(2)+ROcp1_39*SWr2(3);
  xfrc22 = ROcp1_410*SWr2(1)+ROcp1_510*SWr2(2)+ROcp1_610*SWr2(3);
  xfrc32 = ROcp1_710*SWr2(1)+ROcp1_810*SWr2(2)+ROcp1_910*SWr2(3);
  frc(1,10) = s.frc(1,10)+xfrc12;
  frc(2,10) = s.frc(2,10)+xfrc22;
  frc(3,10) = s.frc(3,10)+xfrc32;
  xtrq12 = ROcp1_19*SWr2(4)+ROcp1_29*SWr2(5)+ROcp1_39*SWr2(6);
  xtrq22 = ROcp1_410*SWr2(4)+ROcp1_510*SWr2(5)+ROcp1_610*SWr2(6);
  xtrq32 = ROcp1_710*SWr2(4)+ROcp1_810*SWr2(5)+ROcp1_910*SWr2(6);
  trq(1,10) = s.trq(1,10)+xtrq12-xfrc22*(SWr2(9)-s.l(3,10))+xfrc32*SWr2(8);
  trq(2,10) = s.trq(2,10)+xtrq22+xfrc12*(SWr2(9)-s.l(3,10))-xfrc32*SWr2(7);
  trq(3,10) = s.trq(3,10)+xtrq32-xfrc12*SWr2(8)+xfrc22*SWr2(7);

% = = Block_0_0_1_3_0_1 = = 
 
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

% = = Block_0_0_1_3_0_4 = = 
 
% Sensor Kinematics 


  ROcp2_111 = ROcp2_16*C11-S11*S5;
  ROcp2_211 = ROcp2_26*C11-ROcp2_85*S11;
  ROcp2_311 = ROcp2_36*C11-ROcp2_95*S11;
  ROcp2_711 = ROcp2_16*S11+C11*S5;
  ROcp2_811 = ROcp2_26*S11+ROcp2_85*C11;
  ROcp2_911 = ROcp2_36*S11+ROcp2_95*C11;
  ROcp2_412 = ROcp2_46*C12+ROcp2_711*S12;
  ROcp2_512 = ROcp2_56*C12+ROcp2_811*S12;
  ROcp2_612 = ROcp2_66*C12+ROcp2_911*S12;
  ROcp2_712 = -(ROcp2_46*S12-ROcp2_711*C12);
  ROcp2_812 = -(ROcp2_56*S12-ROcp2_811*C12);
  ROcp2_912 = -(ROcp2_66*S12-ROcp2_911*C12);
  RLcp2_111 = ROcp2_16*s.dpt(1,3);
  RLcp2_211 = ROcp2_26*s.dpt(1,3);
  RLcp2_311 = ROcp2_36*s.dpt(1,3);
  OMcp2_111 = OMcp2_16+qd(11)*ROcp2_46;
  OMcp2_211 = OMcp2_26+qd(11)*ROcp2_56;
  OMcp2_311 = OMcp2_36+qd(11)*ROcp2_66;
  ORcp2_111 = OMcp2_26*RLcp2_311-OMcp2_36*RLcp2_211;
  ORcp2_211 = -(OMcp2_16*RLcp2_311-OMcp2_36*RLcp2_111);
  ORcp2_311 = OMcp2_16*RLcp2_211-OMcp2_26*RLcp2_111;
  OMcp2_112 = OMcp2_111+qd(12)*ROcp2_111;
  OMcp2_212 = OMcp2_211+qd(12)*ROcp2_211;
  OMcp2_312 = OMcp2_311+qd(12)*ROcp2_311;
  OPcp2_112 = OPcp2_16+qd(11)*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)+qd(12)*(OMcp2_211*ROcp2_311-OMcp2_311*ROcp2_211)+qdd(11)*ROcp2_46+qdd(12)*...
 ROcp2_111;
  OPcp2_212 = OPcp2_26-qd(11)*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46)-qd(12)*(OMcp2_111*ROcp2_311-OMcp2_311*ROcp2_111)+qdd(11)*ROcp2_56+qdd(12)*...
 ROcp2_211;
  OPcp2_312 = OPcp2_36+qd(11)*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)+qd(12)*(OMcp2_111*ROcp2_211-OMcp2_211*ROcp2_111)+qdd(11)*ROcp2_66+qdd(12)*...
 ROcp2_311;
  RLcp2_133 = ROcp2_712*s.dpt(3,17);
  RLcp2_233 = ROcp2_812*s.dpt(3,17);
  RLcp2_333 = ROcp2_912*s.dpt(3,17);
  ORcp2_133 = OMcp2_212*RLcp2_333-OMcp2_312*RLcp2_233;
  ORcp2_233 = -(OMcp2_112*RLcp2_333-OMcp2_312*RLcp2_133);
  ORcp2_333 = OMcp2_112*RLcp2_233-OMcp2_212*RLcp2_133;
  PxF3(1) = q(1)+RLcp2_111+RLcp2_133;
  PxF3(2) = q(2)+RLcp2_211+RLcp2_233;
  PxF3(3) = q(3)+RLcp2_311+RLcp2_333;
  RxF3(1,1) = ROcp2_111;
  RxF3(1,2) = ROcp2_211;
  RxF3(1,3) = ROcp2_311;
  RxF3(2,1) = ROcp2_412;
  RxF3(2,2) = ROcp2_512;
  RxF3(2,3) = ROcp2_612;
  RxF3(3,1) = ROcp2_712;
  RxF3(3,2) = ROcp2_812;
  RxF3(3,3) = ROcp2_912;
  VxF3(1) = qd(1)+ORcp2_111+ORcp2_133;
  VxF3(2) = qd(2)+ORcp2_211+ORcp2_233;
  VxF3(3) = qd(3)+ORcp2_311+ORcp2_333;
  OMxF3(1) = OMcp2_112;
  OMxF3(2) = OMcp2_212;
  OMxF3(3) = OMcp2_312;
  AxF3(1) = qdd(1)+OMcp2_212*ORcp2_333+OMcp2_26*ORcp2_311-OMcp2_312*ORcp2_233-OMcp2_36*ORcp2_211+OPcp2_212*RLcp2_333+OPcp2_26*RLcp2_311-OPcp2_312...
 *RLcp2_233-OPcp2_36*RLcp2_211;
  AxF3(2) = qdd(2)-OMcp2_112*ORcp2_333-OMcp2_16*ORcp2_311+OMcp2_312*ORcp2_133+OMcp2_36*ORcp2_111-OPcp2_112*RLcp2_333-OPcp2_16*RLcp2_311+OPcp2_312...
 *RLcp2_133+OPcp2_36*RLcp2_111;
  AxF3(3) = qdd(3)+OMcp2_112*ORcp2_233+OMcp2_16*ORcp2_211-OMcp2_212*ORcp2_133-OMcp2_26*ORcp2_111+OPcp2_112*RLcp2_233+OPcp2_16*RLcp2_211-OPcp2_212...
 *RLcp2_133-OPcp2_26*RLcp2_111;
  OMPxF3(1) = OPcp2_112;
  OMPxF3(2) = OPcp2_212;
  OMPxF3(3) = OPcp2_312;
 
% Sensor Forces Computation 

  SWr3 = usrfun.fext(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc13 = ROcp2_111*SWr3(1)+ROcp2_211*SWr3(2)+ROcp2_311*SWr3(3);
  xfrc23 = ROcp2_412*SWr3(1)+ROcp2_512*SWr3(2)+ROcp2_612*SWr3(3);
  xfrc33 = ROcp2_712*SWr3(1)+ROcp2_812*SWr3(2)+ROcp2_912*SWr3(3);
  frc(1,12) = s.frc(1,12)+xfrc13;
  frc(2,12) = s.frc(2,12)+xfrc23;
  frc(3,12) = s.frc(3,12)+xfrc33;
  xtrq13 = ROcp2_111*SWr3(4)+ROcp2_211*SWr3(5)+ROcp2_311*SWr3(6);
  xtrq23 = ROcp2_412*SWr3(4)+ROcp2_512*SWr3(5)+ROcp2_612*SWr3(6);
  xtrq33 = ROcp2_712*SWr3(4)+ROcp2_812*SWr3(5)+ROcp2_912*SWr3(6);
  trq(1,12) = s.trq(1,12)+xtrq13-xfrc23*(SWr3(9)-s.l(3,12))+xfrc33*SWr3(8);
  trq(2,12) = s.trq(2,12)+xtrq23+xfrc13*(SWr3(9)-s.l(3,12))-xfrc33*SWr3(7);
  trq(3,12) = s.trq(3,12)+xtrq33-xfrc13*SWr3(8)+xfrc23*SWr3(7);

% = = Block_0_0_1_4_0_1 = = 
 
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

% = = Block_0_0_1_4_0_5 = = 
 
% Sensor Kinematics 


  ROcp3_113 = ROcp3_16*C13-S13*S5;
  ROcp3_213 = ROcp3_26*C13-ROcp3_85*S13;
  ROcp3_313 = ROcp3_36*C13-ROcp3_95*S13;
  ROcp3_713 = ROcp3_16*S13+C13*S5;
  ROcp3_813 = ROcp3_26*S13+ROcp3_85*C13;
  ROcp3_913 = ROcp3_36*S13+ROcp3_95*C13;
  ROcp3_414 = ROcp3_46*C14+ROcp3_713*S14;
  ROcp3_514 = ROcp3_56*C14+ROcp3_813*S14;
  ROcp3_614 = ROcp3_66*C14+ROcp3_913*S14;
  ROcp3_714 = -(ROcp3_46*S14-ROcp3_713*C14);
  ROcp3_814 = -(ROcp3_56*S14-ROcp3_813*C14);
  ROcp3_914 = -(ROcp3_66*S14-ROcp3_913*C14);
  RLcp3_113 = ROcp3_46*s.dpt(2,4);
  RLcp3_213 = ROcp3_56*s.dpt(2,4);
  RLcp3_313 = ROcp3_66*s.dpt(2,4);
  OMcp3_113 = OMcp3_16+qd(13)*ROcp3_46;
  OMcp3_213 = OMcp3_26+qd(13)*ROcp3_56;
  OMcp3_313 = OMcp3_36+qd(13)*ROcp3_66;
  ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
  ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
  ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
  OMcp3_114 = OMcp3_113+qd(14)*ROcp3_113;
  OMcp3_214 = OMcp3_213+qd(14)*ROcp3_213;
  OMcp3_314 = OMcp3_313+qd(14)*ROcp3_313;
  OPcp3_114 = OPcp3_16+qd(13)*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd(14)*(OMcp3_213*ROcp3_313-OMcp3_313*ROcp3_213)+qdd(13)*ROcp3_46+qdd(14)*...
 ROcp3_113;
  OPcp3_214 = OPcp3_26-qd(13)*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd(14)*(OMcp3_113*ROcp3_313-OMcp3_313*ROcp3_113)+qdd(13)*ROcp3_56+qdd(14)*...
 ROcp3_213;
  OPcp3_314 = OPcp3_36+qd(13)*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd(14)*(OMcp3_113*ROcp3_213-OMcp3_213*ROcp3_113)+qdd(13)*ROcp3_66+qdd(14)*...
 ROcp3_313;
  RLcp3_134 = ROcp3_714*s.dpt(3,19);
  RLcp3_234 = ROcp3_814*s.dpt(3,19);
  RLcp3_334 = ROcp3_914*s.dpt(3,19);
  ORcp3_134 = OMcp3_214*RLcp3_334-OMcp3_314*RLcp3_234;
  ORcp3_234 = -(OMcp3_114*RLcp3_334-OMcp3_314*RLcp3_134);
  ORcp3_334 = OMcp3_114*RLcp3_234-OMcp3_214*RLcp3_134;
  PxF4(1) = q(1)+RLcp3_113+RLcp3_134;
  PxF4(2) = q(2)+RLcp3_213+RLcp3_234;
  PxF4(3) = q(3)+RLcp3_313+RLcp3_334;
  RxF4(1,1) = ROcp3_113;
  RxF4(1,2) = ROcp3_213;
  RxF4(1,3) = ROcp3_313;
  RxF4(2,1) = ROcp3_414;
  RxF4(2,2) = ROcp3_514;
  RxF4(2,3) = ROcp3_614;
  RxF4(3,1) = ROcp3_714;
  RxF4(3,2) = ROcp3_814;
  RxF4(3,3) = ROcp3_914;
  VxF4(1) = qd(1)+ORcp3_113+ORcp3_134;
  VxF4(2) = qd(2)+ORcp3_213+ORcp3_234;
  VxF4(3) = qd(3)+ORcp3_313+ORcp3_334;
  OMxF4(1) = OMcp3_114;
  OMxF4(2) = OMcp3_214;
  OMxF4(3) = OMcp3_314;
  AxF4(1) = qdd(1)+OMcp3_214*ORcp3_334+OMcp3_26*ORcp3_313-OMcp3_314*ORcp3_234-OMcp3_36*ORcp3_213+OPcp3_214*RLcp3_334+OPcp3_26*RLcp3_313-OPcp3_314...
 *RLcp3_234-OPcp3_36*RLcp3_213;
  AxF4(2) = qdd(2)-OMcp3_114*ORcp3_334-OMcp3_16*ORcp3_313+OMcp3_314*ORcp3_134+OMcp3_36*ORcp3_113-OPcp3_114*RLcp3_334-OPcp3_16*RLcp3_313+OPcp3_314...
 *RLcp3_134+OPcp3_36*RLcp3_113;
  AxF4(3) = qdd(3)+OMcp3_114*ORcp3_234+OMcp3_16*ORcp3_213-OMcp3_214*ORcp3_134-OMcp3_26*ORcp3_113+OPcp3_114*RLcp3_234+OPcp3_16*RLcp3_213-OPcp3_214...
 *RLcp3_134-OPcp3_26*RLcp3_113;
  OMPxF4(1) = OPcp3_114;
  OMPxF4(2) = OPcp3_214;
  OMPxF4(3) = OPcp3_314;
 
% Sensor Forces Computation 

  SWr4 = usrfun.fext(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_113*SWr4(1)+ROcp3_213*SWr4(2)+ROcp3_313*SWr4(3);
  xfrc24 = ROcp3_414*SWr4(1)+ROcp3_514*SWr4(2)+ROcp3_614*SWr4(3);
  xfrc34 = ROcp3_714*SWr4(1)+ROcp3_814*SWr4(2)+ROcp3_914*SWr4(3);
  frc(1,14) = s.frc(1,14)+xfrc14;
  frc(2,14) = s.frc(2,14)+xfrc24;
  frc(3,14) = s.frc(3,14)+xfrc34;
  xtrq14 = ROcp3_113*SWr4(4)+ROcp3_213*SWr4(5)+ROcp3_313*SWr4(6);
  xtrq24 = ROcp3_414*SWr4(4)+ROcp3_514*SWr4(5)+ROcp3_614*SWr4(6);
  xtrq34 = ROcp3_714*SWr4(4)+ROcp3_814*SWr4(5)+ROcp3_914*SWr4(6);
  trq(1,14) = s.trq(1,14)+xtrq14-xfrc24*(SWr4(9)-s.l(3,14))+xfrc34*SWr4(8);
  trq(2,14) = s.trq(2,14)+xtrq24+xfrc14*(SWr4(9)-s.l(3,14))-xfrc34*SWr4(7);
  trq(3,14) = s.trq(3,14)+xtrq34-xfrc14*SWr4(8)+xfrc24*SWr4(7);

% = = Block_0_0_1_4_1_0 = = 
 
% Symbolic Outputs  

  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);

% ====== END Task 0 ====== 

  

