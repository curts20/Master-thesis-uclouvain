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
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 1537
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.030 seconds
%
%-------------------------------------------------------------
%
function [sens] = sensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,14);

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

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_0_1 = = 
 
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
    OMcp0_26 = OMcp0_25+ROcp0_85*qd(6);
    OMcp0_36 = OMcp0_35+ROcp0_95*qd(6);
    OPcp0_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp0_26 = ROcp0_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp0_35*S5-ROcp0_95*qd(4));
    OPcp0_36 = ROcp0_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp0_25*S5-ROcp0_85*qd(4));
    RLcp0_115 = s.dpt(3,6)*S5;
    RLcp0_215 = ROcp0_85*s.dpt(3,6);
    RLcp0_315 = ROcp0_95*s.dpt(3,6);
    POcp0_115 = RLcp0_115+q(1);
    POcp0_215 = RLcp0_215+q(2);
    POcp0_315 = RLcp0_315+q(3);
    JTcp0_115_5 = s.dpt(3,6)*C5;
    JTcp0_215_5 = RLcp0_115*S4;
    JTcp0_315_5 = -RLcp0_115*C4;
    ORcp0_115 = OMcp0_26*RLcp0_315-OMcp0_36*RLcp0_215;
    ORcp0_215 = -(OMcp0_16*RLcp0_315-OMcp0_36*RLcp0_115);
    ORcp0_315 = OMcp0_16*RLcp0_215-OMcp0_26*RLcp0_115;
    VIcp0_115 = ORcp0_115+qd(1);
    VIcp0_215 = ORcp0_215+qd(2);
    VIcp0_315 = ORcp0_315+qd(3);
    ACcp0_115 = qdd(1)+OMcp0_26*ORcp0_315-OMcp0_36*ORcp0_215+OPcp0_26*RLcp0_315-OPcp0_36*RLcp0_215;
    ACcp0_215 = qdd(2)-OMcp0_16*ORcp0_315+OMcp0_36*ORcp0_115-OPcp0_16*RLcp0_315+OPcp0_36*RLcp0_115;
    ACcp0_315 = qdd(3)+OMcp0_16*ORcp0_215-OMcp0_26*ORcp0_115+OPcp0_16*RLcp0_215-OPcp0_26*RLcp0_115;

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp0_115;
    sens.P(2) = POcp0_215;
    sens.P(3) = POcp0_315;
    sens.R(1,1) = ROcp0_16;
    sens.R(1,2) = ROcp0_26;
    sens.R(1,3) = ROcp0_36;
    sens.R(2,1) = ROcp0_46;
    sens.R(2,2) = ROcp0_56;
    sens.R(2,3) = ROcp0_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp0_85;
    sens.R(3,3) = ROcp0_95;
    sens.V(1) = VIcp0_115;
    sens.V(2) = VIcp0_215;
    sens.V(3) = VIcp0_315;
    sens.OM(1) = OMcp0_16;
    sens.OM(2) = OMcp0_26;
    sens.OM(3) = OMcp0_36;
    sens.J(1,1) = (1.0);
    sens.J(1,5) = JTcp0_115_5;
    sens.J(2,2) = (1.0);
    sens.J(2,4) = -RLcp0_315;
    sens.J(2,5) = JTcp0_215_5;
    sens.J(3,3) = (1.0);
    sens.J(3,4) = RLcp0_215;
    sens.J(3,5) = JTcp0_315_5;
    sens.J(4,4) = (1.0);
    sens.J(4,6) = S5;
    sens.J(5,5) = C4;
    sens.J(5,6) = ROcp0_85;
    sens.J(6,5) = S4;
    sens.J(6,6) = ROcp0_95;
    sens.A(1) = ACcp0_115;
    sens.A(2) = ACcp0_215;
    sens.A(3) = ACcp0_315;
    sens.OMP(1) = OPcp0_16;
    sens.OMP(2) = OPcp0_26;
    sens.OMP(3) = OPcp0_36;
 
% 
case 2, 


% = = Block_1_0_0_2_0_1 = = 
 
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
    OMcp1_26 = OMcp1_25+ROcp1_85*qd(6);
    OMcp1_36 = OMcp1_35+ROcp1_95*qd(6);
    OPcp1_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp1_26 = ROcp1_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp1_35*S5-ROcp1_95*qd(4));
    OPcp1_36 = ROcp1_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp1_25*S5-ROcp1_85*qd(4));

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = q(1);
    sens.P(2) = q(2);
    sens.P(3) = q(3);
    sens.R(1,1) = ROcp1_16;
    sens.R(1,2) = ROcp1_26;
    sens.R(1,3) = ROcp1_36;
    sens.R(2,1) = ROcp1_46;
    sens.R(2,2) = ROcp1_56;
    sens.R(2,3) = ROcp1_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp1_85;
    sens.R(3,3) = ROcp1_95;
    sens.V(1) = qd(1);
    sens.V(2) = qd(2);
    sens.V(3) = qd(3);
    sens.OM(1) = OMcp1_16;
    sens.OM(2) = OMcp1_26;
    sens.OM(3) = OMcp1_36;
    sens.A(1) = qdd(1);
    sens.A(2) = qdd(2);
    sens.A(3) = qdd(3);
    sens.OMP(1) = OPcp1_16;
    sens.OMP(2) = OPcp1_26;
    sens.OMP(3) = OPcp1_36;
 
% 
case 3, 


% = = Block_1_0_0_3_0_1 = = 
 
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
    OMcp2_26 = OMcp2_25+ROcp2_85*qd(6);
    OMcp2_36 = OMcp2_35+ROcp2_95*qd(6);
    OPcp2_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp2_26 = ROcp2_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp2_35*S5-ROcp2_95*qd(4));
    OPcp2_36 = ROcp2_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp2_25*S5-ROcp2_85*qd(4));
    RLcp2_117 = ROcp2_16*s.dpt(1,7);
    RLcp2_217 = ROcp2_26*s.dpt(1,7);
    RLcp2_317 = ROcp2_36*s.dpt(1,7);
    POcp2_117 = RLcp2_117+q(1);
    POcp2_217 = RLcp2_217+q(2);
    POcp2_317 = RLcp2_317+q(3);
    ORcp2_117 = OMcp2_26*RLcp2_317-OMcp2_36*RLcp2_217;
    ORcp2_217 = -(OMcp2_16*RLcp2_317-OMcp2_36*RLcp2_117);
    ORcp2_317 = OMcp2_16*RLcp2_217-OMcp2_26*RLcp2_117;
    VIcp2_117 = ORcp2_117+qd(1);
    VIcp2_217 = ORcp2_217+qd(2);
    VIcp2_317 = ORcp2_317+qd(3);
    ACcp2_117 = qdd(1)+OMcp2_26*ORcp2_317-OMcp2_36*ORcp2_217+OPcp2_26*RLcp2_317-OPcp2_36*RLcp2_217;
    ACcp2_217 = qdd(2)-OMcp2_16*ORcp2_317+OMcp2_36*ORcp2_117-OPcp2_16*RLcp2_317+OPcp2_36*RLcp2_117;
    ACcp2_317 = qdd(3)+OMcp2_16*ORcp2_217-OMcp2_26*ORcp2_117+OPcp2_16*RLcp2_217-OPcp2_26*RLcp2_117;

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp2_117;
    sens.P(2) = POcp2_217;
    sens.P(3) = POcp2_317;
    sens.R(1,1) = ROcp2_16;
    sens.R(1,2) = ROcp2_26;
    sens.R(1,3) = ROcp2_36;
    sens.R(2,1) = ROcp2_46;
    sens.R(2,2) = ROcp2_56;
    sens.R(2,3) = ROcp2_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp2_85;
    sens.R(3,3) = ROcp2_95;
    sens.V(1) = VIcp2_117;
    sens.V(2) = VIcp2_217;
    sens.V(3) = VIcp2_317;
    sens.OM(1) = OMcp2_16;
    sens.OM(2) = OMcp2_26;
    sens.OM(3) = OMcp2_36;
    sens.A(1) = ACcp2_117;
    sens.A(2) = ACcp2_217;
    sens.A(3) = ACcp2_317;
    sens.OMP(1) = OPcp2_16;
    sens.OMP(2) = OPcp2_26;
    sens.OMP(3) = OPcp2_36;
 
% 
case 4, 


% = = Block_1_0_0_4_0_1 = = 
 
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
    OMcp3_26 = OMcp3_25+ROcp3_85*qd(6);
    OMcp3_36 = OMcp3_35+ROcp3_95*qd(6);
    OPcp3_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp3_26 = ROcp3_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp3_35*S5-ROcp3_95*qd(4));
    OPcp3_36 = ROcp3_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp3_25*S5-ROcp3_85*qd(4));
    RLcp3_118 = ROcp3_16*s.dpt(1,8);
    RLcp3_218 = ROcp3_26*s.dpt(1,8);
    RLcp3_318 = ROcp3_36*s.dpt(1,8);
    POcp3_118 = RLcp3_118+q(1);
    POcp3_218 = RLcp3_218+q(2);
    POcp3_318 = RLcp3_318+q(3);
    ORcp3_118 = OMcp3_26*RLcp3_318-OMcp3_36*RLcp3_218;
    ORcp3_218 = -(OMcp3_16*RLcp3_318-OMcp3_36*RLcp3_118);
    ORcp3_318 = OMcp3_16*RLcp3_218-OMcp3_26*RLcp3_118;
    VIcp3_118 = ORcp3_118+qd(1);
    VIcp3_218 = ORcp3_218+qd(2);
    VIcp3_318 = ORcp3_318+qd(3);
    ACcp3_118 = qdd(1)+OMcp3_26*ORcp3_318-OMcp3_36*ORcp3_218+OPcp3_26*RLcp3_318-OPcp3_36*RLcp3_218;
    ACcp3_218 = qdd(2)-OMcp3_16*ORcp3_318+OMcp3_36*ORcp3_118-OPcp3_16*RLcp3_318+OPcp3_36*RLcp3_118;
    ACcp3_318 = qdd(3)+OMcp3_16*ORcp3_218-OMcp3_26*ORcp3_118+OPcp3_16*RLcp3_218-OPcp3_26*RLcp3_118;

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp3_118;
    sens.P(2) = POcp3_218;
    sens.P(3) = POcp3_318;
    sens.R(1,1) = ROcp3_16;
    sens.R(1,2) = ROcp3_26;
    sens.R(1,3) = ROcp3_36;
    sens.R(2,1) = ROcp3_46;
    sens.R(2,2) = ROcp3_56;
    sens.R(2,3) = ROcp3_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp3_85;
    sens.R(3,3) = ROcp3_95;
    sens.V(1) = VIcp3_118;
    sens.V(2) = VIcp3_218;
    sens.V(3) = VIcp3_318;
    sens.OM(1) = OMcp3_16;
    sens.OM(2) = OMcp3_26;
    sens.OM(3) = OMcp3_36;
    sens.A(1) = ACcp3_118;
    sens.A(2) = ACcp3_218;
    sens.A(3) = ACcp3_318;
    sens.OMP(1) = OPcp3_16;
    sens.OMP(2) = OPcp3_26;
    sens.OMP(3) = OPcp3_36;
 
% 
case 5, 


% = = Block_1_0_0_5_0_1 = = 
 
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
    OMcp4_26 = OMcp4_25+ROcp4_85*qd(6);
    OMcp4_36 = OMcp4_35+ROcp4_95*qd(6);
    OPcp4_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp4_26 = ROcp4_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp4_35*S5-ROcp4_95*qd(4));
    OPcp4_36 = ROcp4_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp4_25*S5-ROcp4_85*qd(4));
    RLcp4_119 = ROcp4_46*s.dpt(2,9);
    RLcp4_219 = ROcp4_56*s.dpt(2,9);
    RLcp4_319 = ROcp4_66*s.dpt(2,9);
    POcp4_119 = RLcp4_119+q(1);
    POcp4_219 = RLcp4_219+q(2);
    POcp4_319 = RLcp4_319+q(3);
    ORcp4_119 = OMcp4_26*RLcp4_319-OMcp4_36*RLcp4_219;
    ORcp4_219 = -(OMcp4_16*RLcp4_319-OMcp4_36*RLcp4_119);
    ORcp4_319 = OMcp4_16*RLcp4_219-OMcp4_26*RLcp4_119;
    VIcp4_119 = ORcp4_119+qd(1);
    VIcp4_219 = ORcp4_219+qd(2);
    VIcp4_319 = ORcp4_319+qd(3);
    ACcp4_119 = qdd(1)+OMcp4_26*ORcp4_319-OMcp4_36*ORcp4_219+OPcp4_26*RLcp4_319-OPcp4_36*RLcp4_219;
    ACcp4_219 = qdd(2)-OMcp4_16*ORcp4_319+OMcp4_36*ORcp4_119-OPcp4_16*RLcp4_319+OPcp4_36*RLcp4_119;
    ACcp4_319 = qdd(3)+OMcp4_16*ORcp4_219-OMcp4_26*ORcp4_119+OPcp4_16*RLcp4_219-OPcp4_26*RLcp4_119;

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp4_119;
    sens.P(2) = POcp4_219;
    sens.P(3) = POcp4_319;
    sens.R(1,1) = ROcp4_16;
    sens.R(1,2) = ROcp4_26;
    sens.R(1,3) = ROcp4_36;
    sens.R(2,1) = ROcp4_46;
    sens.R(2,2) = ROcp4_56;
    sens.R(2,3) = ROcp4_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp4_85;
    sens.R(3,3) = ROcp4_95;
    sens.V(1) = VIcp4_119;
    sens.V(2) = VIcp4_219;
    sens.V(3) = VIcp4_319;
    sens.OM(1) = OMcp4_16;
    sens.OM(2) = OMcp4_26;
    sens.OM(3) = OMcp4_36;
    sens.A(1) = ACcp4_119;
    sens.A(2) = ACcp4_219;
    sens.A(3) = ACcp4_319;
    sens.OMP(1) = OPcp4_16;
    sens.OMP(2) = OPcp4_26;
    sens.OMP(3) = OPcp4_36;
 
% 
case 6, 


% = = Block_1_0_0_6_0_1 = = 
 
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
    OMcp5_26 = OMcp5_25+ROcp5_85*qd(6);
    OMcp5_36 = OMcp5_35+ROcp5_95*qd(6);
    OPcp5_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp5_26 = ROcp5_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp5_35*S5-ROcp5_95*qd(4));
    OPcp5_36 = ROcp5_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp5_25*S5-ROcp5_85*qd(4));
    RLcp5_120 = ROcp5_46*s.dpt(2,10);
    RLcp5_220 = ROcp5_56*s.dpt(2,10);
    RLcp5_320 = ROcp5_66*s.dpt(2,10);
    POcp5_120 = RLcp5_120+q(1);
    POcp5_220 = RLcp5_220+q(2);
    POcp5_320 = RLcp5_320+q(3);
    ORcp5_120 = OMcp5_26*RLcp5_320-OMcp5_36*RLcp5_220;
    ORcp5_220 = -(OMcp5_16*RLcp5_320-OMcp5_36*RLcp5_120);
    ORcp5_320 = OMcp5_16*RLcp5_220-OMcp5_26*RLcp5_120;
    VIcp5_120 = ORcp5_120+qd(1);
    VIcp5_220 = ORcp5_220+qd(2);
    VIcp5_320 = ORcp5_320+qd(3);
    ACcp5_120 = qdd(1)+OMcp5_26*ORcp5_320-OMcp5_36*ORcp5_220+OPcp5_26*RLcp5_320-OPcp5_36*RLcp5_220;
    ACcp5_220 = qdd(2)-OMcp5_16*ORcp5_320+OMcp5_36*ORcp5_120-OPcp5_16*RLcp5_320+OPcp5_36*RLcp5_120;
    ACcp5_320 = qdd(3)+OMcp5_16*ORcp5_220-OMcp5_26*ORcp5_120+OPcp5_16*RLcp5_220-OPcp5_26*RLcp5_120;

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp5_120;
    sens.P(2) = POcp5_220;
    sens.P(3) = POcp5_320;
    sens.R(1,1) = ROcp5_16;
    sens.R(1,2) = ROcp5_26;
    sens.R(1,3) = ROcp5_36;
    sens.R(2,1) = ROcp5_46;
    sens.R(2,2) = ROcp5_56;
    sens.R(2,3) = ROcp5_66;
    sens.R(3,1) = S5;
    sens.R(3,2) = ROcp5_85;
    sens.R(3,3) = ROcp5_95;
    sens.V(1) = VIcp5_120;
    sens.V(2) = VIcp5_220;
    sens.V(3) = VIcp5_320;
    sens.OM(1) = OMcp5_16;
    sens.OM(2) = OMcp5_26;
    sens.OM(3) = OMcp5_36;
    sens.A(1) = ACcp5_120;
    sens.A(2) = ACcp5_220;
    sens.A(3) = ACcp5_320;
    sens.OMP(1) = OPcp5_16;
    sens.OMP(2) = OPcp5_26;
    sens.OMP(3) = OPcp5_36;
 
% 
case 7, 


% = = Block_1_0_0_7_0_1 = = 
 
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
    OMcp6_26 = OMcp6_25+ROcp6_85*qd(6);
    OMcp6_36 = OMcp6_35+ROcp6_95*qd(6);
    OPcp6_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp6_26 = ROcp6_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp6_35*S5-ROcp6_95*qd(4));
    OPcp6_36 = ROcp6_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp6_25*S5-ROcp6_85*qd(4));

% = = Block_1_0_0_7_0_2 = = 
 
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
    OMcp6_17 = OMcp6_16+ROcp6_46*qd(7);
    OMcp6_27 = OMcp6_26+ROcp6_56*qd(7);
    OMcp6_37 = OMcp6_36+ROcp6_66*qd(7);
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    OMcp6_18 = OMcp6_17+ROcp6_17*qd(8);
    OMcp6_28 = OMcp6_27+ROcp6_27*qd(8);
    OMcp6_38 = OMcp6_37+ROcp6_37*qd(8);
    OPcp6_18 = OPcp6_16+ROcp6_17*qdd(8)+ROcp6_46*qdd(7)+qd(7)*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56)+qd(8)*(OMcp6_27*ROcp6_37-OMcp6_37*ROcp6_27);
    OPcp6_28 = OPcp6_26+ROcp6_27*qdd(8)+ROcp6_56*qdd(7)-qd(7)*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46)-qd(8)*(OMcp6_17*ROcp6_37-OMcp6_37*ROcp6_17);
    OPcp6_38 = OPcp6_36+ROcp6_37*qdd(8)+ROcp6_66*qdd(7)+qd(7)*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46)+qd(8)*(OMcp6_17*ROcp6_27-OMcp6_27*ROcp6_17);
    RLcp6_121 = ROcp6_78*s.dpt(3,11);
    RLcp6_221 = ROcp6_88*s.dpt(3,11);
    RLcp6_321 = ROcp6_98*s.dpt(3,11);
    POcp6_121 = RLcp6_121+RLcp6_17+q(1);
    POcp6_221 = RLcp6_221+RLcp6_27+q(2);
    POcp6_321 = RLcp6_321+RLcp6_37+q(3);
    ORcp6_121 = OMcp6_28*RLcp6_321-OMcp6_38*RLcp6_221;
    ORcp6_221 = -(OMcp6_18*RLcp6_321-OMcp6_38*RLcp6_121);
    ORcp6_321 = OMcp6_18*RLcp6_221-OMcp6_28*RLcp6_121;
    VIcp6_121 = ORcp6_121+ORcp6_17+qd(1);
    VIcp6_221 = ORcp6_221+ORcp6_27+qd(2);
    VIcp6_321 = ORcp6_321+ORcp6_37+qd(3);
    ACcp6_121 = qdd(1)+OMcp6_26*ORcp6_37+OMcp6_28*ORcp6_321-OMcp6_36*ORcp6_27-OMcp6_38*ORcp6_221+OPcp6_26*RLcp6_37+OPcp6_28*RLcp6_321-OPcp6_36*...
 RLcp6_27-OPcp6_38*RLcp6_221;
    ACcp6_221 = qdd(2)-OMcp6_16*ORcp6_37-OMcp6_18*ORcp6_321+OMcp6_36*ORcp6_17+OMcp6_38*ORcp6_121-OPcp6_16*RLcp6_37-OPcp6_18*RLcp6_321+OPcp6_36*...
 RLcp6_17+OPcp6_38*RLcp6_121;
    ACcp6_321 = qdd(3)+OMcp6_16*ORcp6_27+OMcp6_18*ORcp6_221-OMcp6_26*ORcp6_17-OMcp6_28*ORcp6_121+OPcp6_16*RLcp6_27+OPcp6_18*RLcp6_221-OPcp6_26*...
 RLcp6_17-OPcp6_28*RLcp6_121;

% = = Block_1_0_0_7_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp6_121;
    sens.P(2) = POcp6_221;
    sens.P(3) = POcp6_321;
    sens.R(1,1) = ROcp6_17;
    sens.R(1,2) = ROcp6_27;
    sens.R(1,3) = ROcp6_37;
    sens.R(2,1) = ROcp6_48;
    sens.R(2,2) = ROcp6_58;
    sens.R(2,3) = ROcp6_68;
    sens.R(3,1) = ROcp6_78;
    sens.R(3,2) = ROcp6_88;
    sens.R(3,3) = ROcp6_98;
    sens.V(1) = VIcp6_121;
    sens.V(2) = VIcp6_221;
    sens.V(3) = VIcp6_321;
    sens.OM(1) = OMcp6_18;
    sens.OM(2) = OMcp6_28;
    sens.OM(3) = OMcp6_38;
    sens.A(1) = ACcp6_121;
    sens.A(2) = ACcp6_221;
    sens.A(3) = ACcp6_321;
    sens.OMP(1) = OPcp6_18;
    sens.OMP(2) = OPcp6_28;
    sens.OMP(3) = OPcp6_38;
 
% 
case 8, 


% = = Block_1_0_0_8_0_1 = = 
 
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
    OMcp7_26 = OMcp7_25+ROcp7_85*qd(6);
    OMcp7_36 = OMcp7_35+ROcp7_95*qd(6);
    OPcp7_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp7_26 = ROcp7_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp7_35*S5-ROcp7_95*qd(4));
    OPcp7_36 = ROcp7_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp7_25*S5-ROcp7_85*qd(4));

% = = Block_1_0_0_8_0_3 = = 
 
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
    OMcp7_19 = OMcp7_16+ROcp7_46*qd(9);
    OMcp7_29 = OMcp7_26+ROcp7_56*qd(9);
    OMcp7_39 = OMcp7_36+ROcp7_66*qd(9);
    ORcp7_19 = OMcp7_26*RLcp7_39-OMcp7_36*RLcp7_29;
    ORcp7_29 = -(OMcp7_16*RLcp7_39-OMcp7_36*RLcp7_19);
    ORcp7_39 = OMcp7_16*RLcp7_29-OMcp7_26*RLcp7_19;
    OMcp7_110 = OMcp7_19+ROcp7_19*qd(10);
    OMcp7_210 = OMcp7_29+ROcp7_29*qd(10);
    OMcp7_310 = OMcp7_39+ROcp7_39*qd(10);
    OPcp7_110 = OPcp7_16+ROcp7_19*qdd(10)+ROcp7_46*qdd(9)+qd(10)*(OMcp7_29*ROcp7_39-OMcp7_39*ROcp7_29)+qd(9)*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56);
    OPcp7_210 = OPcp7_26+ROcp7_29*qdd(10)+ROcp7_56*qdd(9)-qd(10)*(OMcp7_19*ROcp7_39-OMcp7_39*ROcp7_19)-qd(9)*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46);
    OPcp7_310 = OPcp7_36+ROcp7_39*qdd(10)+ROcp7_66*qdd(9)+qd(10)*(OMcp7_19*ROcp7_29-OMcp7_29*ROcp7_19)+qd(9)*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46);
    RLcp7_122 = ROcp7_710*s.dpt(3,12);
    RLcp7_222 = ROcp7_810*s.dpt(3,12);
    RLcp7_322 = ROcp7_910*s.dpt(3,12);
    POcp7_122 = RLcp7_122+RLcp7_19+q(1);
    POcp7_222 = RLcp7_222+RLcp7_29+q(2);
    POcp7_322 = RLcp7_322+RLcp7_39+q(3);
    ORcp7_122 = OMcp7_210*RLcp7_322-OMcp7_310*RLcp7_222;
    ORcp7_222 = -(OMcp7_110*RLcp7_322-OMcp7_310*RLcp7_122);
    ORcp7_322 = OMcp7_110*RLcp7_222-OMcp7_210*RLcp7_122;
    VIcp7_122 = ORcp7_122+ORcp7_19+qd(1);
    VIcp7_222 = ORcp7_222+ORcp7_29+qd(2);
    VIcp7_322 = ORcp7_322+ORcp7_39+qd(3);
    ACcp7_122 = qdd(1)+OMcp7_210*ORcp7_322+OMcp7_26*ORcp7_39-OMcp7_310*ORcp7_222-OMcp7_36*ORcp7_29+OPcp7_210*RLcp7_322+OPcp7_26*RLcp7_39-OPcp7_310*...
 RLcp7_222-OPcp7_36*RLcp7_29;
    ACcp7_222 = qdd(2)-OMcp7_110*ORcp7_322-OMcp7_16*ORcp7_39+OMcp7_310*ORcp7_122+OMcp7_36*ORcp7_19-OPcp7_110*RLcp7_322-OPcp7_16*RLcp7_39+OPcp7_310*...
 RLcp7_122+OPcp7_36*RLcp7_19;
    ACcp7_322 = qdd(3)+OMcp7_110*ORcp7_222+OMcp7_16*ORcp7_29-OMcp7_210*ORcp7_122-OMcp7_26*ORcp7_19+OPcp7_110*RLcp7_222+OPcp7_16*RLcp7_29-OPcp7_210*...
 RLcp7_122-OPcp7_26*RLcp7_19;

% = = Block_1_0_0_8_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp7_122;
    sens.P(2) = POcp7_222;
    sens.P(3) = POcp7_322;
    sens.R(1,1) = ROcp7_19;
    sens.R(1,2) = ROcp7_29;
    sens.R(1,3) = ROcp7_39;
    sens.R(2,1) = ROcp7_410;
    sens.R(2,2) = ROcp7_510;
    sens.R(2,3) = ROcp7_610;
    sens.R(3,1) = ROcp7_710;
    sens.R(3,2) = ROcp7_810;
    sens.R(3,3) = ROcp7_910;
    sens.V(1) = VIcp7_122;
    sens.V(2) = VIcp7_222;
    sens.V(3) = VIcp7_322;
    sens.OM(1) = OMcp7_110;
    sens.OM(2) = OMcp7_210;
    sens.OM(3) = OMcp7_310;
    sens.A(1) = ACcp7_122;
    sens.A(2) = ACcp7_222;
    sens.A(3) = ACcp7_322;
    sens.OMP(1) = OPcp7_110;
    sens.OMP(2) = OPcp7_210;
    sens.OMP(3) = OPcp7_310;
 
% 
case 9, 


% = = Block_1_0_0_9_0_1 = = 
 
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
    OMcp8_26 = OMcp8_25+ROcp8_85*qd(6);
    OMcp8_36 = OMcp8_35+ROcp8_95*qd(6);
    OPcp8_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp8_26 = ROcp8_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp8_35*S5-ROcp8_95*qd(4));
    OPcp8_36 = ROcp8_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp8_25*S5-ROcp8_85*qd(4));

% = = Block_1_0_0_9_0_4 = = 
 
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
    OMcp8_111 = OMcp8_16+ROcp8_46*qd(11);
    OMcp8_211 = OMcp8_26+ROcp8_56*qd(11);
    OMcp8_311 = OMcp8_36+ROcp8_66*qd(11);
    ORcp8_111 = OMcp8_26*RLcp8_311-OMcp8_36*RLcp8_211;
    ORcp8_211 = -(OMcp8_16*RLcp8_311-OMcp8_36*RLcp8_111);
    ORcp8_311 = OMcp8_16*RLcp8_211-OMcp8_26*RLcp8_111;
    OMcp8_112 = OMcp8_111+ROcp8_111*qd(12);
    OMcp8_212 = OMcp8_211+ROcp8_211*qd(12);
    OMcp8_312 = OMcp8_311+ROcp8_311*qd(12);
    OPcp8_112 = OPcp8_16+ROcp8_111*qdd(12)+ROcp8_46*qdd(11)+qd(11)*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56)+qd(12)*(OMcp8_211*ROcp8_311-OMcp8_311*...
 ROcp8_211);
    OPcp8_212 = OPcp8_26+ROcp8_211*qdd(12)+ROcp8_56*qdd(11)-qd(11)*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46)-qd(12)*(OMcp8_111*ROcp8_311-OMcp8_311*...
 ROcp8_111);
    OPcp8_312 = OPcp8_36+ROcp8_311*qdd(12)+ROcp8_66*qdd(11)+qd(11)*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46)+qd(12)*(OMcp8_111*ROcp8_211-OMcp8_211*...
 ROcp8_111);
    RLcp8_123 = ROcp8_712*s.dpt(3,13);
    RLcp8_223 = ROcp8_812*s.dpt(3,13);
    RLcp8_323 = ROcp8_912*s.dpt(3,13);
    POcp8_123 = RLcp8_111+RLcp8_123+q(1);
    POcp8_223 = RLcp8_211+RLcp8_223+q(2);
    POcp8_323 = RLcp8_311+RLcp8_323+q(3);
    ORcp8_123 = OMcp8_212*RLcp8_323-OMcp8_312*RLcp8_223;
    ORcp8_223 = -(OMcp8_112*RLcp8_323-OMcp8_312*RLcp8_123);
    ORcp8_323 = OMcp8_112*RLcp8_223-OMcp8_212*RLcp8_123;
    VIcp8_123 = ORcp8_111+ORcp8_123+qd(1);
    VIcp8_223 = ORcp8_211+ORcp8_223+qd(2);
    VIcp8_323 = ORcp8_311+ORcp8_323+qd(3);
    ACcp8_123 = qdd(1)+OMcp8_212*ORcp8_323+OMcp8_26*ORcp8_311-OMcp8_312*ORcp8_223-OMcp8_36*ORcp8_211+OPcp8_212*RLcp8_323+OPcp8_26*RLcp8_311-...
 OPcp8_312*RLcp8_223-OPcp8_36*RLcp8_211;
    ACcp8_223 = qdd(2)-OMcp8_112*ORcp8_323-OMcp8_16*ORcp8_311+OMcp8_312*ORcp8_123+OMcp8_36*ORcp8_111-OPcp8_112*RLcp8_323-OPcp8_16*RLcp8_311+...
 OPcp8_312*RLcp8_123+OPcp8_36*RLcp8_111;
    ACcp8_323 = qdd(3)+OMcp8_112*ORcp8_223+OMcp8_16*ORcp8_211-OMcp8_212*ORcp8_123-OMcp8_26*ORcp8_111+OPcp8_112*RLcp8_223+OPcp8_16*RLcp8_211-...
 OPcp8_212*RLcp8_123-OPcp8_26*RLcp8_111;

% = = Block_1_0_0_9_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp8_123;
    sens.P(2) = POcp8_223;
    sens.P(3) = POcp8_323;
    sens.R(1,1) = ROcp8_111;
    sens.R(1,2) = ROcp8_211;
    sens.R(1,3) = ROcp8_311;
    sens.R(2,1) = ROcp8_412;
    sens.R(2,2) = ROcp8_512;
    sens.R(2,3) = ROcp8_612;
    sens.R(3,1) = ROcp8_712;
    sens.R(3,2) = ROcp8_812;
    sens.R(3,3) = ROcp8_912;
    sens.V(1) = VIcp8_123;
    sens.V(2) = VIcp8_223;
    sens.V(3) = VIcp8_323;
    sens.OM(1) = OMcp8_112;
    sens.OM(2) = OMcp8_212;
    sens.OM(3) = OMcp8_312;
    sens.A(1) = ACcp8_123;
    sens.A(2) = ACcp8_223;
    sens.A(3) = ACcp8_323;
    sens.OMP(1) = OPcp8_112;
    sens.OMP(2) = OPcp8_212;
    sens.OMP(3) = OPcp8_312;
 
% 
case 10, 


% = = Block_1_0_0_10_0_1 = = 
 
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
    OMcp9_26 = OMcp9_25+ROcp9_85*qd(6);
    OMcp9_36 = OMcp9_35+ROcp9_95*qd(6);
    OPcp9_16 = qdd(4)+qdd(6)*S5+qd(5)*qd(6)*C5;
    OPcp9_26 = ROcp9_85*qdd(6)+qdd(5)*C4-qd(4)*qd(5)*S4+qd(6)*(OMcp9_35*S5-ROcp9_95*qd(4));
    OPcp9_36 = ROcp9_95*qdd(6)+qdd(5)*S4+qd(4)*qd(5)*C4-qd(6)*(OMcp9_25*S5-ROcp9_85*qd(4));

% = = Block_1_0_0_10_0_5 = = 
 
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
    OMcp9_113 = OMcp9_16+ROcp9_46*qd(13);
    OMcp9_213 = OMcp9_26+ROcp9_56*qd(13);
    OMcp9_313 = OMcp9_36+ROcp9_66*qd(13);
    ORcp9_113 = OMcp9_26*RLcp9_313-OMcp9_36*RLcp9_213;
    ORcp9_213 = -(OMcp9_16*RLcp9_313-OMcp9_36*RLcp9_113);
    ORcp9_313 = OMcp9_16*RLcp9_213-OMcp9_26*RLcp9_113;
    OMcp9_114 = OMcp9_113+ROcp9_113*qd(14);
    OMcp9_214 = OMcp9_213+ROcp9_213*qd(14);
    OMcp9_314 = OMcp9_313+ROcp9_313*qd(14);
    OPcp9_114 = OPcp9_16+ROcp9_113*qdd(14)+ROcp9_46*qdd(13)+qd(13)*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56)+qd(14)*(OMcp9_213*ROcp9_313-OMcp9_313*...
 ROcp9_213);
    OPcp9_214 = OPcp9_26+ROcp9_213*qdd(14)+ROcp9_56*qdd(13)-qd(13)*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46)-qd(14)*(OMcp9_113*ROcp9_313-OMcp9_313*...
 ROcp9_113);
    OPcp9_314 = OPcp9_36+ROcp9_313*qdd(14)+ROcp9_66*qdd(13)+qd(13)*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46)+qd(14)*(OMcp9_113*ROcp9_213-OMcp9_213*...
 ROcp9_113);
    RLcp9_124 = ROcp9_714*s.dpt(3,14);
    RLcp9_224 = ROcp9_814*s.dpt(3,14);
    RLcp9_324 = ROcp9_914*s.dpt(3,14);
    POcp9_124 = RLcp9_113+RLcp9_124+q(1);
    POcp9_224 = RLcp9_213+RLcp9_224+q(2);
    POcp9_324 = RLcp9_313+RLcp9_324+q(3);
    ORcp9_124 = OMcp9_214*RLcp9_324-OMcp9_314*RLcp9_224;
    ORcp9_224 = -(OMcp9_114*RLcp9_324-OMcp9_314*RLcp9_124);
    ORcp9_324 = OMcp9_114*RLcp9_224-OMcp9_214*RLcp9_124;
    VIcp9_124 = ORcp9_113+ORcp9_124+qd(1);
    VIcp9_224 = ORcp9_213+ORcp9_224+qd(2);
    VIcp9_324 = ORcp9_313+ORcp9_324+qd(3);
    ACcp9_124 = qdd(1)+OMcp9_214*ORcp9_324+OMcp9_26*ORcp9_313-OMcp9_314*ORcp9_224-OMcp9_36*ORcp9_213+OPcp9_214*RLcp9_324+OPcp9_26*RLcp9_313-...
 OPcp9_314*RLcp9_224-OPcp9_36*RLcp9_213;
    ACcp9_224 = qdd(2)-OMcp9_114*ORcp9_324-OMcp9_16*ORcp9_313+OMcp9_314*ORcp9_124+OMcp9_36*ORcp9_113-OPcp9_114*RLcp9_324-OPcp9_16*RLcp9_313+...
 OPcp9_314*RLcp9_124+OPcp9_36*RLcp9_113;
    ACcp9_324 = qdd(3)+OMcp9_114*ORcp9_224+OMcp9_16*ORcp9_213-OMcp9_214*ORcp9_124-OMcp9_26*ORcp9_113+OPcp9_114*RLcp9_224+OPcp9_16*RLcp9_213-...
 OPcp9_214*RLcp9_124-OPcp9_26*RLcp9_113;

% = = Block_1_0_0_10_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp9_124;
    sens.P(2) = POcp9_224;
    sens.P(3) = POcp9_324;
    sens.R(1,1) = ROcp9_113;
    sens.R(1,2) = ROcp9_213;
    sens.R(1,3) = ROcp9_313;
    sens.R(2,1) = ROcp9_414;
    sens.R(2,2) = ROcp9_514;
    sens.R(2,3) = ROcp9_614;
    sens.R(3,1) = ROcp9_714;
    sens.R(3,2) = ROcp9_814;
    sens.R(3,3) = ROcp9_914;
    sens.V(1) = VIcp9_124;
    sens.V(2) = VIcp9_224;
    sens.V(3) = VIcp9_324;
    sens.OM(1) = OMcp9_114;
    sens.OM(2) = OMcp9_214;
    sens.OM(3) = OMcp9_314;
    sens.A(1) = ACcp9_124;
    sens.A(2) = ACcp9_224;
    sens.A(3) = ACcp9_324;
    sens.OMP(1) = OPcp9_114;
    sens.OMP(2) = OPcp9_214;
    sens.OMP(3) = OPcp9_314;

end


% ====== END Task 1 ====== 

  

