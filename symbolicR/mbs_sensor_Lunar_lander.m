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
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 970
%
%	==> Generation Time :  0.020 seconds
%	==> Post-Processing :  0.020 seconds
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

% = = Block_1_0_0_1_0_2 = = 
 
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
    OMcp0_17 = OMcp0_16+ROcp0_46*qd(7);
    OMcp0_27 = OMcp0_26+ROcp0_56*qd(7);
    OMcp0_37 = OMcp0_36+ROcp0_66*qd(7);
    ORcp0_17 = OMcp0_26*RLcp0_37-OMcp0_36*RLcp0_27;
    ORcp0_27 = -(OMcp0_16*RLcp0_37-OMcp0_36*RLcp0_17);
    ORcp0_37 = OMcp0_16*RLcp0_27-OMcp0_26*RLcp0_17;
    OMcp0_18 = OMcp0_17+ROcp0_17*qd(8);
    OMcp0_28 = OMcp0_27+ROcp0_27*qd(8);
    OMcp0_38 = OMcp0_37+ROcp0_37*qd(8);
    OPcp0_18 = OPcp0_16+ROcp0_17*qdd(8)+ROcp0_46*qdd(7)+qd(7)*(OMcp0_26*ROcp0_66-OMcp0_36*ROcp0_56)+qd(8)*(OMcp0_27*ROcp0_37-OMcp0_37*ROcp0_27);
    OPcp0_28 = OPcp0_26+ROcp0_27*qdd(8)+ROcp0_56*qdd(7)-qd(7)*(OMcp0_16*ROcp0_66-OMcp0_36*ROcp0_46)-qd(8)*(OMcp0_17*ROcp0_37-OMcp0_37*ROcp0_17);
    OPcp0_38 = OPcp0_36+ROcp0_37*qdd(8)+ROcp0_66*qdd(7)+qd(7)*(OMcp0_16*ROcp0_56-OMcp0_26*ROcp0_46)+qd(8)*(OMcp0_17*ROcp0_27-OMcp0_27*ROcp0_17);
    RLcp0_131 = ROcp0_78*s.dpt(3,13);
    RLcp0_231 = ROcp0_88*s.dpt(3,13);
    RLcp0_331 = ROcp0_98*s.dpt(3,13);
    POcp0_131 = RLcp0_131+RLcp0_17+q(1);
    POcp0_231 = RLcp0_231+RLcp0_27+q(2);
    POcp0_331 = RLcp0_331+RLcp0_37+q(3);
    ORcp0_131 = OMcp0_28*RLcp0_331-OMcp0_38*RLcp0_231;
    ORcp0_231 = -(OMcp0_18*RLcp0_331-OMcp0_38*RLcp0_131);
    ORcp0_331 = OMcp0_18*RLcp0_231-OMcp0_28*RLcp0_131;
    VIcp0_131 = ORcp0_131+ORcp0_17+qd(1);
    VIcp0_231 = ORcp0_231+ORcp0_27+qd(2);
    VIcp0_331 = ORcp0_331+ORcp0_37+qd(3);
    ACcp0_131 = qdd(1)+OMcp0_26*ORcp0_37+OMcp0_28*ORcp0_331-OMcp0_36*ORcp0_27-OMcp0_38*ORcp0_231+OPcp0_26*RLcp0_37+OPcp0_28*RLcp0_331-OPcp0_36*...
 RLcp0_27-OPcp0_38*RLcp0_231;
    ACcp0_231 = qdd(2)-OMcp0_16*ORcp0_37-OMcp0_18*ORcp0_331+OMcp0_36*ORcp0_17+OMcp0_38*ORcp0_131-OPcp0_16*RLcp0_37-OPcp0_18*RLcp0_331+OPcp0_36*...
 RLcp0_17+OPcp0_38*RLcp0_131;
    ACcp0_331 = qdd(3)+OMcp0_16*ORcp0_27+OMcp0_18*ORcp0_231-OMcp0_26*ORcp0_17-OMcp0_28*ORcp0_131+OPcp0_16*RLcp0_27+OPcp0_18*RLcp0_231-OPcp0_26*...
 RLcp0_17-OPcp0_28*RLcp0_131;

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp0_131;
    sens.P(2) = POcp0_231;
    sens.P(3) = POcp0_331;
    sens.R(1,1) = ROcp0_17;
    sens.R(1,2) = ROcp0_27;
    sens.R(1,3) = ROcp0_37;
    sens.R(2,1) = ROcp0_48;
    sens.R(2,2) = ROcp0_58;
    sens.R(2,3) = ROcp0_68;
    sens.R(3,1) = ROcp0_78;
    sens.R(3,2) = ROcp0_88;
    sens.R(3,3) = ROcp0_98;
    sens.V(1) = VIcp0_131;
    sens.V(2) = VIcp0_231;
    sens.V(3) = VIcp0_331;
    sens.OM(1) = OMcp0_18;
    sens.OM(2) = OMcp0_28;
    sens.OM(3) = OMcp0_38;
    sens.A(1) = ACcp0_131;
    sens.A(2) = ACcp0_231;
    sens.A(3) = ACcp0_331;
    sens.OMP(1) = OPcp0_18;
    sens.OMP(2) = OPcp0_28;
    sens.OMP(3) = OPcp0_38;
 
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

% = = Block_1_0_0_2_0_3 = = 
 
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
    OMcp1_19 = OMcp1_16+ROcp1_46*qd(9);
    OMcp1_29 = OMcp1_26+ROcp1_56*qd(9);
    OMcp1_39 = OMcp1_36+ROcp1_66*qd(9);
    ORcp1_19 = OMcp1_26*RLcp1_39-OMcp1_36*RLcp1_29;
    ORcp1_29 = -(OMcp1_16*RLcp1_39-OMcp1_36*RLcp1_19);
    ORcp1_39 = OMcp1_16*RLcp1_29-OMcp1_26*RLcp1_19;
    OMcp1_110 = OMcp1_19+ROcp1_19*qd(10);
    OMcp1_210 = OMcp1_29+ROcp1_29*qd(10);
    OMcp1_310 = OMcp1_39+ROcp1_39*qd(10);
    OPcp1_110 = OPcp1_16+ROcp1_19*qdd(10)+ROcp1_46*qdd(9)+qd(10)*(OMcp1_29*ROcp1_39-OMcp1_39*ROcp1_29)+qd(9)*(OMcp1_26*ROcp1_66-OMcp1_36*ROcp1_56);
    OPcp1_210 = OPcp1_26+ROcp1_29*qdd(10)+ROcp1_56*qdd(9)-qd(10)*(OMcp1_19*ROcp1_39-OMcp1_39*ROcp1_19)-qd(9)*(OMcp1_16*ROcp1_66-OMcp1_36*ROcp1_46);
    OPcp1_310 = OPcp1_36+ROcp1_39*qdd(10)+ROcp1_66*qdd(9)+qd(10)*(OMcp1_19*ROcp1_29-OMcp1_29*ROcp1_19)+qd(9)*(OMcp1_16*ROcp1_56-OMcp1_26*ROcp1_46);
    RLcp1_132 = ROcp1_710*s.dpt(3,15);
    RLcp1_232 = ROcp1_810*s.dpt(3,15);
    RLcp1_332 = ROcp1_910*s.dpt(3,15);
    POcp1_132 = RLcp1_132+RLcp1_19+q(1);
    POcp1_232 = RLcp1_232+RLcp1_29+q(2);
    POcp1_332 = RLcp1_332+RLcp1_39+q(3);
    ORcp1_132 = OMcp1_210*RLcp1_332-OMcp1_310*RLcp1_232;
    ORcp1_232 = -(OMcp1_110*RLcp1_332-OMcp1_310*RLcp1_132);
    ORcp1_332 = OMcp1_110*RLcp1_232-OMcp1_210*RLcp1_132;
    VIcp1_132 = ORcp1_132+ORcp1_19+qd(1);
    VIcp1_232 = ORcp1_232+ORcp1_29+qd(2);
    VIcp1_332 = ORcp1_332+ORcp1_39+qd(3);
    ACcp1_132 = qdd(1)+OMcp1_210*ORcp1_332+OMcp1_26*ORcp1_39-OMcp1_310*ORcp1_232-OMcp1_36*ORcp1_29+OPcp1_210*RLcp1_332+OPcp1_26*RLcp1_39-OPcp1_310*...
 RLcp1_232-OPcp1_36*RLcp1_29;
    ACcp1_232 = qdd(2)-OMcp1_110*ORcp1_332-OMcp1_16*ORcp1_39+OMcp1_310*ORcp1_132+OMcp1_36*ORcp1_19-OPcp1_110*RLcp1_332-OPcp1_16*RLcp1_39+OPcp1_310*...
 RLcp1_132+OPcp1_36*RLcp1_19;
    ACcp1_332 = qdd(3)+OMcp1_110*ORcp1_232+OMcp1_16*ORcp1_29-OMcp1_210*ORcp1_132-OMcp1_26*ORcp1_19+OPcp1_110*RLcp1_232+OPcp1_16*RLcp1_29-OPcp1_210*...
 RLcp1_132-OPcp1_26*RLcp1_19;

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp1_132;
    sens.P(2) = POcp1_232;
    sens.P(3) = POcp1_332;
    sens.R(1,1) = ROcp1_19;
    sens.R(1,2) = ROcp1_29;
    sens.R(1,3) = ROcp1_39;
    sens.R(2,1) = ROcp1_410;
    sens.R(2,2) = ROcp1_510;
    sens.R(2,3) = ROcp1_610;
    sens.R(3,1) = ROcp1_710;
    sens.R(3,2) = ROcp1_810;
    sens.R(3,3) = ROcp1_910;
    sens.V(1) = VIcp1_132;
    sens.V(2) = VIcp1_232;
    sens.V(3) = VIcp1_332;
    sens.OM(1) = OMcp1_110;
    sens.OM(2) = OMcp1_210;
    sens.OM(3) = OMcp1_310;
    sens.A(1) = ACcp1_132;
    sens.A(2) = ACcp1_232;
    sens.A(3) = ACcp1_332;
    sens.OMP(1) = OPcp1_110;
    sens.OMP(2) = OPcp1_210;
    sens.OMP(3) = OPcp1_310;
 
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

% = = Block_1_0_0_3_0_4 = = 
 
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
    OMcp2_111 = OMcp2_16+ROcp2_46*qd(11);
    OMcp2_211 = OMcp2_26+ROcp2_56*qd(11);
    OMcp2_311 = OMcp2_36+ROcp2_66*qd(11);
    ORcp2_111 = OMcp2_26*RLcp2_311-OMcp2_36*RLcp2_211;
    ORcp2_211 = -(OMcp2_16*RLcp2_311-OMcp2_36*RLcp2_111);
    ORcp2_311 = OMcp2_16*RLcp2_211-OMcp2_26*RLcp2_111;
    OMcp2_112 = OMcp2_111+ROcp2_111*qd(12);
    OMcp2_212 = OMcp2_211+ROcp2_211*qd(12);
    OMcp2_312 = OMcp2_311+ROcp2_311*qd(12);
    OPcp2_112 = OPcp2_16+ROcp2_111*qdd(12)+ROcp2_46*qdd(11)+qd(11)*(OMcp2_26*ROcp2_66-OMcp2_36*ROcp2_56)+qd(12)*(OMcp2_211*ROcp2_311-OMcp2_311*...
 ROcp2_211);
    OPcp2_212 = OPcp2_26+ROcp2_211*qdd(12)+ROcp2_56*qdd(11)-qd(11)*(OMcp2_16*ROcp2_66-OMcp2_36*ROcp2_46)-qd(12)*(OMcp2_111*ROcp2_311-OMcp2_311*...
 ROcp2_111);
    OPcp2_312 = OPcp2_36+ROcp2_311*qdd(12)+ROcp2_66*qdd(11)+qd(11)*(OMcp2_16*ROcp2_56-OMcp2_26*ROcp2_46)+qd(12)*(OMcp2_111*ROcp2_211-OMcp2_211*...
 ROcp2_111);
    RLcp2_133 = ROcp2_712*s.dpt(3,17);
    RLcp2_233 = ROcp2_812*s.dpt(3,17);
    RLcp2_333 = ROcp2_912*s.dpt(3,17);
    POcp2_133 = RLcp2_111+RLcp2_133+q(1);
    POcp2_233 = RLcp2_211+RLcp2_233+q(2);
    POcp2_333 = RLcp2_311+RLcp2_333+q(3);
    ORcp2_133 = OMcp2_212*RLcp2_333-OMcp2_312*RLcp2_233;
    ORcp2_233 = -(OMcp2_112*RLcp2_333-OMcp2_312*RLcp2_133);
    ORcp2_333 = OMcp2_112*RLcp2_233-OMcp2_212*RLcp2_133;
    VIcp2_133 = ORcp2_111+ORcp2_133+qd(1);
    VIcp2_233 = ORcp2_211+ORcp2_233+qd(2);
    VIcp2_333 = ORcp2_311+ORcp2_333+qd(3);
    ACcp2_133 = qdd(1)+OMcp2_212*ORcp2_333+OMcp2_26*ORcp2_311-OMcp2_312*ORcp2_233-OMcp2_36*ORcp2_211+OPcp2_212*RLcp2_333+OPcp2_26*RLcp2_311-...
 OPcp2_312*RLcp2_233-OPcp2_36*RLcp2_211;
    ACcp2_233 = qdd(2)-OMcp2_112*ORcp2_333-OMcp2_16*ORcp2_311+OMcp2_312*ORcp2_133+OMcp2_36*ORcp2_111-OPcp2_112*RLcp2_333-OPcp2_16*RLcp2_311+...
 OPcp2_312*RLcp2_133+OPcp2_36*RLcp2_111;
    ACcp2_333 = qdd(3)+OMcp2_112*ORcp2_233+OMcp2_16*ORcp2_211-OMcp2_212*ORcp2_133-OMcp2_26*ORcp2_111+OPcp2_112*RLcp2_233+OPcp2_16*RLcp2_211-...
 OPcp2_212*RLcp2_133-OPcp2_26*RLcp2_111;

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp2_133;
    sens.P(2) = POcp2_233;
    sens.P(3) = POcp2_333;
    sens.R(1,1) = ROcp2_111;
    sens.R(1,2) = ROcp2_211;
    sens.R(1,3) = ROcp2_311;
    sens.R(2,1) = ROcp2_412;
    sens.R(2,2) = ROcp2_512;
    sens.R(2,3) = ROcp2_612;
    sens.R(3,1) = ROcp2_712;
    sens.R(3,2) = ROcp2_812;
    sens.R(3,3) = ROcp2_912;
    sens.V(1) = VIcp2_133;
    sens.V(2) = VIcp2_233;
    sens.V(3) = VIcp2_333;
    sens.OM(1) = OMcp2_112;
    sens.OM(2) = OMcp2_212;
    sens.OM(3) = OMcp2_312;
    sens.A(1) = ACcp2_133;
    sens.A(2) = ACcp2_233;
    sens.A(3) = ACcp2_333;
    sens.OMP(1) = OPcp2_112;
    sens.OMP(2) = OPcp2_212;
    sens.OMP(3) = OPcp2_312;
 
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

% = = Block_1_0_0_4_0_5 = = 
 
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
    OMcp3_113 = OMcp3_16+ROcp3_46*qd(13);
    OMcp3_213 = OMcp3_26+ROcp3_56*qd(13);
    OMcp3_313 = OMcp3_36+ROcp3_66*qd(13);
    ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
    ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
    ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
    OMcp3_114 = OMcp3_113+ROcp3_113*qd(14);
    OMcp3_214 = OMcp3_213+ROcp3_213*qd(14);
    OMcp3_314 = OMcp3_313+ROcp3_313*qd(14);
    OPcp3_114 = OPcp3_16+ROcp3_113*qdd(14)+ROcp3_46*qdd(13)+qd(13)*(OMcp3_26*ROcp3_66-OMcp3_36*ROcp3_56)+qd(14)*(OMcp3_213*ROcp3_313-OMcp3_313*...
 ROcp3_213);
    OPcp3_214 = OPcp3_26+ROcp3_213*qdd(14)+ROcp3_56*qdd(13)-qd(13)*(OMcp3_16*ROcp3_66-OMcp3_36*ROcp3_46)-qd(14)*(OMcp3_113*ROcp3_313-OMcp3_313*...
 ROcp3_113);
    OPcp3_314 = OPcp3_36+ROcp3_313*qdd(14)+ROcp3_66*qdd(13)+qd(13)*(OMcp3_16*ROcp3_56-OMcp3_26*ROcp3_46)+qd(14)*(OMcp3_113*ROcp3_213-OMcp3_213*...
 ROcp3_113);
    RLcp3_134 = ROcp3_714*s.dpt(3,19);
    RLcp3_234 = ROcp3_814*s.dpt(3,19);
    RLcp3_334 = ROcp3_914*s.dpt(3,19);
    POcp3_134 = RLcp3_113+RLcp3_134+q(1);
    POcp3_234 = RLcp3_213+RLcp3_234+q(2);
    POcp3_334 = RLcp3_313+RLcp3_334+q(3);
    ORcp3_134 = OMcp3_214*RLcp3_334-OMcp3_314*RLcp3_234;
    ORcp3_234 = -(OMcp3_114*RLcp3_334-OMcp3_314*RLcp3_134);
    ORcp3_334 = OMcp3_114*RLcp3_234-OMcp3_214*RLcp3_134;
    VIcp3_134 = ORcp3_113+ORcp3_134+qd(1);
    VIcp3_234 = ORcp3_213+ORcp3_234+qd(2);
    VIcp3_334 = ORcp3_313+ORcp3_334+qd(3);
    ACcp3_134 = qdd(1)+OMcp3_214*ORcp3_334+OMcp3_26*ORcp3_313-OMcp3_314*ORcp3_234-OMcp3_36*ORcp3_213+OPcp3_214*RLcp3_334+OPcp3_26*RLcp3_313-...
 OPcp3_314*RLcp3_234-OPcp3_36*RLcp3_213;
    ACcp3_234 = qdd(2)-OMcp3_114*ORcp3_334-OMcp3_16*ORcp3_313+OMcp3_314*ORcp3_134+OMcp3_36*ORcp3_113-OPcp3_114*RLcp3_334-OPcp3_16*RLcp3_313+...
 OPcp3_314*RLcp3_134+OPcp3_36*RLcp3_113;
    ACcp3_334 = qdd(3)+OMcp3_114*ORcp3_234+OMcp3_16*ORcp3_213-OMcp3_214*ORcp3_134-OMcp3_26*ORcp3_113+OPcp3_114*RLcp3_234+OPcp3_16*RLcp3_213-...
 OPcp3_214*RLcp3_134-OPcp3_26*RLcp3_113;

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp3_134;
    sens.P(2) = POcp3_234;
    sens.P(3) = POcp3_334;
    sens.R(1,1) = ROcp3_113;
    sens.R(1,2) = ROcp3_213;
    sens.R(1,3) = ROcp3_313;
    sens.R(2,1) = ROcp3_414;
    sens.R(2,2) = ROcp3_514;
    sens.R(2,3) = ROcp3_614;
    sens.R(3,1) = ROcp3_714;
    sens.R(3,2) = ROcp3_814;
    sens.R(3,3) = ROcp3_914;
    sens.V(1) = VIcp3_134;
    sens.V(2) = VIcp3_234;
    sens.V(3) = VIcp3_334;
    sens.OM(1) = OMcp3_114;
    sens.OM(2) = OMcp3_214;
    sens.OM(3) = OMcp3_314;
    sens.A(1) = ACcp3_134;
    sens.A(2) = ACcp3_234;
    sens.A(3) = ACcp3_334;
    sens.OMP(1) = OPcp3_114;
    sens.OMP(2) = OPcp3_214;
    sens.OMP(3) = OPcp3_314;

end


% ====== END Task 1 ====== 

  

