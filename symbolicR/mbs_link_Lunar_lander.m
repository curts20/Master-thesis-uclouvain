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
%	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
%	==> Flops complexity : 607
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.020 seconds
%
%-------------------------------------------------------------
%
function [frc,trq,Flnk,Z,Zd] = link(s,tsim,usrfun)

 frc = zeros(3,14);
 trq = zeros(3,14);
 Flnk = zeros(8,1);
 Z = zeros(8,1);
 Zd = zeros(8,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_0_1_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk1_78 = S7*C8;
  ROlnk1_98 = C7*C8;
  RLlnk1_116 = ROlnk1_78*s.dpt(3,14);
  RLlnk1_216 = -s.dpt(3,14)*S8;
  RLlnk1_316 = ROlnk1_98*s.dpt(3,14);

% = = Block_0_1_0_0_2_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk2_78 = S7*C8;
  ROlnk2_98 = C7*C8;
  RLlnk2_117 = ROlnk2_78*s.dpt(3,14);
  RLlnk2_217 = -s.dpt(3,14)*S8;
  RLlnk2_317 = ROlnk2_98*s.dpt(3,14);

% = = Block_0_1_0_0_3_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk4_710 = C10*S9;
  ROlnk4_910 = C10*C9;
  RLlnk4_119 = ROlnk4_710*s.dpt(3,16);
  RLlnk4_219 = -s.dpt(3,16)*S10;
  RLlnk4_319 = ROlnk4_910*s.dpt(3,16);

% = = Block_0_1_0_0_4_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk6_710 = C10*S9;
  ROlnk6_910 = C10*C9;
  RLlnk6_121 = ROlnk6_710*s.dpt(3,16);
  RLlnk6_221 = -s.dpt(3,16)*S10;
  RLlnk6_321 = ROlnk6_910*s.dpt(3,16);

% = = Block_0_1_0_0_5_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk9_712 = S11*C12;
  ROlnk9_912 = C11*C12;
  RLlnk9_124 = ROlnk9_712*s.dpt(3,18);
  RLlnk9_224 = -s.dpt(3,18)*S12;
  RLlnk9_324 = ROlnk9_912*s.dpt(3,18);

% = = Block_0_1_0_0_6_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk11_712 = S11*C12;
  ROlnk11_912 = C11*C12;
  RLlnk11_126 = ROlnk11_712*s.dpt(3,18);
  RLlnk11_226 = -s.dpt(3,18)*S12;
  RLlnk11_326 = ROlnk11_912*s.dpt(3,18);

% = = Block_0_1_0_0_7_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk12_714 = S13*C14;
  ROlnk12_914 = C13*C14;
  RLlnk12_127 = ROlnk12_714*s.dpt(3,20);
  RLlnk12_227 = -s.dpt(3,20)*S14;
  RLlnk12_327 = ROlnk12_914*s.dpt(3,20);

% = = Block_0_1_0_0_8_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk14_714 = S13*C14;
  ROlnk14_914 = C13*C14;
  RLlnk14_129 = ROlnk14_714*s.dpt(3,20);
  RLlnk14_229 = -s.dpt(3,20)*S14;
  RLlnk14_329 = ROlnk14_914*s.dpt(3,20);

% = = Block_0_1_0_1_1_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk11 = RLlnk1_116+s.dpt(1,1)-s.dpt(1,5);
  Plnk21 = RLlnk1_216-s.dpt(2,5);
  Z1 = sqrt(Plnk11*Plnk11+Plnk21*Plnk21+RLlnk1_316*RLlnk1_316);
  e11 = Plnk11/Z1;
  e21 = Plnk21/Z1;
  e31 = RLlnk1_316/Z1;
  Zd1 = -(qd(8)*e21*s.dpt(3,14)*C8-e11*(qd(7)*RLlnk1_316+qd(8)*RLlnk1_216*S7)+e31*(qd(7)*RLlnk1_116-qd(8)*RLlnk1_216*C7));
 
% Link Force Computation 

  Flink1 = usrfun.flink(Z1,Zd1,s,tsim,1);

% = = Block_0_1_0_1_2_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk12 = -(RLlnk2_117+s.dpt(1,1)-s.dpt(1,6));
  Plnk22 = -(RLlnk2_217-s.dpt(2,6));
  Z2 = sqrt(Plnk12*Plnk12+Plnk22*Plnk22+RLlnk2_317*RLlnk2_317);
  e12 = Plnk12/Z2;
  e22 = Plnk22/Z2;
  e32 = -RLlnk2_317/Z2;
  Zd2 = qd(8)*e22*s.dpt(3,14)*C8-e12*(qd(7)*RLlnk2_317+qd(8)*RLlnk2_217*S7)+e32*(qd(7)*RLlnk2_117-qd(8)*RLlnk2_217*C7);
 
% Link Force Computation 

  Flink2 = usrfun.flink(Z2,Zd2,s,tsim,2);

% = = Block_0_1_0_1_3_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk13 = -(RLlnk4_119-s.dpt(1,8));
  Plnk23 = -(RLlnk4_219+s.dpt(2,2)-s.dpt(2,8));
  Z3 = sqrt(Plnk13*Plnk13+Plnk23*Plnk23+RLlnk4_319*RLlnk4_319);
  e13 = Plnk13/Z3;
  e23 = Plnk23/Z3;
  e33 = -RLlnk4_319/Z3;
  Zd3 = qd(10)*e23*s.dpt(3,16)*C10-e13*(qd(10)*RLlnk4_219*S9+qd(9)*RLlnk4_319)-e33*(qd(10)*RLlnk4_219*C9-qd(9)*RLlnk4_119);
 
% Link Force Computation 

  Flink3 = usrfun.flink(Z3,Zd3,s,tsim,3);

% = = Block_0_1_0_1_4_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk14 = -(RLlnk6_121-s.dpt(1,7));
  Plnk24 = -(RLlnk6_221+s.dpt(2,2)-s.dpt(2,7));
  Z4 = sqrt(Plnk14*Plnk14+Plnk24*Plnk24+RLlnk6_321*RLlnk6_321);
  e14 = Plnk14/Z4;
  e24 = Plnk24/Z4;
  e34 = -RLlnk6_321/Z4;
  Zd4 = qd(10)*e24*s.dpt(3,16)*C10-e14*(qd(10)*RLlnk6_221*S9+qd(9)*RLlnk6_321)-e34*(qd(10)*RLlnk6_221*C9-qd(9)*RLlnk6_121);
 
% Link Force Computation 

  Flink4 = usrfun.flink(Z4,Zd4,s,tsim,4);

% = = Block_0_1_0_1_5_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk15 = RLlnk9_124-s.dpt(1,10)+s.dpt(1,3);
  Plnk25 = RLlnk9_224-s.dpt(2,10);
  Z5 = sqrt(Plnk15*Plnk15+Plnk25*Plnk25+RLlnk9_324*RLlnk9_324);
  e15 = Plnk15/Z5;
  e25 = Plnk25/Z5;
  e35 = RLlnk9_324/Z5;
  Zd5 = -(qd(12)*e25*s.dpt(3,18)*C12-e15*(qd(11)*RLlnk9_324+qd(12)*RLlnk9_224*S11)+e35*(qd(11)*RLlnk9_124-qd(12)*RLlnk9_224*C11));
 
% Link Force Computation 

  Flink5 = usrfun.flink(Z5,Zd5,s,tsim,5);

% = = Block_0_1_0_1_6_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk16 = RLlnk11_126+s.dpt(1,3)-s.dpt(1,9);
  Plnk26 = RLlnk11_226-s.dpt(2,9);
  Z6 = sqrt(Plnk16*Plnk16+Plnk26*Plnk26+RLlnk11_326*RLlnk11_326);
  e16 = Plnk16/Z6;
  e26 = Plnk26/Z6;
  e36 = RLlnk11_326/Z6;
  Zd6 = -(qd(12)*e26*s.dpt(3,18)*C12-e16*(qd(11)*RLlnk11_326+qd(12)*RLlnk11_226*S11)+e36*(qd(11)*RLlnk11_126-qd(12)*RLlnk11_226*C11));
 
% Link Force Computation 

  Flink6 = usrfun.flink(Z6,Zd6,s,tsim,6);

% = = Block_0_1_0_1_7_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk17 = -(RLlnk12_127-s.dpt(1,12));
  Plnk27 = -(RLlnk12_227-s.dpt(2,12)+s.dpt(2,4));
  Z7 = sqrt(Plnk17*Plnk17+Plnk27*Plnk27+RLlnk12_327*RLlnk12_327);
  e17 = Plnk17/Z7;
  e27 = Plnk27/Z7;
  e37 = -RLlnk12_327/Z7;
  Zd7 = qd(14)*e27*s.dpt(3,20)*C14-e17*(qd(13)*RLlnk12_327+qd(14)*RLlnk12_227*S13)+e37*(qd(13)*RLlnk12_127-qd(14)*RLlnk12_227*C13);
 
% Link Force Computation 

  Flink7 = usrfun.flink(Z7,Zd7,s,tsim,7);

% = = Block_0_1_0_1_8_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk18 = -(RLlnk14_129-s.dpt(1,11));
  Plnk28 = -(RLlnk14_229-s.dpt(2,11)+s.dpt(2,4));
  Z8 = sqrt(Plnk18*Plnk18+Plnk28*Plnk28+RLlnk14_329*RLlnk14_329);
  e18 = Plnk18/Z8;
  e28 = Plnk28/Z8;
  e38 = -RLlnk14_329/Z8;
  Zd8 = qd(14)*e28*s.dpt(3,20)*C14-e18*(qd(13)*RLlnk14_329+qd(14)*RLlnk14_229*S13)+e38*(qd(13)*RLlnk14_129-qd(14)*RLlnk14_229*C13);
 
% Link Force Computation 

  Flink8 = usrfun.flink(Z8,Zd8,s,tsim,8);

% = = Block_0_1_0_2_2_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk11 = Flink1*e11;
  fPlnk21 = Flink1*e21;
  fPlnk31 = Flink1*e31;

% = = Block_0_1_0_2_2_2 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk11 = Flink1*(e11*C7-e31*S7);
  fSlnk21 = Flink1*(e21*C8+S8*(e11*S7+e31*C7));
  s.frc(1,8) = s.frc(1,8)-fSlnk11;
  s.frc(2,8) = s.frc(2,8)-fSlnk21;
  s.frc(3,8) = s.frc(3,8)-Flink1*(ROlnk1_78*e11+ROlnk1_98*e31-e21*S8);
  s.trq(1,8) = s.trq(1,8)+fSlnk21*(s.dpt(3,14)-s.l(3,8));
  s.trq(2,8) = s.trq(2,8)-fSlnk11*(s.dpt(3,14)-s.l(3,8));

% = = Block_0_1_0_2_3_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk12 = Flink2*e12;
  fSlnk22 = Flink2*e22;
  fSlnk32 = Flink2*e32;

% = = Block_0_1_0_2_3_2 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk12 = Flink2*(e12*C7-e32*S7);
  fPlnk22 = Flink2*(e22*C8+S8*(e12*S7+e32*C7));
  fPlnk32 = Flink2*(ROlnk2_78*e12+ROlnk2_98*e32-e22*S8);
  frc(1,8) = fPlnk12+s.frc(1,8);
  frc(2,8) = fPlnk22+s.frc(2,8);
  frc(3,8) = fPlnk32+s.frc(3,8);
  trq(1,8) = s.trq(1,8)-fPlnk22*(s.dpt(3,14)-s.l(3,8));
  trq(2,8) = s.trq(2,8)+fPlnk12*(s.dpt(3,14)-s.l(3,8));

% = = Block_0_1_0_2_4_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk13 = Flink3*e13;
  fSlnk23 = Flink3*e23;
  fSlnk33 = Flink3*e33;

% = = Block_0_1_0_2_4_3 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk13 = Flink3*(e13*C9-e33*S9);
  fPlnk23 = Flink3*(e23*C10+S10*(e13*S9+e33*C9));
  s.frc(1,10) = s.frc(1,10)+fPlnk13;
  s.frc(2,10) = s.frc(2,10)+fPlnk23;
  s.frc(3,10) = s.frc(3,10)+Flink3*(ROlnk4_710*e13+ROlnk4_910*e33-e23*S10);
  s.trq(1,10) = s.trq(1,10)-fPlnk23*(s.dpt(3,16)-s.l(3,10));
  s.trq(2,10) = s.trq(2,10)+fPlnk13*(s.dpt(3,16)-s.l(3,10));

% = = Block_0_1_0_2_5_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk14 = Flink4*e14;
  fSlnk24 = Flink4*e24;
  fSlnk34 = Flink4*e34;

% = = Block_0_1_0_2_5_3 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk14 = Flink4*(e14*C9-e34*S9);
  fPlnk24 = Flink4*(e24*C10+S10*(e14*S9+e34*C9));
  fPlnk34 = Flink4*(ROlnk6_710*e14+ROlnk6_910*e34-e24*S10);
  frc(1,10) = fPlnk14+s.frc(1,10);
  frc(2,10) = fPlnk24+s.frc(2,10);
  frc(3,10) = fPlnk34+s.frc(3,10);
  trq(1,10) = s.trq(1,10)-fPlnk24*(s.dpt(3,16)-s.l(3,10));
  trq(2,10) = s.trq(2,10)+fPlnk14*(s.dpt(3,16)-s.l(3,10));

% = = Block_0_1_0_2_6_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk15 = Flink5*e15;
  fPlnk25 = Flink5*e25;
  fPlnk35 = Flink5*e35;

% = = Block_0_1_0_2_6_4 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk15 = Flink5*(e15*C11-e35*S11);
  fSlnk25 = Flink5*(e25*C12+S12*(e15*S11+e35*C11));
  s.frc(1,12) = s.frc(1,12)-fSlnk15;
  s.frc(2,12) = s.frc(2,12)-fSlnk25;
  s.frc(3,12) = s.frc(3,12)-Flink5*(ROlnk9_712*e15+ROlnk9_912*e35-e25*S12);
  s.trq(1,12) = s.trq(1,12)+fSlnk25*(s.dpt(3,18)-s.l(3,12));
  s.trq(2,12) = s.trq(2,12)-fSlnk15*(s.dpt(3,18)-s.l(3,12));

% = = Block_0_1_0_2_7_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk16 = Flink6*e16;
  fPlnk26 = Flink6*e26;
  fPlnk36 = Flink6*e36;

% = = Block_0_1_0_2_7_4 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk16 = Flink6*(e16*C11-e36*S11);
  fSlnk26 = Flink6*(e26*C12+S12*(e16*S11+e36*C11));
  fSlnk36 = Flink6*(ROlnk11_712*e16+ROlnk11_912*e36-e26*S12);
  frc(1,12) = -(fSlnk16-s.frc(1,12));
  frc(2,12) = -(fSlnk26-s.frc(2,12));
  frc(3,12) = -(fSlnk36-s.frc(3,12));
  trq(1,12) = s.trq(1,12)+fSlnk26*(s.dpt(3,18)-s.l(3,12));
  trq(2,12) = s.trq(2,12)-fSlnk16*(s.dpt(3,18)-s.l(3,12));

% = = Block_0_1_0_2_8_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk17 = Flink7*e17;
  fSlnk27 = Flink7*e27;
  fSlnk37 = Flink7*e37;
  s.frc(1,6) = s.frc(1,6)+fPlnk11+fPlnk15+fPlnk16-fSlnk12-fSlnk13-fSlnk14-fSlnk17;
  s.frc(2,6) = s.frc(2,6)+fPlnk21+fPlnk25+fPlnk26-fSlnk22-fSlnk23-fSlnk24-fSlnk27;
  s.frc(3,6) = s.frc(3,6)+fPlnk31+fPlnk35+fPlnk36-fSlnk32-fSlnk33-fSlnk34-fSlnk37;
  s.trq(1,6) = s.trq(1,6)+fPlnk21*s.l(3,6)+fPlnk25*s.l(3,6)+fPlnk26*s.l(3,6)+fPlnk31*s.dpt(2,5)+fPlnk35*s.dpt(2,10)+fPlnk36*s.dpt(2,9)-fSlnk22*...
 s.l(3,6)-fSlnk23*s.l(3,6)-fSlnk24*s.l(3,6)-fSlnk27*s.l(3,6)-fSlnk32*s.dpt(2,6)-fSlnk33*s.dpt(2,8)-fSlnk34*s.dpt(2,7)-fSlnk37*s.dpt(2,12);
  s.trq(2,6) = s.trq(2,6)-fPlnk11*s.l(3,6)-fPlnk15*s.l(3,6)-fPlnk16*s.l(3,6)-fPlnk31*s.dpt(1,5)-fPlnk35*s.dpt(1,10)-fPlnk36*s.dpt(1,9)+fSlnk12*...
 s.l(3,6)+fSlnk13*s.l(3,6)+fSlnk14*s.l(3,6)+fSlnk17*s.l(3,6)+fSlnk32*s.dpt(1,6)+fSlnk33*s.dpt(1,8)+fSlnk34*s.dpt(1,7)+fSlnk37*s.dpt(1,12);
  s.trq(3,6) = s.trq(3,6)-fPlnk11*s.dpt(2,5)-fPlnk15*s.dpt(2,10)-fPlnk16*s.dpt(2,9)+fPlnk21*s.dpt(1,5)+fPlnk25*s.dpt(1,10)+fPlnk26*s.dpt(1,9)+...
 fSlnk12*s.dpt(2,6)+fSlnk13*s.dpt(2,8)+fSlnk14*s.dpt(2,7)+fSlnk17*s.dpt(2,12)-fSlnk22*s.dpt(1,6)-fSlnk23*s.dpt(1,8)-fSlnk24*s.dpt(1,7)-fSlnk27*...
 s.dpt(1,12);

% = = Block_0_1_0_2_8_5 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk17 = Flink7*(e17*C13-e37*S13);
  fPlnk27 = Flink7*(e27*C14+S14*(e17*S13+e37*C13));
  s.frc(1,14) = s.frc(1,14)+fPlnk17;
  s.frc(2,14) = s.frc(2,14)+fPlnk27;
  s.frc(3,14) = s.frc(3,14)+Flink7*(ROlnk12_714*e17+ROlnk12_914*e37-e27*S14);
  s.trq(1,14) = s.trq(1,14)-fPlnk27*(s.dpt(3,20)-s.l(3,14));
  s.trq(2,14) = s.trq(2,14)+fPlnk17*(s.dpt(3,20)-s.l(3,14));

% = = Block_0_1_0_2_9_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk18 = Flink8*e18;
  fSlnk28 = Flink8*e28;
  fSlnk38 = Flink8*e38;
  frc(1,6) = -(fSlnk18-s.frc(1,6));
  frc(2,6) = -(fSlnk28-s.frc(2,6));
  frc(3,6) = -(fSlnk38-s.frc(3,6));
  trq(1,6) = s.trq(1,6)-fSlnk28*s.l(3,6)-fSlnk38*s.dpt(2,11);
  trq(2,6) = s.trq(2,6)+fSlnk18*s.l(3,6)+fSlnk38*s.dpt(1,11);
  trq(3,6) = s.trq(3,6)+fSlnk18*s.dpt(2,11)-fSlnk28*s.dpt(1,11);

% = = Block_0_1_0_2_9_5 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk18 = Flink8*(e18*C13-e38*S13);
  fPlnk28 = Flink8*(e28*C14+S14*(e18*S13+e38*C13));
  fPlnk38 = Flink8*(ROlnk14_714*e18+ROlnk14_914*e38-e28*S14);
  frc(1,14) = fPlnk18+s.frc(1,14);
  frc(2,14) = fPlnk28+s.frc(2,14);
  frc(3,14) = fPlnk38+s.frc(3,14);
  trq(1,14) = s.trq(1,14)-fPlnk28*(s.dpt(3,20)-s.l(3,14));
  trq(2,14) = s.trq(2,14)+fPlnk18*(s.dpt(3,20)-s.l(3,14));

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  trq(3,8) = s.trq(3,8);
  trq(3,10) = s.trq(3,10);
  trq(3,12) = s.trq(3,12);
  trq(3,14) = s.trq(3,14);
  Flnk(1) = Flink1;
  Flnk(2) = Flink2;
  Flnk(3) = Flink3;
  Flnk(4) = Flink4;
  Flnk(5) = Flink5;
  Flnk(6) = Flink6;
  Flnk(7) = Flink7;
  Flnk(8) = Flink8;
  Z(1) = Z1;
  Z(2) = Z2;
  Z(3) = Z3;
  Z(4) = Z4;
  Z(5) = Z5;
  Z(6) = Z6;
  Z(7) = Z7;
  Z(8) = Z8;
  Zd(1) = Zd1;
  Zd(2) = Zd2;
  Zd(3) = Zd3;
  Zd(4) = Zd4;
  Zd(5) = Zd5;
  Zd(6) = Zd6;
  Zd(7) = Zd7;
  Zd(8) = Zd8;

% ====== END Task 0 ====== 

  

