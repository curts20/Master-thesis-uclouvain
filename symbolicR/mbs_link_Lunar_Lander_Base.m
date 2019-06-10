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
%	==> Generation Date : Mon Nov 19 12:22:29 2018
%
%	==> Project name : Lunar_Lander_Base
%	==> using XML input file 
%
%	==> Number of joints : 15
%
%	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
%	==> Flops complexity : 607
%
%	==> Generation Time :  0.010 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [frc,trq,Flnk,Z,Zd] = link(s,tsim,usrfun)

 frc = zeros(3,15);
 trq = zeros(3,15);
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

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C8 = cos(q(8));
  S8 = sin(q(8));
  C9 = cos(q(9));
  S9 = sin(q(9));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C10 = cos(q(10));
  S10 = sin(q(10));
  C11 = cos(q(11));
  S11 = sin(q(11));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C12 = cos(q(12));
  S12 = sin(q(12));
  C13 = cos(q(13));
  S13 = sin(q(13));

% = = Block_0_0_0_0_0_6 = = 
 
% Trigonometric Variables  

  C14 = cos(q(14));
  S14 = sin(q(14));
  C15 = cos(q(15));
  S15 = sin(q(15));

% = = Block_0_1_0_0_1_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk1_79 = S8*C9;
  ROlnk1_99 = C8*C9;
  RLlnk1_117 = ROlnk1_79*s.dpt(3,14);
  RLlnk1_217 = -s.dpt(3,14)*S9;
  RLlnk1_317 = ROlnk1_99*s.dpt(3,14);

% = = Block_0_1_0_0_2_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk2_79 = S8*C9;
  ROlnk2_99 = C8*C9;
  RLlnk2_118 = ROlnk2_79*s.dpt(3,14);
  RLlnk2_218 = -s.dpt(3,14)*S9;
  RLlnk2_318 = ROlnk2_99*s.dpt(3,14);

% = = Block_0_1_0_0_3_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk4_711 = S10*C11;
  ROlnk4_911 = C10*C11;
  RLlnk4_120 = ROlnk4_711*s.dpt(3,16);
  RLlnk4_220 = -s.dpt(3,16)*S11;
  RLlnk4_320 = ROlnk4_911*s.dpt(3,16);

% = = Block_0_1_0_0_4_4 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk6_711 = S10*C11;
  ROlnk6_911 = C10*C11;
  RLlnk6_122 = ROlnk6_711*s.dpt(3,16);
  RLlnk6_222 = -s.dpt(3,16)*S11;
  RLlnk6_322 = ROlnk6_911*s.dpt(3,16);

% = = Block_0_1_0_0_5_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk9_713 = S12*C13;
  ROlnk9_913 = C12*C13;
  RLlnk9_125 = ROlnk9_713*s.dpt(3,18);
  RLlnk9_225 = -s.dpt(3,18)*S13;
  RLlnk9_325 = ROlnk9_913*s.dpt(3,18);

% = = Block_0_1_0_0_6_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk11_713 = S12*C13;
  ROlnk11_913 = C12*C13;
  RLlnk11_127 = ROlnk11_713*s.dpt(3,18);
  RLlnk11_227 = -s.dpt(3,18)*S13;
  RLlnk11_327 = ROlnk11_913*s.dpt(3,18);

% = = Block_0_1_0_0_7_6 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk12_715 = S14*C15;
  ROlnk12_915 = C14*C15;
  RLlnk12_128 = ROlnk12_715*s.dpt(3,20);
  RLlnk12_228 = -s.dpt(3,20)*S15;
  RLlnk12_328 = ROlnk12_915*s.dpt(3,20);

% = = Block_0_1_0_0_8_6 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  ROlnk14_715 = S14*C15;
  ROlnk14_915 = C14*C15;
  RLlnk14_130 = ROlnk14_715*s.dpt(3,20);
  RLlnk14_230 = -s.dpt(3,20)*S15;
  RLlnk14_330 = ROlnk14_915*s.dpt(3,20);

% = = Block_0_1_0_1_1_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk11 = RLlnk1_117+s.dpt(1,1)-s.dpt(1,5);
  Plnk21 = RLlnk1_217-s.dpt(2,5);
  Z1 = sqrt(Plnk11*Plnk11+Plnk21*Plnk21+RLlnk1_317*RLlnk1_317);
  e11 = Plnk11/Z1;
  e21 = Plnk21/Z1;
  e31 = RLlnk1_317/Z1;
  Zd1 = -(qd(9)*e21*s.dpt(3,14)*C9-e11*(qd(8)*RLlnk1_317+qd(9)*RLlnk1_217*S8)+e31*(qd(8)*RLlnk1_117-qd(9)*RLlnk1_217*C8));
 
% Link Force Computation 

  Flink1 = usrfun.flink(Z1,Zd1,s,tsim,1);

% = = Block_0_1_0_1_2_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk12 = -(RLlnk2_118+s.dpt(1,1)-s.dpt(1,6));
  Plnk22 = -(RLlnk2_218-s.dpt(2,6));
  Z2 = sqrt(Plnk12*Plnk12+Plnk22*Plnk22+RLlnk2_318*RLlnk2_318);
  e12 = Plnk12/Z2;
  e22 = Plnk22/Z2;
  e32 = -RLlnk2_318/Z2;
  Zd2 = qd(9)*e22*s.dpt(3,14)*C9-e12*(qd(8)*RLlnk2_318+qd(9)*RLlnk2_218*S8)+e32*(qd(8)*RLlnk2_118-qd(9)*RLlnk2_218*C8);
 
% Link Force Computation 

  Flink2 = usrfun.flink(Z2,Zd2,s,tsim,2);

% = = Block_0_1_0_1_3_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk13 = -(RLlnk4_120-s.dpt(1,8));
  Plnk23 = -(RLlnk4_220+s.dpt(2,2)-s.dpt(2,8));
  Z3 = sqrt(Plnk13*Plnk13+Plnk23*Plnk23+RLlnk4_320*RLlnk4_320);
  e13 = Plnk13/Z3;
  e23 = Plnk23/Z3;
  e33 = -RLlnk4_320/Z3;
  Zd3 = qd(11)*e23*s.dpt(3,16)*C11-e13*(qd(10)*RLlnk4_320+qd(11)*RLlnk4_220*S10)+e33*(qd(10)*RLlnk4_120-qd(11)*RLlnk4_220*C10);
 
% Link Force Computation 

  Flink3 = usrfun.flink(Z3,Zd3,s,tsim,3);

% = = Block_0_1_0_1_4_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk14 = -(RLlnk6_122-s.dpt(1,7));
  Plnk24 = -(RLlnk6_222+s.dpt(2,2)-s.dpt(2,7));
  Z4 = sqrt(Plnk14*Plnk14+Plnk24*Plnk24+RLlnk6_322*RLlnk6_322);
  e14 = Plnk14/Z4;
  e24 = Plnk24/Z4;
  e34 = -RLlnk6_322/Z4;
  Zd4 = qd(11)*e24*s.dpt(3,16)*C11-e14*(qd(10)*RLlnk6_322+qd(11)*RLlnk6_222*S10)+e34*(qd(10)*RLlnk6_122-qd(11)*RLlnk6_222*C10);
 
% Link Force Computation 

  Flink4 = usrfun.flink(Z4,Zd4,s,tsim,4);

% = = Block_0_1_0_1_5_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk15 = RLlnk9_125-s.dpt(1,10)+s.dpt(1,3);
  Plnk25 = RLlnk9_225-s.dpt(2,10);
  Z5 = sqrt(Plnk15*Plnk15+Plnk25*Plnk25+RLlnk9_325*RLlnk9_325);
  e15 = Plnk15/Z5;
  e25 = Plnk25/Z5;
  e35 = RLlnk9_325/Z5;
  Zd5 = -(qd(13)*e25*s.dpt(3,18)*C13-e15*(qd(12)*RLlnk9_325+qd(13)*RLlnk9_225*S12)+e35*(qd(12)*RLlnk9_125-qd(13)*RLlnk9_225*C12));
 
% Link Force Computation 

  Flink5 = usrfun.flink(Z5,Zd5,s,tsim,5);

% = = Block_0_1_0_1_6_5 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk16 = RLlnk11_127+s.dpt(1,3)-s.dpt(1,9);
  Plnk26 = RLlnk11_227-s.dpt(2,9);
  Z6 = sqrt(Plnk16*Plnk16+Plnk26*Plnk26+RLlnk11_327*RLlnk11_327);
  e16 = Plnk16/Z6;
  e26 = Plnk26/Z6;
  e36 = RLlnk11_327/Z6;
  Zd6 = -(qd(13)*e26*s.dpt(3,18)*C13-e16*(qd(12)*RLlnk11_327+qd(13)*RLlnk11_227*S12)+e36*(qd(12)*RLlnk11_127-qd(13)*RLlnk11_227*C12));
 
% Link Force Computation 

  Flink6 = usrfun.flink(Z6,Zd6,s,tsim,6);

% = = Block_0_1_0_1_7_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk17 = -(RLlnk12_128-s.dpt(1,12));
  Plnk27 = -(RLlnk12_228-s.dpt(2,12)+s.dpt(2,4));
  Z7 = sqrt(Plnk17*Plnk17+Plnk27*Plnk27+RLlnk12_328*RLlnk12_328);
  e17 = Plnk17/Z7;
  e27 = Plnk27/Z7;
  e37 = -RLlnk12_328/Z7;
  Zd7 = qd(15)*e27*s.dpt(3,20)*C15-e17*(qd(14)*RLlnk12_328+qd(15)*RLlnk12_228*S14)+e37*(qd(14)*RLlnk12_128-qd(15)*RLlnk12_228*C14);
 
% Link Force Computation 

  Flink7 = usrfun.flink(Z7,Zd7,s,tsim,7);

% = = Block_0_1_0_1_8_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk18 = -(RLlnk14_130-s.dpt(1,11));
  Plnk28 = -(RLlnk14_230-s.dpt(2,11)+s.dpt(2,4));
  Z8 = sqrt(Plnk18*Plnk18+Plnk28*Plnk28+RLlnk14_330*RLlnk14_330);
  e18 = Plnk18/Z8;
  e28 = Plnk28/Z8;
  e38 = -RLlnk14_330/Z8;
  Zd8 = qd(15)*e28*s.dpt(3,20)*C15-e18*(qd(14)*RLlnk14_330+qd(15)*RLlnk14_230*S14)+e38*(qd(14)*RLlnk14_130-qd(15)*RLlnk14_230*C14);
 
% Link Force Computation 

  Flink8 = usrfun.flink(Z8,Zd8,s,tsim,8);

% = = Block_0_1_0_2_2_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk11 = Flink1*e11;
  fPlnk21 = Flink1*e21;
  fPlnk31 = Flink1*e31;

% = = Block_0_1_0_2_2_3 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk11 = Flink1*(e11*C8-e31*S8);
  fSlnk21 = Flink1*(e21*C9+S9*(e11*S8+e31*C8));
  s.frc(1,9) = s.frc(1,9)-fSlnk11;
  s.frc(2,9) = s.frc(2,9)-fSlnk21;
  s.frc(3,9) = s.frc(3,9)-Flink1*(ROlnk1_79*e11+ROlnk1_99*e31-e21*S9);
  s.trq(1,9) = s.trq(1,9)+fSlnk21*(s.dpt(3,14)-s.l(3,9));
  s.trq(2,9) = s.trq(2,9)-fSlnk11*(s.dpt(3,14)-s.l(3,9));

% = = Block_0_1_0_2_3_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk12 = Flink2*e12;
  fSlnk22 = Flink2*e22;
  fSlnk32 = Flink2*e32;

% = = Block_0_1_0_2_3_3 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk12 = Flink2*(e12*C8-e32*S8);
  fPlnk22 = Flink2*(e22*C9+S9*(e12*S8+e32*C8));
  fPlnk32 = Flink2*(ROlnk2_79*e12+ROlnk2_99*e32-e22*S9);
  frc(1,9) = fPlnk12+s.frc(1,9);
  frc(2,9) = fPlnk22+s.frc(2,9);
  frc(3,9) = fPlnk32+s.frc(3,9);
  trq(1,9) = s.trq(1,9)-fPlnk22*(s.dpt(3,14)-s.l(3,9));
  trq(2,9) = s.trq(2,9)+fPlnk12*(s.dpt(3,14)-s.l(3,9));

% = = Block_0_1_0_2_4_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk13 = Flink3*e13;
  fSlnk23 = Flink3*e23;
  fSlnk33 = Flink3*e33;

% = = Block_0_1_0_2_4_4 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk13 = Flink3*(e13*C10-e33*S10);
  fPlnk23 = Flink3*(e23*C11+S11*(e13*S10+e33*C10));
  s.frc(1,11) = s.frc(1,11)+fPlnk13;
  s.frc(2,11) = s.frc(2,11)+fPlnk23;
  s.frc(3,11) = s.frc(3,11)+Flink3*(ROlnk4_711*e13+ROlnk4_911*e33-e23*S11);
  s.trq(1,11) = s.trq(1,11)-fPlnk23*(s.dpt(3,16)-s.l(3,11));
  s.trq(2,11) = s.trq(2,11)+fPlnk13*(s.dpt(3,16)-s.l(3,11));

% = = Block_0_1_0_2_5_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk14 = Flink4*e14;
  fSlnk24 = Flink4*e24;
  fSlnk34 = Flink4*e34;

% = = Block_0_1_0_2_5_4 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk14 = Flink4*(e14*C10-e34*S10);
  fPlnk24 = Flink4*(e24*C11+S11*(e14*S10+e34*C10));
  fPlnk34 = Flink4*(ROlnk6_711*e14+ROlnk6_911*e34-e24*S11);
  frc(1,11) = fPlnk14+s.frc(1,11);
  frc(2,11) = fPlnk24+s.frc(2,11);
  frc(3,11) = fPlnk34+s.frc(3,11);
  trq(1,11) = s.trq(1,11)-fPlnk24*(s.dpt(3,16)-s.l(3,11));
  trq(2,11) = s.trq(2,11)+fPlnk14*(s.dpt(3,16)-s.l(3,11));

% = = Block_0_1_0_2_6_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk15 = Flink5*e15;
  fPlnk25 = Flink5*e25;
  fPlnk35 = Flink5*e35;

% = = Block_0_1_0_2_6_5 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk15 = Flink5*(e15*C12-e35*S12);
  fSlnk25 = Flink5*(e25*C13+S13*(e15*S12+e35*C12));
  s.frc(1,13) = s.frc(1,13)-fSlnk15;
  s.frc(2,13) = s.frc(2,13)-fSlnk25;
  s.frc(3,13) = s.frc(3,13)-Flink5*(ROlnk9_713*e15+ROlnk9_913*e35-e25*S13);
  s.trq(1,13) = s.trq(1,13)+fSlnk25*(s.dpt(3,18)-s.l(3,13));
  s.trq(2,13) = s.trq(2,13)-fSlnk15*(s.dpt(3,18)-s.l(3,13));

% = = Block_0_1_0_2_7_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk16 = Flink6*e16;
  fPlnk26 = Flink6*e26;
  fPlnk36 = Flink6*e36;

% = = Block_0_1_0_2_7_5 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk16 = Flink6*(e16*C12-e36*S12);
  fSlnk26 = Flink6*(e26*C13+S13*(e16*S12+e36*C12));
  fSlnk36 = Flink6*(ROlnk11_713*e16+ROlnk11_913*e36-e26*S13);
  frc(1,13) = -(fSlnk16-s.frc(1,13));
  frc(2,13) = -(fSlnk26-s.frc(2,13));
  frc(3,13) = -(fSlnk36-s.frc(3,13));
  trq(1,13) = s.trq(1,13)+fSlnk26*(s.dpt(3,18)-s.l(3,13));
  trq(2,13) = s.trq(2,13)-fSlnk16*(s.dpt(3,18)-s.l(3,13));

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

% = = Block_0_1_0_2_8_6 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk17 = Flink7*(e17*C14-e37*S14);
  fPlnk27 = Flink7*(e27*C15+S15*(e17*S14+e37*C14));
  s.frc(1,15) = s.frc(1,15)+fPlnk17;
  s.frc(2,15) = s.frc(2,15)+fPlnk27;
  s.frc(3,15) = s.frc(3,15)+Flink7*(ROlnk12_715*e17+ROlnk12_915*e37-e27*S15);
  s.trq(1,15) = s.trq(1,15)-fPlnk27*(s.dpt(3,20)-s.l(3,15));
  s.trq(2,15) = s.trq(2,15)+fPlnk17*(s.dpt(3,20)-s.l(3,15));

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

% = = Block_0_1_0_2_9_6 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk18 = Flink8*(e18*C14-e38*S14);
  fPlnk28 = Flink8*(e28*C15+S15*(e18*S14+e38*C14));
  fPlnk38 = Flink8*(ROlnk14_715*e18+ROlnk14_915*e38-e28*S15);
  frc(1,15) = fPlnk18+s.frc(1,15);
  frc(2,15) = fPlnk28+s.frc(2,15);
  frc(3,15) = fPlnk38+s.frc(3,15);
  trq(1,15) = s.trq(1,15)-fPlnk28*(s.dpt(3,20)-s.l(3,15));
  trq(2,15) = s.trq(2,15)+fPlnk18*(s.dpt(3,20)-s.l(3,15));

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  frc(1,7) = s.frc(1,7);
  frc(2,7) = s.frc(2,7);
  frc(3,7) = s.frc(3,7);
  trq(1,7) = s.trq(1,7);
  trq(2,7) = s.trq(2,7);
  trq(3,7) = s.trq(3,7);
  trq(3,9) = s.trq(3,9);
  trq(3,11) = s.trq(3,11);
  trq(3,13) = s.trq(3,13);
  trq(3,15) = s.trq(3,15);
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

  

