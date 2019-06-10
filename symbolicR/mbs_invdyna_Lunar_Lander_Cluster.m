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
%	==> Generation Date : Sun Mar 24 10:34:44 2019
%
%	==> Project name : Lunar_Lander_Cluster
%	==> using XML input file 
%
%	==> Number of joints : 14
%
%	==> Function : F 2 : Inverse Dynamics : RNEA
%	==> Flops complexity : 816
%
%	==> Generation Time :  0.010 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [Qq] = invdyna(s,tsim,usrfun)

 Qq = zeros(14,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_0_0_0 = = 
 
% Forward Kinematics 

  ALPHA33 = qdd(3)-s.g(3);
  ALPHA24 = qdd(2)*C4+ALPHA33*S4;
  ALPHA34 = -(qdd(2)*S4-ALPHA33*C4);
  OM15 = qd(4)*C5;
  OMp15 = -(qd(4)*qd(5)*S5-qdd(4)*C5);
  ALPHA15 = qdd(1)*C5-ALPHA34*S5;
  ALPHA35 = qdd(1)*S5+ALPHA34*C5;
  OM16 = qd(5)*S6+OM15*C6;
  OM26 = qd(5)*C6-OM15*S6;
  OM36 = qd(6)+qd(4)*S5;
  OMp16 = C6*(OMp15+qd(5)*qd(6))+S6*(qdd(5)-qd(6)*OM15);
  OMp26 = C6*(qdd(5)-qd(6)*OM15)-S6*(OMp15+qd(5)*qd(6));
  OMp36 = qdd(6)+qd(4)*qd(5)*C5+qdd(4)*S5;
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BETA26 = BS26-OMp36;
  BETA46 = BS26+OMp36;
  BETA76 = BS36-OMp26;
  BETA86 = BS66+OMp16;
  ALPHA16 = ALPHA15*C6+ALPHA24*S6;
  ALPHA26 = -(ALPHA15*S6-ALPHA24*C6);
  OM27 = qd(7)+OM26;
  OM37 = OM16*S7+OM36*C7;
  OMp27 = qdd(7)+OMp26;
  OMp37 = C7*(OMp36+qd(7)*OM16)+S7*(OMp16-qd(7)*OM36);
  ALPHA27 = ALPHA26+BETA46*s.dpt(1,2);
  ALPHA37 = C7*(ALPHA35+BETA76*s.dpt(1,2))+S7*(ALPHA16+BS16*s.dpt(1,2));
  OM18 = qd(8)+OM16*C7-OM36*S7;
  OM28 = OM27*C8+OM37*S8;
  OM38 = -(OM27*S8-OM37*C8);
  OMp18 = qdd(8)+C7*(OMp16-qd(7)*OM36)-S7*(OMp36+qd(7)*OM16);
  OMp28 = C8*(OMp27+qd(8)*OM37)+S8*(OMp37-qd(8)*OM27);
  OM29 = qd(9)+OM26;
  OM39 = OM16*S9+OM36*C9;
  OMp29 = qdd(9)+OMp26;
  OMp39 = C9*(OMp36+qd(9)*OM16)+S9*(OMp16-qd(9)*OM36);
  ALPHA29 = ALPHA26+BS56*s.dpt(2,3);
  ALPHA39 = C9*(ALPHA35+BETA86*s.dpt(2,3))+S9*(ALPHA16+BETA26*s.dpt(2,3));
  OM110 = qd(10)+OM16*C9-OM36*S9;
  OM210 = OM29*C10+OM39*S10;
  OM310 = -(OM29*S10-OM39*C10);
  OMp110 = qdd(10)+C9*(OMp16-qd(9)*OM36)-S9*(OMp36+qd(9)*OM16);
  OMp210 = C10*(OMp29+qd(10)*OM39)+S10*(OMp39-qd(10)*OM29);
  OM211 = qd(11)+OM26;
  OM311 = OM16*S11+OM36*C11;
  OMp211 = qdd(11)+OMp26;
  OMp311 = C11*(OMp36+qd(11)*OM16)+S11*(OMp16-qd(11)*OM36);
  ALPHA211 = ALPHA26+BETA46*s.dpt(1,4);
  ALPHA311 = C11*(ALPHA35+BETA76*s.dpt(1,4))+S11*(ALPHA16+BS16*s.dpt(1,4));
  OM112 = qd(12)+OM16*C11-OM36*S11;
  OM212 = OM211*C12+OM311*S12;
  OM312 = -(OM211*S12-OM311*C12);
  OMp112 = qdd(12)+C11*(OMp16-qd(11)*OM36)-S11*(OMp36+qd(11)*OM16);
  OMp212 = C12*(OMp211+qd(12)*OM311)+S12*(OMp311-qd(12)*OM211);
  OM213 = qd(13)+OM26;
  OM313 = OM16*S13+OM36*C13;
  OMp213 = qdd(13)+OMp26;
  OMp313 = C13*(OMp36+qd(13)*OM16)+S13*(OMp16-qd(13)*OM36);
  ALPHA213 = ALPHA26+BS56*s.dpt(2,5);
  ALPHA313 = C13*(ALPHA35+BETA86*s.dpt(2,5))+S13*(ALPHA16+BETA26*s.dpt(2,5));
  OM114 = qd(14)+OM16*C13-OM36*S13;
  OM214 = OM213*C14+OM313*S14;
  OM314 = -(OM213*S14-OM313*C14);
  OMp114 = qdd(14)+C13*(OMp16-qd(13)*OM36)-S13*(OMp36+qd(13)*OM16);
  OMp214 = C14*(OMp213+qd(14)*OM313)+S14*(OMp313-qd(14)*OM213);
 
% Backward Dynamics 

  Fs114 = -(s.frc(1,14)-s.m(14)*(s.l(3,14)*(OMp214+OM114*OM314)+C13*(ALPHA16+BETA26*s.dpt(2,5))-S13*(ALPHA35+BETA86*s.dpt(2,5))));
  Fs214 = -(s.frc(2,14)-s.m(14)*(ALPHA213*C14+ALPHA313*S14-s.l(3,14)*(OMp114-OM214*OM314)));
  Fs314 = -(s.frc(3,14)+s.m(14)*(ALPHA213*S14-ALPHA313*C14+s.l(3,14)*(OM114*OM114+OM214*OM214)));
  Cq114 = -(s.trq(1,14)-s.In(1,14)*OMp114+Fs214*s.l(3,14)+OM214*OM314*(s.In(5,14)-s.In(9,14)));
  Cq214 = -(s.trq(2,14)-s.In(5,14)*OMp214-Fs114*s.l(3,14)-OM114*OM314*(s.In(1,14)-s.In(9,14)));
  Cq314 = -(s.trq(3,14)-s.In(9,14)*(C14*(OMp313-qd(14)*OM213)-S14*(OMp213+qd(14)*OM313))+OM114*OM214*(s.In(1,14)-s.In(5,14)));
  Fq313 = Fs214*S14+Fs314*C14;
  Cq213 = Cq214*C14-Cq314*S14;
  Cq313 = Cq214*S14+Cq314*C14;
  Fs112 = -(s.frc(1,12)-s.m(12)*(s.l(3,12)*(OMp212+OM112*OM312)+C11*(ALPHA16+BS16*s.dpt(1,4))-S11*(ALPHA35+BETA76*s.dpt(1,4))));
  Fs212 = -(s.frc(2,12)-s.m(12)*(ALPHA211*C12+ALPHA311*S12-s.l(3,12)*(OMp112-OM212*OM312)));
  Fs312 = -(s.frc(3,12)+s.m(12)*(ALPHA211*S12-ALPHA311*C12+s.l(3,12)*(OM112*OM112+OM212*OM212)));
  Cq112 = -(s.trq(1,12)-s.In(1,12)*OMp112+Fs212*s.l(3,12)+OM212*OM312*(s.In(5,12)-s.In(9,12)));
  Cq212 = -(s.trq(2,12)-s.In(5,12)*OMp212-Fs112*s.l(3,12)-OM112*OM312*(s.In(1,12)-s.In(9,12)));
  Cq312 = -(s.trq(3,12)-s.In(9,12)*(C12*(OMp311-qd(12)*OM211)-S12*(OMp211+qd(12)*OM311))+OM112*OM212*(s.In(1,12)-s.In(5,12)));
  Fq211 = Fs212*C12-Fs312*S12;
  Fq311 = Fs212*S12+Fs312*C12;
  Cq211 = Cq212*C12-Cq312*S12;
  Cq311 = Cq212*S12+Cq312*C12;
  Fs110 = -(s.frc(1,10)-s.m(10)*(s.l(3,10)*(OMp210+OM110*OM310)+C9*(ALPHA16+BETA26*s.dpt(2,3))-S9*(ALPHA35+BETA86*s.dpt(2,3))));
  Fs210 = -(s.frc(2,10)-s.m(10)*(ALPHA29*C10+ALPHA39*S10-s.l(3,10)*(OMp110-OM210*OM310)));
  Fs310 = -(s.frc(3,10)+s.m(10)*(ALPHA29*S10-ALPHA39*C10+s.l(3,10)*(OM110*OM110+OM210*OM210)));
  Cq110 = -(s.trq(1,10)-s.In(1,10)*OMp110+Fs210*s.l(3,10)+OM210*OM310*(s.In(5,10)-s.In(9,10)));
  Cq210 = -(s.trq(2,10)-s.In(5,10)*OMp210-Fs110*s.l(3,10)-OM110*OM310*(s.In(1,10)-s.In(9,10)));
  Cq310 = -(s.trq(3,10)-s.In(9,10)*(C10*(OMp39-qd(10)*OM29)-S10*(OMp29+qd(10)*OM39))+OM110*OM210*(s.In(1,10)-s.In(5,10)));
  Fq39 = Fs210*S10+Fs310*C10;
  Cq29 = Cq210*C10-Cq310*S10;
  Cq39 = Cq210*S10+Cq310*C10;
  Fs18 = -(s.frc(1,8)-s.m(8)*(s.l(3,8)*(OMp28+OM18*OM38)+C7*(ALPHA16+BS16*s.dpt(1,2))-S7*(ALPHA35+BETA76*s.dpt(1,2))));
  Fs28 = -(s.frc(2,8)-s.m(8)*(ALPHA27*C8+ALPHA37*S8-s.l(3,8)*(OMp18-OM28*OM38)));
  Fs38 = -(s.frc(3,8)+s.m(8)*(ALPHA27*S8-ALPHA37*C8+s.l(3,8)*(OM18*OM18+OM28*OM28)));
  Cq18 = -(s.trq(1,8)-s.In(1,8)*OMp18+Fs28*s.l(3,8)+OM28*OM38*(s.In(5,8)-s.In(9,8)));
  Cq28 = -(s.trq(2,8)-s.In(5,8)*OMp28-Fs18*s.l(3,8)-OM18*OM38*(s.In(1,8)-s.In(9,8)));
  Cq38 = -(s.trq(3,8)-s.In(9,8)*(C8*(OMp37-qd(8)*OM27)-S8*(OMp27+qd(8)*OM37))+OM18*OM28*(s.In(1,8)-s.In(5,8)));
  Fq27 = Fs28*C8-Fs38*S8;
  Fq37 = Fs28*S8+Fs38*C8;
  Cq27 = Cq28*C8-Cq38*S8;
  Cq37 = Cq28*S8+Cq38*C8;
  Fs16 = -(s.frc(1,6)-s.m(6)*(ALPHA16+s.l(3,6)*(BS36+OMp26)));
  Fs26 = -(s.frc(2,6)-s.m(6)*(ALPHA26+s.l(3,6)*(BS66-OMp16)));
  Fq16 = Fs16+Fq311*S11+Fq313*S13+Fq37*S7+Fq39*S9+Fs110*C9+Fs112*C11+Fs114*C13+Fs18*C7;
  Fq26 = Fq211+Fq27+Fs26+Fs210*C10+Fs214*C14-Fs310*S10-Fs314*S14;
  Fq36 = -(s.frc(3,6)-s.m(6)*(ALPHA35-s.l(3,6)*(OM16*OM16+OM26*OM26))-Fq311*C11-Fq313*C13-Fq37*C7-Fq39*C9+Fs110*S9+Fs112*S11+Fs114*S13+Fs18*S7);
  Cq16 = -(s.trq(1,6)-s.In(1,6)*OMp16-s.In(2,6)*OMp26-s.In(3,6)*OMp36-Cq110*C9-Cq112*C11-Cq114*C13-Cq18*C7-Cq311*S11-Cq313*S13-Cq37*S7-Cq39*S9+...
 Fs26*s.l(3,6)-OM26*(s.In(3,6)*OM16+s.In(6,6)*OM26+s.In(9,6)*OM36)+OM36*(s.In(2,6)*OM16+s.In(5,6)*OM26+s.In(6,6)*OM36)-s.dpt(2,3)*(Fq39*C9-Fs110*S9)-...
 s.dpt(2,5)*(Fq313*C13-Fs114*S13));
  Cq26 = -(s.trq(2,6)-Cq211-Cq213-Cq27-Cq29-s.In(2,6)*OMp16-s.In(5,6)*OMp26-s.In(6,6)*OMp36-Fs16*s.l(3,6)+OM16*(s.In(3,6)*OM16+s.In(6,6)*OM26+...
 s.In(9,6)*OM36)-OM36*(s.In(1,6)*OM16+s.In(2,6)*OM26+s.In(3,6)*OM36)+s.dpt(1,2)*(Fq37*C7-Fs18*S7)+s.dpt(1,4)*(Fq311*C11-Fs112*S11));
  Cq36 = -(s.trq(3,6)-s.In(3,6)*OMp16-s.In(6,6)*OMp26-s.In(9,6)*OMp36+Cq110*S9+Cq112*S11+Cq114*S13+Cq18*S7-Cq311*C11-Cq313*C13-Cq37*C7-Cq39*C9-...
 Fq211*s.dpt(1,4)-Fq27*s.dpt(1,2)-OM16*(s.In(2,6)*OM16+s.In(5,6)*OM26+s.In(6,6)*OM36)+OM26*(s.In(1,6)*OM16+s.In(2,6)*OM26+s.In(3,6)*OM36)+s.dpt(2,3)*(...
 Fq39*S9+Fs110*C9)+s.dpt(2,5)*(Fq313*S13+Fs114*C13));
  Fq15 = Fq16*C6-Fq26*S6;
  Fq25 = Fq16*S6+Fq26*C6;
  Cq25 = Cq16*S6+Cq26*C6;
  Fq14 = Fq15*C5+Fq36*S5;
  Fq34 = -(Fq15*S5-Fq36*C5);
  Cq14 = Cq36*S5+C5*(Cq16*C6-Cq26*S6);
  Fq23 = Fq25*C4-Fq34*S4;
  Fq33 = Fq25*S4+Fq34*C4;

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  Qq(1) = Fq14;
  Qq(2) = Fq23;
  Qq(3) = Fq33;
  Qq(4) = Cq14;
  Qq(5) = Cq25;
  Qq(6) = Cq36;
  Qq(7) = Cq27;
  Qq(8) = Cq18;
  Qq(9) = Cq29;
  Qq(10) = Cq110;
  Qq(11) = Cq211;
  Qq(12) = Cq112;
  Qq(13) = Cq213;
  Qq(14) = Cq114;

% ====== END Task 0 ====== 

  

