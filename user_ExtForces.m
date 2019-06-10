function Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)
%
% PxF(3,1) : absolute position vector of the external force application point 
% RxF(3,3) : absolute rotation matrix of the body
% VxF(3,1) : absolute velocity vector of the external force application point 
% OMxF(3,1) : absolute angular velocity vector of the body
% AxF(3,1) : absolute acceleration vector of the external force application point 
% OMPxF(3,1) : absolute angular acceleration vector of the body
%
% => All above vectors are expressed in the inertial reference frame !
%
% mbs_data : multibody data structure
% tsim : current time
% ixF : index of the external force sensor ('F' type in MBsysPad)
%        (can be obtained via the 'mbs_get_F_sensor_id' function)
%
% Swr(9,1) = [Fx; Fy; Fz; Mx; My; Mz; dxF];
%   - Force components (expressed in the inertial frame) : Fx, Fy, Fz
%   - Pure torque components (expressed in the inertial frame) : Mx, My, Mz
%   - Application point local coordinates vector (expressed in the body-fixed frame) : dxF(1:3,1);
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Fx=0.0; Fy=0.0; Fz=0.0;
Mx=0.0; My=0.0; Mz=0.0;
idpt = mbs_data.xfidpt(ixF);
dxF = mbs_data.dpt(:,idpt);

%/*-- Begin of user code --*/
% 
% Use the 'mbs_get_F_sensor_id' function to get easily the force sensor
% indices, e.g. :
[F1,F2,F3,F4]  = mbs_get_F_sensor_id(MBS_info,'Ground_F1','Ground_F2','Ground_F3','Ground_F4');
[C5,C1,C2,C3,C4]  = mbs_get_F_sensor_id(MBS_info,'Cluster_C','Cluster_1','Cluster_2','Cluster_3','Cluster_4');
%[F2,F3] = mbs_get_F_sensor_id(MBS_info,'myFsensor_2','myFsensor_3');
%
K_g=200000;  %Ground Parameters
D_g=K_g/50;
%% Calculation of the control
Fu=zeros(1,5);
[Fu]=Control_lqr_cluster(mbs_data);
% Fu(5)=0;
for i=1:5
    if Fu(i)>MBS_user.max_thrust
       Fu(i)=MBS_user.max_thrust;
    end
    if Fu(i)<0
       Fu(i)=0;
    end
end

% % --------------Try to do a pulse control
% Fu=zeros(1,5);
% for i=1:5
%     if Fc(i)>0.01
%      Fu(i)=10000; 
%      MBS_user.t(i)=tsim;
%      MBS_user.State(i)=1;
%     elseif (tsim-MBS_user.t(i))>0.1
%      Fu(i)=0; 
%      MBS_user.State(i)=0;
%     else
%          Fu(i)=10000; 
%          MBS_user.State(i)=1;
%          
%     end
% end
%% Implementation 
switch(ixF)
  
     
        case {F1,F2,F3,F4}
       gap=0;
     
            if (PxF(3)<gap)
                Fz=- K_g*(PxF(3))-D_g*VxF(3);
               
                nu=0.4;
                N=Fz;
                Beta = 39;
                Fx=-nu*N*atan(Beta* VxF(1));
                Fy =-nu*N*atan(Beta* VxF(2)); 
                Fi =[Fx ; Fy ; Fz];
            else
                Fz =0;
                Fx =0;
                Fy =0;
                   Fi =[Fx ; Fy ; Fz];
            end
            
    case C1
% %         LQR
Fz=Fu(1);
       F=[Fx ; Fy ; Fz]; % force vector in the local frame
 Fi = RxF'*F;
        
MBS_user.Thrust1(MBS_user.step)=Fz;

    case C2
        Fz=Fu(2);
        F=[Fx ; Fy ; Fz]; % force vector in the local frame
 Fi = RxF'*F;
        

MBS_user.Thrust2(MBS_user.step)=Fz;
    case C3
        Fz=Fu(3);
        F=[Fx ; Fy ; Fz]; % force vector in the local frame
 Fi = RxF'*F;
        

MBS_user.Thrust3(MBS_user.step)=Fz;
    case C4
        Fz=Fu(4);
        F=[Fx ; Fy ; Fz]; % force vector in the local frame
 Fi = RxF'*F;
        MBS_user.Thrust4(MBS_user.step)=Fz;
    case C5
        Fz=Fu(5);
        F=[Fx ; Fy ; Fz]; % force vector in the local frame
 Fi = RxF'*F;
             
        MBS_user.Thrust5(MBS_user.step)=Fz;  
       

end
Swr= [Fi(1); Fi(2); Fi(3); Mx; My; Mz; dxF];
MBS_user.Tank_fuel(MBS_user.step)=mbs_data.m(6)*(1-0.5372)/((1-0.5372)*MBS_user.Init_Mass)*100;
return
