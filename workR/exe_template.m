%--------------------------------------------------------------------------
%   Universite catholique de Louvain
%   CEREM : Centre for research in mechatronics
%   http://www.robotran.be  
%   Contact : robotran@prm.ucl.ac.be
%   Version : ROBOTRAN $Version$
%
%   MBsysLab main script template:
%      - featuring default options
%      - to be adapted by the user
%
%   Project : Lunar_Lander
%   Author : Miguel Ángel Cortés
%   Date : $Date$ 
%--------------------------------------------------------------------------

%% 1. Initialization and Project Loading [mbs_load]
%--------------------------------------------------------------------------
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace
global MBS_user;                                                            % Declaration of the global user structure
MBS_user.process = '';                                                      % Initialisation of the user field "process"

% Project loading
prjname = 'Lunar_Lander_Cluster';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs" 
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure
 % Creation of the result folder                                                                          % Have a look at the content of the mbs_data structure on www.robotran.be
 mkdir C:\Users\migue\Documents\MBProjects\Lunar_Lander_Cluster\Result_Cluster
% 2. Coordinate partitioning [mbs_exe_part]                                % For constrained MBS only
%--------------------------------------------------------------------------
MBS_user.process = 'part';

% qu_id = mbs_get_joint_id(mbs_info,{'joint1' 'joint4' 'joint6'});            % Get joint indices from joint names
% mbs_data = mbs_set_qu(mbs_data,qu_id);                                      % Set variables [qu_id] as independent 
% 
% qv_id = mbs_get_joint_id(mbs_info,{'joint3' 'joint5'});                     % Get joint indices from joint names
% mbs_data = mbs_set_qv(mbs_data,qv_id);                                      % Set variables [qv_id] as dependent
% 
% qc_id = mbs_get_joint_id(mbs_info,{'joint2'});                              % Get joint indices from joint names
% mbs_data = mbs_set_qc(mbs_data,qc_id);                                      % Set variables [qc_id] as driven

opt.part = {'rowperm','yes','threshold',1e-9,'verbose','yes'};
% other options : 'visualize', 'clearmbsglobal'                             % Help about options on www.robotran.be

[mbs_part,mbs_data] = mbs_exe_part(mbs_data,opt.part);                      % Coordinate partitioning process

% Coordinate partitioning results
disp('Coordinate partitioning results');
disp(['Sorted independent variables = ', mat2str(mbs_part.ind_u)]);
disp(['Permutated dependent variables = ', mat2str(mbs_part.ind_v)]);
disp(['Permutated independent constraints = ', mat2str(mbs_part.hu)]);
disp(['Redundant constraints = ', mat2str(mbs_part.hv)]);


%% 5. Direct dynamics [mbs_exe_dirdyn]
%--------------------------------------------------------------------------
N=10;
mass_All=linspace(100000,100000,N);
for m=1:1
 mbs_data=mbs_data_ini
    m
 mbs_data.m(6)=mass_All(m);
 MBS_user.Init_Mass_No_engine  = mbs_data.m(6);   
mbs_data.In(:,6)=mbs_data.In(:,6)*(mbs_data.m(6)/16436.827);
MBS_user.iter=m;
MBS_user.step=1;
MBS_user.process = 'dirdyn';
MBS_user.Fuel=1;    %Condition of end of fuel
MBS_user.stepL=0.01;
MBS_user.t=zeros(1,5);
[Max_Thrust,mass]=Rocket_Engines_Cl(mbs_data);
MBS_user.max_thrust=Max_Thrust;
mbs_data.m(6)=mass;
 MBS_user.Init_Mass  = mbs_data.m(6);  
 mbs_data = mbs_set_qu(mbs_data,mbs_data_ini.qu);                            % Retrieving of the initial set of independent variables

%Calculo de coeficiente inicial para control
load('GoodKT.mat')

[minimo,position]=min(abs(Good_KT-MBS_user.Init_Mass));
MBS_user.KT_index=position;

opt.dirdyn = {'time',0:MBS_user.stepL:45,'motion','simulation',...
    'odemethod','ode45','save2file','yes','framerate',1000,...
    'renamefile','no','verbose','yes'};
% other options : 'visualize', 'save2file', 'depinteg', 'dtmax', 'dtinit',
%                 'reltol', 'abstol', 'clearmbsglobal'                      % Help about options on www.robotran.be

[mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);                % Direct dynamics process (time simulation)



%% 

% Graphical Results
%  figure;
%  plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,3));
%  figure;
% plot(mbs_dirdyn.tsim,mbs_dirdyn.qd(:,3));
% title('speed')
% % figure;
% % plot(mbs_dirdyn.tsim,mbs_dirdyn.qdd(:,3));
% % figure;
% figure;
% % 
% plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,5));
% 
% title('Rotation')
% figure;
% plot(mbs_dirdyn.tsim,mbs_dirdyn.qd(:,4));
% plot(mbs_dirdyn.tsim,mbs_dirdyn.qd(:,5));
% title('Rotation speed')
% % 
% plot(mbs_dirdyn.tsim,mbs_dirdyn.qd(:,5));
% title('Rotation speed')
% figure;
% 
% plot(mbs_dirdyn.tsim,mbs_dirdyn.qdd(:,5));
% title('Rotation acc')
MBS_user.Thrust=[MBS_user.Thrust1;MBS_user.Thrust2;MBS_user.Thrust3;MBS_user.Thrust4;MBS_user.Thrust5];

 figure;
plot(mbs_dirdyn.tsim,MBS_user.Thrust,'LineWidth',3);
title('Thrust-Time')
 grid on
 xlabel('Time [s]')
ylabel('Thrust [N]');
%Human data
figure;
plot(mbs_dirdyn.tsim,mbs_dirdyn.qd(:,3),'LineWidth',3);
title('Velocity')
 grid on
 xlabel('Time [s]')
ylabel('Velocity [m/s]')
% figure;
% plot(mbs_dirdyn.tsim,mbs_dirdyn.qdd(:,3));
% figure;
figure;

plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,3),'LineWidth',3);
title('Z position ')
 grid on
 xlabel('Time [s]')
ylabel('Altitude [m]')
figure;
plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,5),'LineWidth',3);
title('Pitch Rotation')
 grid on
 xlabel('Time [s]')
ylabel('Pitch Rotation [rad]')
figure;
plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,4),'LineWidth',3);
title('Roll Rotation')
 grid on
 xlabel('Time [s]')
ylabel('Roll Rotation [rad]')


%% Data output
   
    cd C:\Users\migue\Documents\MBProjects\Lunar_Lander_Cluster\Result_Cluster
    
    filename=strcat('Mass_',num2str(MBS_user.Init_Mass_No_engine),'.xlsx');
    filename2=strcat('USER_',num2str(MBS_user.Init_Mass_No_engine),'.mat');
  xlswrite(filename,mbs_dirdyn.q,'Postion')
   xlswrite(filename,mbs_dirdyn.qd,'Velocity')
    xlswrite(filename,MBS_user.Tank_fuel,'Tank fuel')
    xlswrite(filename,MBS_user.Thrust,'Thrust')
     xlswrite(filename,MBS_user.Human./9.81,'Human Vert Accel G')
    
    save(filename2,'MBS_user')
    cd C:\Users\migue\Documents\MBProjects\Lunar_Lander_Cluster\workR
    
end
%% 8. Closing operations (optional)
%--------------------------------------------------------------------------
mbs_rm_allprjpath;                                                          % Cleaning of the Matlab project paths
%mbs_del_glob('MBS_user','MBS_info','MBS_data');                             % Cleaning of the global MBS variables
