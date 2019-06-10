close all; clc;% Cleaning of the Matlab workspace
%To make it work first you change m_0=1 and then m=m_0+1 to automatically
%restart in the next iteration if there is an error and it can't be
%computed. Good_KT is a vector which saves all the values of mass for which the
%matrix K could be computed (For data treatment pruposes).
global MBS_user MBS_dirdyn;                                                            % Declaration of the global user structure
B=load('B.txt');
% Project loading
prjname = 'Lunar_Lander_Cluster';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs"
mbs_data_ini = mbs_data;
A_Number=20;
m_val=linspace(90000,110000,A_Number); %Values of the mass and number of A matrixes created
m_val=round(m_val);
%Allocation
[N,M]=size(mbs_data.q);
nu=5;   %Number of Engines
nx=6*2; %Number of controlled variables
% 
%  counter=511;
% load('GoodKT.mat')

Names_Archives=dir('C:\Users\migue\Documents\MBProjects\Lunar_Lander_Cluster\userfctR\KT');
m_0=m+1;
for m=m_0:A_Number
    
    
    K=zeros(N,N);
    G=zeros(N,N);
    M_eq = zeros(N,N);
    c_eq = zeros(N,1);
    M = zeros(N,N);
    c = zeros(N,1);
    M_p=zeros(2*N,2*N);
    K_p=zeros(2*N,2*N);
    A_fin=zeros(12,12);
    
    mbs_data.m(6)=m_val(m);
    mbs_data.In(:,6)=mbs_data.In(:,6)*(mbs_data.m(6)/16436.827);
    %Simulation
    [M_eq,c_eq] =mbs_dirdyna_Lunar_Lander_Cluster(mbs_data); %Initial state of M and c
    
    delta_q=1e-5;
    % ¡Aproximation!: k=delta_c/delta_q
    for i=1:6
        mbs_data.q(i)=mbs_data_ini.q(i)+delta_q;
        opt.dirdyn =   {'motion','oneshot','verbose','yes'};
        [mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);
        
        [M,c] =mbs_dirdyna_Lunar_Lander_Cluster(mbs_data); %Initial state of M and c
        K(:,i)=(c-c_eq)./delta_q;
        mbs_data.q(i)=mbs_data_ini.q(i);
    end
    delta_q_p=1e-5;
    for i=1:6
        mbs_data.qd(i)=mbs_data_ini.qd(i)+delta_q;
        opt.dirdyn =   {'motion','oneshot','verbose','yes'};
        [mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);
        
        [M,c] =mbs_dirdyna_Lunar_Lander_Cluster(mbs_data); %Initial state of M and c
        G(:,i)=(c-c_eq)./delta_q;
        mbs_data.qd(i)=mbs_data_ini.qd(i);
    end
    % Computation of the matrix A
    M_p(1:N,N+1:2*N)=M_eq;
    M_p(N+1:2*N,1:N)=M_eq;
    M_p(N+1:2*N,N+1:2*N)=G;
    
    K_p(1:N,1:N)=-M_eq;
    K_p(N+1:2*N,N+1:2*N)=K;
    
    A=-inv(M_p)*K_p;
    A_fin(7:12,7:12)=A(1:6,1:6);
    A_fin(1:6,1:6)=A(15:20,15:20);
    A_fin(7:12,1:6)=A(1:6,15:20);
    A_fin(1:6,7:12)=A(15:20,1:6);
    
    RT=7e0*eye(nu,nu);
    QT=eye(nx,nx);
    QT(4,4) =8e8;
    QT(5,5)=8e8;
    QT(10,10) =5e9;
    QT(11,11) =5e9;
    QT(9,9) =6e9;
    KT= lqr(A_fin,B,QT,RT,0);
    Good_KT(1,counter)=m_val(m); % Values for which i could compute the K
    counter=counter+1;
    mkdir KT
    cd KT
    
    filename=strcat('KT',num2str(m_val(m)),'.txt');
    save(filename,'KT','-ascii')
    
    cd C:\Users\migue\Documents\MBProjects\Lunar_Lander_Cluster\userfctR
    close all
    
    clc
end
save('GoodKT.txt','Good_KT','-ascii');
save('GoodKT.mat','Good_KT');
