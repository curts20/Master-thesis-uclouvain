function[F]=Control_lqr_cluster(mbs_data)
%% Load lqr matrix
global MBS_user MBS_info
load('GoodKT.mat')

if mbs_data.m(6)<Good_KT(MBS_user.KT_index)
    if mbs_data.m(6)<Good_KT(1)
       
    else
        MBS_user.KT_index=MBS_user.KT_index-1;
       
    end
    
    cd C:\Users\migue\Documents\MBProjects\Lunar_Lander_Cluster\userfctR\KT;
    filename=strcat('KT',num2str(Good_KT(MBS_user.KT_index)),'.txt');
    KT=load(filename);
    MBS_user.KT=KT;
elseif MBS_user.step==1
    
    cd C:\Users\migue\Documents\MBProjects\Lunar_Lander_Cluster\userfctR\KT;
    filename=strcat('KT',num2str(Good_KT(MBS_user.KT_index)),'.txt');
    KT=load(filename);
    MBS_user.KT=KT;
end
K=MBS_user.KT;
X_0=zeros(12,1);
if (mbs_data.q(3)>40)
    
    X_0(9)=-5;
elseif (mbs_data.q(3)>=3)
    
    X_0(9)=2;
else
    K=zeros(5,12);
end
X=horzcat(mbs_data.q(1:6)',mbs_data.qd(1:6)');
F=-K*(X'-X_0);

end
