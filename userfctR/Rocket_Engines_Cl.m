function [Max_Thrust,mass] = Rocket_Engines_Cl(mbs_data)
%% Function which determines which engine is used
% -depends on the total mass, thus the total thrust required.
% -I've set 40% margin aprox to lift the lander, so the thrusters have 1.4
% capacity to lift its weight (T>1.4*Weight)
% All the thrust units are [N]


global MBS_user MBS_info 
mass=mbs_data.m(6);


if ( mass>=30000) %For a big mass imaginary engine
    Max_Thrust= 300000/5;
    T_m= 30;
elseif (30000>mass) &&( mass>=10000)
    %RS41
    Max_Thrust= 11100 ;
    T_m=16.43;
elseif (10000>mass) &&( mass>=4000)
    % RB 40 B
    Max_Thrust= 4000;
    T_m=30;
elseif (4000>mass)&&(mass>=900)
    % AVUM
    Max_Thrust= 2452;
    T_m=15.71;
else
    %EAM
    Max_Thrust= 425;
    T_m=10.44;
end
mass=mbs_data.m(6)+5*Max_Thrust/T_m;
end