function [] = Fuel_Consumption_Cl(mbs_data)
%% This function computes the fuel comsumption.
% - I haven't taken into account the different fuel/oxidizer consumptions of the
% different engines, in order to be able to compare the two configurations.
% - I have taken as valid the fuel consumprions of the Apollo Descent engine and i have
% made a linearalization of the fuel consumption (Depending of the % of throttle)
% as well as a linearalization f the size/mass of the fuel tanks. 
% - I consider that the fuel consumption doesn't change the CoG

global MBS_user MBS_info MBS_data

    MBS_user.Tank_i_fuel=0.2073*MBS_user.Init_Mass; %%Relations of the size of the tanks given by the apollo
    MBS_user.Tank_ox=0.3299*MBS_user.Init_Mass;

Thrust(1)=MBS_user.Thrust1(MBS_user.step);
Thrust(2)=MBS_user.Thrust2(MBS_user.step);
Thrust(3)=MBS_user.Thrust3(MBS_user.step);
Thrust(4)=MBS_user.Thrust4(MBS_user.step);
Thrust(5)=MBS_user.Thrust5(MBS_user.step);

if MBS_user.max_thrust==425
    %EAM
    Throttle=Thrust/425;
    
elseif (MBS_user.max_thrust==2452)
    % AVUM RD 843
     Throttle=Thrust/2452;
elseif ( MBS_user.max_thrust==4000)
    % R 40 B
    Throttle=Thrust/4000;
elseif (MBS_user.max_thrust==300000/5)
        Throttle=Thrust/(300000/5);
else
    %RS41
     Throttle=Thrust/11100;
end
    %% Fuel comsumption

   Mean_Throttle=sum(Throttle)/5;
       % MBS_user.stepL is the size of the step used
        Fuel_comsumption=8.7279*MBS_user.stepL*Mean_Throttle*(MBS_user.max_thrust/11100); %In kg/s the Fuel consumption full throttle
                                % If it consuumes 17510 pounds in 910s (DATA)
        Ox_comsumption=MBS_user.Tank_ox/MBS_user.Tank_i_fuel*Fuel_comsumption;  %correlation of total oxidizer and time of usable engine
        MBS_data.m(6)=MBS_data.m(6)-(Ox_comsumption+Fuel_comsumption); %Take out the mass consumed
        MBS_data.In(:,6)=MBS_data.In(:,6)*(MBS_data.m(6)/MBS_user.Init_Mass);  %Modify the inertia matrix accordingly
        
        if mbs_data.m(6)<(1-0.5372)*MBS_user.Init_Mass    %If there is no more fuel 
           MBS_user.Fuel=0; 
           disp('**********************Empty tanks*************************')
           
        end
  

end