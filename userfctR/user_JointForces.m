function [Qq] = user_JointForces(mbs_data,tsim);
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%[Qq] = user_JointForces(mbs_data,tsim);
%
% mbs_data : multibody data structure
% tsim : current time
%
% Qq : joint generalized force/torque (for all joints)
% Qq(i) : joint force/torque in joint (i) along its joint axis
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

Qq = mbs_data.Qq;
% 
% %/*-- Begin of user code --*/
 K = mbs_data.user_model.Spring.K;
 C  = mbs_data.user_model.Spring.C;
 L0    = mbs_data.user_model.Spring.z0;
for i=7 : 10
Qq(i) = - ( K*(mbs_data.q(i)-L0) + C*mbs_data.qd(i) );
Qq(i+4) = - ( K*(mbs_data.q(i+4)+L0) + C*mbs_data.qd(i+4) );
 end
%/*-- End of user code --*/

return
