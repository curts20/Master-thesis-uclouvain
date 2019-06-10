function [uxd] = user_Derivatives(ux,mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%[uxd] = user_Derivatives(ux,mbs_data,tsim)
%
% ux : user vector of state variables
% mbs_data : multibody data structure
% tsim : current time
%
% uxd : user vector of state derivatives

global MBS_user MBS_info

uxd = zeros(mbs_data.Nux,1);

%/*-- Begin of user code --*/

error_speed =abs(mbs_data.qd(3)-1);
   uxd(1) = error_speed;
   ux(2)= error_speed;

%/*-- End of user code --*/

return
