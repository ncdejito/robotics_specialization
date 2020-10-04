function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

%u = 0; % freefall example from supplementary material
%u = params.mass*params.gravity; %fixed height


% FILL IN YOUR CODE HERE
kp = 150;
kv = 15;

E = s_des - s;
e = E(1);
edot = E(2);

if e > 0
  u = params.mass*(kp*e + kv*edot + params.gravity);
elseif e < 0
  u = 0;
else
  u = params.mass*params.gravity;
end


end

