function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% u1 = 0;
% u2 = 0;

% FILL IN YOUR CODE HERE

% using ziegler-nichols method
% https://www.coursera.org/learn/robotics-flight/discussions/weeks/3/threads/1EAZ98OHEeWqYApk6o1_2Q
% Kp=0.8*Ku
% Kd=(Tu/8)*Kp

% % line
% kpy = 0.8*13;
% kvy = (0.6/8)*kpy;
% kpz = 0.8*500;
% kvz = (0.2857/8)*kpz;
% kpphi = 0.8*2000;
% kvphi = (0.133/8)*kpphi;

% sine
kpy = 0.8*12.5;
kvy = (12/8)*kpy;
kpz = 0.8*500;
kvz = (0.2857/8)*kpz;
kpphi = 0.8*2000;
kvphi = (0.133/8)*kpphi;

phi_c = -1/params.gravity * (des_state.acc(1) + kvy*(des_state.vel(1)-state.vel(1)) + kpy*(des_state.pos(1) - state.pos(1)));
u1 = params.mass*(params.gravity + des_state.acc(2) + kvz*(des_state.vel(2) - state.vel(2)) + kpz*(des_state.pos(2) - state.pos(2)));
u2 = params.Ixx*(kvphi*(-state.omega(1)) + kpphi*(phi_c - state.rot(1)));

end

