function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

[kp_1,kp_2, kp_3] = deal(50,50,50);
[kd_1, kd_2, kd_3] = deal(5,5,5);
[kp_phi, kp_theta, kp_psi] = deal(500,500,500);
[kd_phi, kd_theta, kd_psi] = deal(15,15,15);

r_ddot_1_des = des_state.acc(1) + kd_1*(des_state.vel(1)-state.vel(1)) + kp_1*(des_state.pos(1)-state.pos(1));
r_ddot_2_des = des_state.acc(2) + kd_2*(des_state.vel(2)-state.vel(2)) + kp_2*(des_state.pos(2)-state.pos(2));
r_ddot_3_des = des_state.acc(3) + kd_3*(des_state.vel(3)-state.vel(3)) + kp_3*(des_state.pos(3)-state.pos(3));

phi_des = (r_ddot_1_des*sin(state.rot(3))-r_ddot_2_des*cos(state.rot(3)))/params.gravity;
theta_des = (r_ddot_1_des*cos(state.rot(3))+r_ddot_2_des*sin(state.rot(3)))/params.gravity;
psi_des = state.rot(3);

% Thrust
F = params.mass*(params.gravity + r_ddot_3_des);
% F = 0;

% Moment
M = params.I*[...
    kp_phi*(phi_des-state.rot(1))+kd_phi*(0-state.omega(1));...
    kp_theta*(theta_des-state.rot(2))+kd_theta*(0-state.omega(2));...
    kp_psi*(psi_des-state.rot(3))+kd_psi*(des_state.yawdot-state.omega(3));...
];
% M = zeros(3,1);

% =================== Your code ends here ===================

end
