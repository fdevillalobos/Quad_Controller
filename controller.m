function [F, M, trpy, drpy, error2] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega

% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des

% The gains are params.kps = [x_kp, y_kp, z_kp] and params.kds = [x_kd, y_kd, z_kd]

% =================== Your code goes here ===================
persistent err_x err_y err_z error
% Calculate Accumulated Errors.
if(size(err_x,1)==0)
    err_x = 0;
    err_y = 0;
    err_z = 0;
    error = 0;
end
if(t == 0)
    err_x = 0;  err_y = 0;  err_z = 0;
    fprintf('New Loop \n');
end

err_x = err_x + abs(qd{qn}.pos(1) - qd{qn}.pos_des(1));
err_y = err_y + abs(qd{qn}.pos(2) - qd{qn}.pos_des(2));
err_z = err_z + abs(qd{qn}.pos(3) - qd{qn}.pos_des(3));
if(nargout > 4)
    error = err_x + err_y + err_z;
    error2 = error;
end

% Rotation Matrix
roll  = qd{qn}.euler(1);
pitch = qd{qn}.euler(2);
yaw   = qd{qn}.euler(3);
R = [cos(yaw)*cos(pitch) - sin(roll)*sin(yaw)*sin(pitch),-cos(roll)*sin(yaw),cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw);
     cos(pitch)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch) - cos(yaw)*cos(pitch)*sin(roll);
     -cos(roll)*sin(pitch)                              , sin(roll)         , cos(roll) * cos(pitch)];
 %I don't have pitch_dot and roll_dot, so how do I calculate p q r??
 

% Error Control     Better Values: [25 4] [7 4] [15 4];
kp = params.kps(1);     kd = params.kds(1);
r1_2dot = kd * (qd{qn}.vel_des(1) - qd{qn}.vel(1)) + kp * (qd{qn}.pos_des(1) - qd{qn}.pos(1))  + qd{qn}.acc_des(1);
kp = params.kps(2);     kd = params.kds(2);
r2_2dot = kd * (qd{qn}.vel_des(2) - qd{qn}.vel(2)) + kp * (qd{qn}.pos_des(2) - qd{qn}.pos(2))  + qd{qn}.acc_des(2);
kp = params.kps(3);     kd = params.kds(3);
r3_2dot = kd * (qd{qn}.vel_des(3) - qd{qn}.vel(3)) + kp * (qd{qn}.pos_des(3) - qd{qn}.pos(3))  + qd{qn}.acc_des(3);

% Thurst
F    = params.mass * (params.grav + r3_2dot) / R(3,3);


% Desired roll, pitch and yaw
phi_des   = 1/params.grav* (r1_2dot* sin(qd{qn}.yaw_des) - r2_2dot* cos(qd{qn}.yaw_des));
theta_des = 1/params.grav* (r1_2dot* cos(qd{qn}.yaw_des) + r2_2dot* sin(qd{qn}.yaw_des));
psi_des   = qd{qn}.yaw_des;

%[phi_des theta_des psi_des]

% Moment
kp_phi   = 10;     kd_phi   = 1;
kp_theta = 10;     kd_theta = 1;
kp_psi   = 10;     kd_psi   = 1;
p_des    =  0;     q_des    = 0;     r_des = qd{qn}.yawdot_des;


M    = [kp_phi  *(phi_des  -qd{qn}.euler(1)) + kd_phi   * (p_des - qd{qn}.omega(1));
        kp_theta*(theta_des-qd{qn}.euler(2)) + kd_theta * (q_des - qd{qn}.omega(2));
        kp_psi  *(psi_des  -qd{qn}.euler(3)) + kd_psi   * (r_des - qd{qn}.omega(3))];

%M = zeros(3,1);

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
