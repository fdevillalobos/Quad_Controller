function [desired_state] = diam2(t, qn)

persistent spline splined splinedd %spliney splineyd splineydd splinez splinezd splinezdd

loop_time = 10;

if (t<=0 || t == inf)
    Point_1 = [0,0,0];
    Point_2 = [0.25,  sqrt(2),   sqrt(2)];
    Point_3 = [0.50, 0       , 2*sqrt(2)];
    Point_4 = [0.75, -sqrt(2),   sqrt(2)];
    Point_5 = [1   ,        0,         0];
    
    P_x = [Point_1(1); Point_2(1); Point_3(1); Point_4(1); Point_5(1)];
    P_y = [Point_1(2); Point_2(2); Point_3(2); Point_4(2); Point_5(2)];
    P_z = [Point_1(3); Point_2(3); Point_3(3); Point_4(3); Point_5(3)];
    
    pad = 10;
    Path_x = zeros(size(P_x,1)+ 2*pad,1);
    Path_y = zeros(size(P_y,1)+ 2*pad,1);
    Path_z = zeros(size(P_z,1)+ 2*pad,1);
    
    Path_x(1:pad) = Point_1(1);     Path_x(pad+1:pad+size(P_x,1)) = P_x;      Path_x(pad+size(P_x,1)+1:end) = Point_5(1);
    Path_y(1:pad) = Point_1(2);     Path_y(pad+1:pad+size(P_y,1)) = P_y;      Path_y(pad+size(P_y,1)+1:end) = Point_5(2);
    Path_z(1:pad) = Point_1(1);     Path_z(pad+1:pad+size(P_z,1)) = P_z;      Path_z(pad+size(P_z,1)+1:end) = Point_5(3);
    
    step = loop_time / (size(P_x,1)-1);
    pad_time = pad*step;
    %scale = loop_time / (size(Path_x,1) - 1);
    %time  = (1:size(Path_x,1))';
    time  = (-pad_time:step:pad_time+step*(size(P_x,1)-1))';
    
    Path = [Path_x Path_y Path_z]';
    spline  = spapi(5,time,Path);
    splined  = fnder(spline);
    splinedd = fnder(splined);
    
end

pos = fnval(spline,  t);
vel = fnval(splined, t);
acc = fnval(splinedd,t);

% pos(2,1) = fnval(spliney,  t);
% vel(2,1) = fnval(splineyd, t);
% acc(2,1) = fnval(splineydd,t);
%
% pos(3,1) = fnval(splinez,  t);
% vel(3,1) = fnval(splinezd, t);
% acc(3,1) = fnval(splinezdd,t);

if(t>=loop_time)
    Point_1 = [0,0,0];
    Point_2 = [0.25,  sqrt(2),   sqrt(2)];
    Point_3 = [0.50, 0       , 2*sqrt(2)];
    Point_4 = [0.75, -sqrt(2),   sqrt(2)];
    Point_5 = [1   ,        0,         0];
    zero    = [0; 0; 0];
    pos = Point_5';
    vel = zero;
    acc = zero;
end

% figure(1); subplot(2,2,1);
% fnplt(splinex); hold on, plot(Points(:,1),Points(:,2),'o'), grid on;
% 
% subplot(2,2,2);
% fnplt(spliney); grid on;
% 
% subplot(2,2,3);
% fnplt(splinez); grid on;

% subplot(2,2,2);
% fnplt(splined); grid on;
% subplot(2,2,3);
% fnplt(splinedd); grid on;

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end