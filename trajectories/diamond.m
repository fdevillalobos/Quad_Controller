function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

Point_1 = [0,0,0];
Point_2 = [0.25,  sqrt(2),   sqrt(2)];
Point_3 = [0.50, 0       , 2*sqrt(2)];
Point_4 = [0.75, -sqrt(2),   sqrt(2)];
Point_5 = [1   ,        0,         0];
vel0    = [0,0,0];
acc0    = [0,0,0];

loop_time = 7;

if(t <= loop_time/4)
    [pos, vel, acc] = quintic(0, loop_time/4, [Point_1;vel0;acc0], [Point_2;vel0;acc0], t);
elseif(t > loop_time/4 && t <= loop_time/2)
    [pos, vel, acc] = quintic(loop_time/4, loop_time/2, [Point_2;vel0;acc0], [Point_3;vel0;acc0], t);
elseif(t > loop_time/2 && t <= 3*loop_time/4)
    [pos, vel, acc] = quintic(loop_time/2, 3*loop_time/4, [Point_3;vel0;acc0], [Point_4;vel0;acc0], t);
elseif(t > 3*loop_time/4 && t <= loop_time)
    [pos, vel, acc] = quintic(3*loop_time/4,   loop_time, [Point_4;vel0;acc0], [Point_5;vel0;acc0], t);
else
    pos = Point_5;
    vel = vel0;
    acc = acc0;
end
       
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end


function [pos, vel, acc] = quintic(t0, tf, Param0, Param1, t)
   
   A = [1  t0  t0^2    t0^3     t0^4      t0^5;
        0   1  2*t0  3*t0^2   4*t0^3    5*t0^4;
        0   0   2    6*t0    12*t0^2   20*t0^3;
        1  tf  tf^2    tf^3     tf^4      tf^5;
        0   1  2*tf  3*tf^2   4*tf^3    5*tf^4;
        0   0   2    6*tf    12*tf^2   20*tf^3];
    
   B = [Param0; Param1];
   
   x = A\B;
   
   pos = (x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 + x(5,:)*t^4 + x(6,:)*t^5)';
   vel =      (x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + 4*x(5,:)*t^3 + 5*x(6,:)*t^4)';
   acc =                 (2*x(3,:) + 6*x(4,:)*t + 12*x(5,:)*t^2 + 20*x(6,:)*t^3)';
   
end
