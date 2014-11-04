function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle
% This is parametrized to that x,y respond to a z configuration.
% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

if(qn == 1)
    %Helix:
    r = 5;                     %meters
    loop_time = 7;             %Time to complete Helix
    H = 2.5;                   %Final Height of Helix
    
    if(t > loop_time)
        x = 5;  y = 0;  z = H;
        x_d =  0;     x_dd = 0;
        y_d =  0;     y_dd = 0;
        z_d =  0;     z_dd = 0;
    else
        A = 2*pi / H;
        C = pi() * (t/loop_time - 1/2);
        B = pi()/loop_time;
        
        
        z = H/2 + H/2* sin(C);
        x = r * cos(A*z);
        y = r * sin(A*z);
        
        x_d  = -r * A * H/2 * B * sin(A*z) * cos(C);
        y_d  =  r * A * H/2 * B * cos(A*z) * cos(C);
        z_d  =  H/2 * B * cos(C);
        
        x_dd = -r * A * H/2 * B^2 * (A * H/2 * cos(A*z) * (cos(C))^2 - sin(A*z) * sin(C));
        y_dd = -r * A * H/2 * B^2 * (A * H/2 * sin(A*z) * (cos(C))^2 + cos(A*z) * sin(C));
        z_dd = -H/2 * B^2 * sin(C);
        
    end
    
end

pos = [x; y; z];
vel = [x_d; y_d; z_d];
acc = [x_dd; y_dd; z_dd];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
