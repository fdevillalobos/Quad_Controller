function [ is_terminate ] = terminate_check( x, time, stop, pos_tol, vel_tol, time_tol )
%TERMINATE_CHECK Check termination criteria

pos_check = true;
vel_check = true;
nquad = length(stop);

for qn = 1:nquad
    pos_check = pos_check && (norm(x{qn}(1:3) - stop{qn}) < pos_tol);
    vel_check = vel_check && (norm(x{qn}(4:6)) < vel_tol);
end

is_terminate = (pos_check && vel_check) || (time > time_tol);

end