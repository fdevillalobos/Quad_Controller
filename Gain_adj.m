clc
clear all;
close all;
x_kp = 10;
y_kp = 12;
z_kp = 12;
x_kd = 11;
y_kd = 11;
z_kd = 9;

min_error = 1675;
% for i = x_kp:2:18
%     for j = y_kp:2:18
%         for k = z_kp:2:18
%             for l = x_kd:1:12
%                 for m = y_kd:1:12
%                     for n = z_kd:1:12
%                         error = runsim_2(i,j,k,l,m,n);
%                         fprintf('kps: %d %d %d.    kds: %d %d %d.    Error: %f16  \n \n', i,j,k,l,m,n, error);
%                         if(error < min_error)
%                             min_error = error;
%                             fprintf('New Minimum Error. error = %f16  \n \n', min_error);
%                             opt_xkp = x_kp;
%                             opt_ykp = y_kp;
%                             opt_zkp = z_kp;
%                             opt_xkd = x_kd;
%                             opt_ykd = y_kd;
%                             opt_zkd = z_kd;
%                         end
%                         close all;
%                     end
%                 end
%             end
%         end
%     end
% end

for i = x_kp:2:18
    for k = z_kp:2:18
        for l = x_kd:1:12
            for n = z_kd:1:12
                error = runsim_2(i,i,k,l,l,n);
                fprintf('kps: %d %d %d.    kds: %d %d %d.    Error: %f16  \n \n', i,i,k,l,l,n, error);
                if(error < min_error)
                    min_error = error;
                    fprintf('New Minimum Error. error = %f16  \n \n', min_error);
                    opt_xkp = x_kp;
                    opt_zkp = z_kp;
                    opt_xkd = x_kd;
                    opt_zkd = z_kd;
                    fprintf('Optimal Values: kps: %d %d %d, and kds: %d %d %d  \n \n', i,i,k,l,l,n);
                end
                close all;
            end
        end
    end
end