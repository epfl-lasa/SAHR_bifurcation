
% This script illustrates the use of periodic DMPs for trajectory representation.
% First, the trajectory is contructed. Then, PDMP_train is called in order
% to train the parameters of the DMP corresponding to the given trajectory.
% Then, the trajectory is reconstracted back from the DMP representation
% using PDMP_integrate. Finally, the training trajectory and the reconstructed
% trajectory are plotted.
% Original version by Andrej Gams, modified by Ilaria Lauzana for
% comparison for the DS from Learning DS with Bifurcations.

% clear

%% construct a trajectory
% dt = 0.002;
% t_end = 22;
% time = linspace(0, t_end, t_end/dt+1);
% 
% trj(:,1) = 0.25 + cos(2*pi*time) + 0.4*sin(4*2*pi*time) + 0.1*sin(6*2*pi*time);
% trj(:,2) = 0.15 + sin(2*pi*time) + 0.2*sin(3*2*pi*time) + 0.05*sin(8*2*pi*time) + 0.15*sin(9*2*pi*time);

%%
for k = 20%[5,8,10,15,20,25]%,30,35]%,50,80]

%% train a DMP
tic;
dt = mean(diff(timeDMP));
PDMP.N = k;
PDMP.Omega = 0.65/2; % set to 0 to train the frequency;
PDMP.a_z = 48;
% PDMP = PDMP_train(trj, dt, PDMP);
PDMP = PDMP_train(trj, timeDMP, PDMP, 0.995, 100, 0);%1);
print_DMP(PDMP)
t_elapsed = toc;
disp(['DMP training time: ', num2str(t_elapsed)]);

%% integrate a DMP
% initial DMP state for integration
S.fi = 0; S.y = PDMP.y0; 
S.z = PDMP.dy0 / ((2*pi) * PDMP.Omega);
% set integration parameters
F = 1;
dt = dt / F;
tt = 0;
TRJ = [S.y]; T = [tt];
tic;
% reconstruct the trajectory back from the periodic DMP
while tt < timeDMP(end)
  for i = 1:F
    S = PDMP_integrate(PDMP, S, dt);
    tt = tt + dt;
  end
  TRJ = [TRJ; S.y];
  T = [T, tt];
end
t_elapsed = toc;
disp(['DMP integration time: ', num2str(t_elapsed)]);

%% display results
figure
hold off, plot(timeDMP, trj, ':', 'LineWidth', 2), hold on, plot(T, TRJ)
title(strcat('periodic DMP training and reproduction - N = ', num2str(k)))
% legend('original DOF1', 'original DOF2', 'reconstructed DOF1','reconstructed DOF2')
legend('original DOF1', 'original DOF2', 'original DOF3', 'reconstructed DOF1','reconstructed DOF2','reconstructed DOF3')

end