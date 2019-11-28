
% This script illustrates the use of discrete DMPs for trajectory representation.
% First, the trajectory is contructed. Then, DMP_train is called in order
% to train the parameters of the DMP corresponding to the given trajectory.
% Then, the trajectory is reconstracted back from the DMP representation
% using DMP_integrate. Finally, the original and the reconstructed trajectory
% are plotted.

clear

%% construct a trajectory
dt = 0.002;
t_end = 2;
time = linspace(0, t_end, t_end / dt + 1);
trj(:,1) = -cos(2*pi*time);
trj(:,2) = cos(3*pi*time);

%% train a DMP
tic
DMP.N = 40;
DMP = DMP_train(trj, time, DMP);
% DMP = DMP_train(trj, dt);
print_DMP(DMP);
t_elapsed = toc;
disp(['DMP training time: ', num2str(t_elapsed)]);

%% integrate a DMP
% initialize the DMP state
S.x = 1; S.y = DMP.y0; S.z = DMP.dy0*DMP.tau;
t = 0;
TRJ = [S.y]; TT = [t]; TRJD = [DMP.dy0];
% reconstruct the trajectory by Euler integration (F times higer than
% the trajectory sampling rate)
F = 2;
dt = dt / F;
tic
while t < 2*t_end
  for i = 1:F
    S = DMP_integrate(DMP, S, dt);
    t = t + dt;
  end
  TT = [TT; t];
  TRJ = [TRJ; S.y];
end
t_elapsed = toc;
disp(['DMP integration time: ', num2str(t_elapsed)]);

%% display results
figure(1),
hold off, plot(time, trj,'k:','LineWidth',2), hold on, plot(TT, TRJ, 'r')
title('discrete DMP training and integration')
legend('original DOF1', 'original DOF2', 'reconstructed DOF1','reconstructed DOF2')
