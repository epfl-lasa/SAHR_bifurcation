%% make DMPs

clear;
close all
make_database

figure; hold on
for iter = 1:11
    plot(trj{iter})
end


for iter = 1:11
    DMP.N = 25;
    DMP = DMP_train(trj{iter}, time, DMP);
    DMP_database{iter} = DMP;
    clear DMP
end



% break
%% track all DMPs and plot them
F = 2;
dt = dt / F;
for iter = 1:11
    TT = [];
    TRJ = [];
    S.x = 1; S.y = DMP_database{iter}.y0; S.z = DMP_database{iter}.dy0*DMP_database{iter}.tau;
    t = 0;
    TRJ = [S.y]; TT = [t]; TRJD = [DMP_database{iter}.dy0];
    % reconstruct the trajectory by Euler integration (F times higer than
    % the trajectory sampling rate)
    
    
    %     tic
    while t < 2*t_end
        for i = 1:F
            S = DMP_integrate(DMP_database{iter}, S, dt);
            t = t + dt;
        end
        TT = [TT; t];
        TRJ = [TRJ; S.y];
    end
    times{iter} = TT;
    trajs{iter} = TRJ;
    iter
end
for iter = 1:11
    plot(trajs{iter},'g--','linewidth',1); hold on
    www{iter} = DMP_database{iter}.w;
end

