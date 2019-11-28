new_DMP = DMP_database{1};
new_DMP.w = nwW(1:end-1)';
new_DMP.goal = nwQ;

TRJ = [];
TT = [];

S.x = 1; S.y = new_DMP.y0; S.z = nwW(end)*new_DMP.tau;
t = 0;
TRJ = [S.y]; TT = [t]; TRJD = [new_DMP.dy0];
% reconstruct the trajectory by Euler integration (F times higer than
% the trajectory sampling rate)

% tic
while t < 2*t_end
    for i = 1:F
        S = DMP_integrate(new_DMP, S, dt);
        t = t + dt;
    end
    TT = [TT; t];
    TRJ = [TRJ; S.y];
end

plot(TRJ,'k--','linewidth',3)
plot(TRJ,'k','linewidth',1)