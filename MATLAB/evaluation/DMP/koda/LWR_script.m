%script generalize LWR
% vnesi query xd med -1 in 3


% helpful flag
if already_exec == 1;
else
    figure; hold on
    for iter = 1:11
        plot(trj{iter})
    end
    F = 2;
    dt = dt / F;
    already_exec =1;
end

s = 1.55;
DMP_param.N= 25;
DMP_param.a_x= 2;
DMP_param.a_z= 48;
DMP_param.is_zero = 1;

% xt = 1.25;

DMP_param.b_z= DMP_param.a_z/4
[DMP_param time] = DMP_generalize(qPath, sp, xt, DMP_param, s, h, cc);


%% integrate a DMP
% initialize the DMP state
DMP_param.y0 = qPath{iter}(1,2);
DMP_param.dy0 = 0;
S.x = 1; S.y = DMP_param.y0; S.z = DMP_param.dy0*DMP_param.tau;
t = 0;
TRJ = [S.y]; TT = [t]; TRJD = [DMP_param.dy0];
% reconstruct the trajectory by Euler integration (F times higer than
% the trajectory sampling rate)

% tic
while t < 2*t_end
    for i = 1:F
        S = DMP_integrate(DMP_param, S, dt);
        t = t + dt;
    end
    TT = [TT; t];
    TRJ = [TRJ; S.y];
end

plot(TRJ,'g--','linewidth',3)
plot(TRJ,'g','linewidth',1)
grid on

plot(ones(11,1)*1000,goals,'r.','markersize',20)
plot(1000,xt,'b.','markersize',20)

plot([1100,1100],[xt+s,xt-s],'k')
return
disp('here')
% break

