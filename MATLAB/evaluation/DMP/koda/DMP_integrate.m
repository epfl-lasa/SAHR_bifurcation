function S = DMP_integrate(DMP, S, dt)
% One integration step for a discrete DMP.
% 
% INPUTS:
% DMP: the structure defining the DMP, as output by DMP_train
% current DMP state S
%   S.y current state
%   S.z current scaled velocity
%   S.x current phase
% dt: integration step
%
% OUTPUTS:
% updated DMP state S 
%   S.y updated state
%   S.z updated scaled velocity
%   S.x updated phase
%

fx = DMP_forcing_term(DMP, S);

for i = 1:size(DMP.w,2)
  % derivatives
  dz = 1 / DMP.tau * (DMP.a_z * (DMP.b_z * (DMP.goal(i) - S.y(i)) - S.z(i)) + fx(i));
  dy = 1 / DMP.tau * S.z(i);

  %integration
  S.z(i) = S.z(i) + dz*dt;
  S.y(i) = S.y(i) + dy*dt;
end

dx_dt = -DMP.a_x * S.x / DMP.tau;
S.x = S.x + dx_dt*dt;