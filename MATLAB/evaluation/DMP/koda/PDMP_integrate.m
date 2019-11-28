function S = PDMP_integrate(DMP, S, dt)
% One integration step for a periodic DMP.
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

% values of kernel functions at phase S.fi
psi = exp(DMP.h .* (cos(S.fi-DMP.c)-1))';

for i = 1:size(DMP.w,2)

  % weighted sum of normalized kernel functions
  fx = sum(DMP.w(:,i).*psi)/sum(psi);

  % derivatives
  dz = (2*pi) * DMP.Omega * (DMP.a_z * (DMP.b_z * (DMP.goal(i) - S.y(i)) - S.z(i)) + fx);
  dy = (2*pi) * DMP.Omega * S.z(i);

  % Euler integration
  S.z(i) = S.z(i) + dz*dt;
  S.y(i) = S.y(i) + dy*dt;
end;

S.fi = S.fi + (2*pi) * DMP.Omega * dt;