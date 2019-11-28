function [y, dy, ddy] = DMP_get_trajectory_state(DMP, S)
% return current position, velocity and accelration defined by DMP state S
% 
% INPUTS
%   DMP ... DMP parameters
%   S   ... current DMP state
%
% OUTPUS
%   y   ... current position
%   yd  ... current velocity 
%   ydd ... current acceleration

y = S.y;

ddy = zeros(1, size(DMP.w,2));
if isfield(DMP, 'tau')
  dy = S.z / DMP.tau;

  fx = DMP_forcing_term(DMP, S);

  for i = 1:size(DMP.w,2)
    ddy(1,i) = 1 / DMP.tau^2 * (DMP.a_z * (DMP.b_z * (DMP.goal(i) - S.y(i)) - S.z(i)) + fx(i));
  end
else
  omega = 2 * pi * DMP.Omega;
  dy = omega * S.z;

  psi = exp(DMP.h .* (cos(S.fi-DMP.c)-1))';
  for i = 1:size(DMP.w,2)
    fx = sum(DMP.w(:,i).*psi)/sum(psi);
    ddy(i) = omega^2 * (DMP.a_z * (DMP.b_z * (DMP.goal(i) - S.y(i)) - S.z(i)) + fx);
  end
end
  