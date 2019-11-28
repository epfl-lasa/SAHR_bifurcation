function fx = DMP_forcing_term(DMP, S)
% One integration step for a discrete DMP.
% 
% INPUTS:
% DMP: the structure defining the DMP, as output by DMP_train
% S: current DMP state
%   S.y current state
%   S.z current scaled velocity
%   S.x current phase
%
% OUTPUTS:
% fx: DMP forcing term
%

% Theoretically, fx = 0 at S.x == exp(-alpha_x) for discrete
% movements with zero velocity and acceleration at the end of motion
fx = zeros(1,size(DMP.w,2));

% values of kernel functions at phase S.x
psi = exp(-0.5*(S.x-DMP.c).^2./DMP.sigma2)';
sum_psi = sum(psi);

for i = 1:size(DMP.w,2)
  if S.x >= DMP.c(end)
    % weighted sum of normalized kernel functions
    fx(i) = S.x * sum(DMP.w(:,i) .* psi) / sum_psi;
  elseif ~DMP.is_zero
    fx(i) = S.x * sum(DMP.w(:,i) .* psi) / (sum_psi + eps);
  end
end
