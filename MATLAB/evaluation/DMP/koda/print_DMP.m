function print_DMP(DMP)
% Displays the values of a DMP
%

if isfield(DMP, 'sigma2')
  disp(['kernel centers and widths: ']); disp([DMP.c', sqrt(DMP.sigma2)']);
end
if isfield(DMP, 'h')
  disp(['kernel centers and widths: ']); disp([DMP.c', sqrt(ones(1, DMP.N) ./ DMP.h)']);
end
disp(['kernel weights: ']); disp(DMP.w);
disp(['Number of basis functions: ', num2str(DMP.N)]);
if isfield(DMP, 'tau')
  disp(['Time constant tau: ', num2str(DMP.tau)]);
end
if isfield(DMP, 'Omega')
  disp(['Periodic DMP frequency: ', num2str(DMP.Omega)]);
end
disp(['DMP constants aplha_z, beta_z, alpha_x: ', num2str([DMP.a_z, DMP.b_z, DMP.a_x])]);
if isfield(DMP, 'tau')
  disp(['Final goal: ', num2str(DMP.goal)]);
else
  disp(['Center of periodic motion: ', num2str(DMP.goal)]);
end
disp(['Initial position: ', num2str(DMP.y0)]);
disp(['Initial velocity: ', num2str(DMP.dy0)]);