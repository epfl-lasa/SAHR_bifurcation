function [DMP_param time] = DMP_generalize(qPath, sp, xd, DMP_param, s, h, cl)
%
% INPUT PARAMETERS:
%   qPath: example trajectories
%   sp: spline approximated reaching postures and timings
%   xd: target reaching position
%   s:  interval for waiting kernel
%   h:  optimized width oh kernel functions
%   cl: location of kernel centers (1*N)

% Dimension of the joint space
n = numel(sp)-1;

DMP_param.goal = zeros(1,n);
DMP_param.w=[];

A = []; f = [];
tic;
for i = 1:n
%     DMP_param.goal(i) = fnval(sp{i}, xd);
    DMP_param.goal(i) = xd;
    time = fnval(sp{end}, xd);
    
    DMP_param.tau = time;
    epsilon = 1.0e-8;
    
    % c_lin = linspace(0, 1.0, DMP_param.N);
    DMP_param.c(i,:) = exp(-DMP_param.a_x * cl(i,:));
    DMP_param.sigma2(i,1:length(cl)-1) = (diff(DMP_param.c(i,:))*h).^2;
    % sigma2=(diff(c)*0.5).^2; 0.5 doloca sirino gausouke - vec kot jih je,
    % ozje morajo biti
    
    DMP_param.sigma2(i,length(cl)) = DMP_param.sigma2(i,(length(cl)-1));
    
    k = 0;
    A = []; f = [];
    
    for j = 1:max(size(sp{end}.breaks))
        
        weights = distance(xd,sp{end}.breaks(:,j),s);
        
        if weights > 1.0e-5
            
            tau = qPath{j}(end,1);
            
            k = k + 1;
            
            % Time replacement parameter
            x = exp(-DMP_param.a_x/tau * qPath{j}(:,1));
            
            for ii = 1:length(qPath{j}(:,1))
                psi = exp(-0.5*((x(ii) - DMP_param.c(i,:)).^2./DMP_param.sigma2(i,:)));
                
                % fac = goal * x(i) / sum(psi);
                fac = x(ii) / sum(psi);
                
                psi = fac * weights * psi;
                
                zero_idx = (psi < epsilon);
                psi(zero_idx) = 0;
                
                A = [A; sparse(psi)];
            end
            
            f = [f; weights * (tau^2 * qPath{j}(:,1+2*n+i) - ...
                DMP_param.a_z * (DMP_param.b_z * (repmat(qPath{j}(end,1+i),size(qPath{j},1),1) - qPath{j}(:,1+i)) -  ...
                tau * qPath{j}(:,1+n+i)))];
            
        end
    end
    
    DMP_param.w = [DMP_param.w, (A \ f)];
end
disp(['Elapsed time: ', num2str(toc), ', number of example trajectories: ', num2str(k)]);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function d = distance(x, x0, scale)

d = norm(x - x0) / scale;

if d < 1
    d = (1 - d^3)^3;
else
    d = 0;
end

return;
