function [params] = optimizePars(init_params, Xdata, dt, begin, stepmax)
%OPTIMIZEPARS Performs optimization to find parameters of the DS with
%bifurcation. (Excluding parameter theta0.)
%   INPUT:
%   init_params:    struct containing the initialization of the parameters
%   Xdata:          data matrix, with size MxN (# datapoints x dimensions)
%   Xvel:           velocities, with size MxN
%   dt:             time difference between subsequent data points
%   begin:          starting index of each data trajectory
%   stepmax:        maximum number of repetitions of optimization loop

N = size(Xdata,2);

% Build the vector of initial value and constraints of parameters for optimization
par0 = []; lb = []; ub = [];
range = [min(min(Xdata))-(min(min(Xdata))/10),...
    max(max(Xdata))+(max(max(Xdata))/10)];
% M
if(isfield(init_params,'M') && ~isempty(init_params.M))
    par0 = [par0 init_params.M];
else
    par0 = [par0 1];
end
lb = [lb 0.001];
ub = [ub 5];
% rho0
if(isfield(init_params,'rho0') && ~isempty(init_params.rho0))
    par0 = [par0 init_params.rho0];
else
    par0 = [par0 0];
end
lb = [lb 0];
ub = [ub range(2)];
% R
if(isfield(init_params,'R') && ~isempty(init_params.R))
    par0 = [par0 init_params.R];
else
    par0 = [par0 randn(1,1)];
end
lb = [lb -10*ones(1,1)];
ub = [ub 10*ones(1,1)];
% a
if(isfield(init_params,'a') && ~isempty(init_params.a))
    par0 = [par0 init_params.a];
else
    par0 = [par0 ones(1,3)];
end
lb = [lb ones(1,N)];
ub = [ub 10*ones(1,N)];
% x0
if(isfield(init_params,'x0') && ~isempty(init_params.x0))
    par0 = [par0 init_params.x0];
else
    par0 = [par0 zeros(1,3)];
end
lb = [lb range(1)*ones(1,N)];
ub = [ub range(2)*ones(1,N)];

% Optimization
options = optimoptions(@fmincon,'MaxIterations',5000,...%'Display','iter',...
    'MaxFunctionEvaluations',10000,'Algorithm','sqp');
x_par2 = par0;

% Loop
steps = 0;
opts = [];
opts.dt = dt;
opts.begin = begin;
err1 = 100;
err2 = 100;
while(steps < stepmax && (err1 > 0.01 || err2 > 0.01))
    % First objective
    obj1 = @(pars) objective1(pars,x_par2,Xdata,init_params.first,N,opts);
    [x_par,err1] = fmincon(obj1,x_par2,[],[],[],[],lb,ub,[],options);
    % Second objective
    obj2 = @(pars) objective2(pars,x_par,Xdata,init_params.second,N,opts);
    [x_par2,err2] = fmincon(obj2,x_par,[],[],[],[],lb,ub,[],options);
    
    % Third objective (match direction)
    
%     if N == 3
%         obj3 = @(pars) norm(dRho(Rsth1(Rdatas(x_par2(4:N+3),pars(N+4:N*2+3))),...
%             pars(1),0,1) - Rvelsth1(Rvels(x_par2(4:N+3),pars(N+4:N*2+3))))^2;
%         [x_par2,err3] = fmincon(obj3,x_par2,[],[],[],[],lb,ub,[],options);
%     end
    
    steps = steps + 1;
end

disp('Minimized errors:');
err1
err2
% err3

% Export found parameters
disp('Parameters found by optimization:');
params = [];
params.rho0 = x_par2(2);
params.M = x_par2(1);
params.R = x_par2(3);
params.a = x_par2(4:N+3);
params.x0 = x_par2(N+4:N*2+3)

end

