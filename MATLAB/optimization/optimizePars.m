function [params] = optimizePars(init_params, Xdata, dt, begin)
%OPTIMIZEPARS Performs optimization to find parameters of the DS with
%bifurcation. (Excluding parameter theta0.)
%   INPUT:
%   init_params:    struct containing the initialization of the parameters
%   Xdata:          data matrix, with size MxN (# datapoints x dimensions)
%   Xvel:           velocities, with size MxN
%   dt:             time difference between subsequent data points
%   begin:          starting index of each data trajectory

N = size(Xdata,2);

% Build the vector of initial value of parameters for optimization
par0 = [];
if(isfield(init_params,'M'))
    par0 = [par0 init_params.M];
else 
    par0 = [par0 1];
end
if(isfield(init_params,'rho0'))
    par0 = [par0 init_params.rho0];
else
    par0 = [par0 0];
end
if(isfield(init_params,'R'))
    par0 = [par0 init_params.R];
else
    par0 = [par0 randn(1,1)];
end
if(isfield(init_params,'a'))
    par0 = [par0 init_params.a];
else
    par0 = [par0 ones(1,3)];
end
if(isfield(init_params,'x0'))
    par0 = [par0 init_params.x0];
else
    par0 = [par0 zeros(1,3)];
end

% Constraints of parameters [M, rho0, R, a and x0]
range = [min(min(Xdata))-(min(min(Xdata))/10),...
    max(max(Xdata))+(max(max(Xdata))/10)];
lb = [0.001 0 -10*ones(1,1) ones(1,N) range(1)*ones(1,N)];
ub = [10 range(2) 10*ones(1,1) 10*ones(1,N) range(2)*ones(1,N)];

% Functions to find spherical coordinates (considering scale and shift)
Rdatas = @(a,x0) cart2hyper(a.*(Xdata+x0));
Rvels = @(a,x0) cart2sphvelocities(a.*(Xdata+x0),...
    subvelocities(a.*(Xdata+x0),dt,begin),Rdatas(a,x0),begin);
Rsrho = @(Rdata) Rdata(:,1);
Rsth1 = @(Rdata) Rdata(:,2);
Rvelsrho = @(Rvel) Rvel(:,1);
if N == 3
    Rvelsth1 = @(Rvel) Rvel(:,2);
end
Rvelsth = @(Rvel) Rvel(:,end);

% Differential equations
dU = @(r,M,rho0,R) 2.*M.*(r(:,1) - rho0);
dRho = @(r,M,rho0,R) - sqrt(M.*2) .* (r - rho0);
dTheta = @(r,M,rho0,R) R .* exp(-dU(r(:,1),M,rho0,R).^2);


% Optimization

options = optimoptions(@fmincon,'MaxIterations',5000,...%'Display','iter',...
    'MaxFunctionEvaluations',10000,'Algorithm','sqp');
% First objective
obj1 = @(pars) norm(dRho(Rsrho(Rdatas(pars(4:N+3),pars(N+4:N*2+3))),...
    pars(1),pars(2),pars(3)) - Rvelsrho(Rvels(pars(4:N+3),pars(N+4:N*2+3))))^2;
[x_par,err1] = fmincon(obj1,par0,[],[],[],[],lb,ub,[],options);
% Second objective
obj2 = @(pars) norm(dTheta(Rsrho(Rdatas(x_par(4:N+3),x_par(N+4:N*2+3))),...
    pars(1),x_par(2),pars(3)) - ...
    Rvelsth(Rvels(x_par(4:N+3),x_par(N+4:N*2+3))))^2;
[x_par2,err2] = fmincon(obj2,x_par,[],[],[],[],lb,ub,[],options);
% Loop
steps = 1;
while(steps < 10 && (err1 > 0.01 || err2 > 0.01))
    % First objective
    obj1 = @(pars) norm(dRho(Rsrho(Rdatas(pars(4:N+3),pars(N+4:N*2+3))),...
        pars(1),pars(2),x_par2(3)) - Rvelsrho(Rvels(pars(4:N+3),pars(N+4:N*2+3))))^2;
    [x_par,err1] = fmincon(obj1,x_par2,[],[],[],[],lb,ub,[],options);
    % Second objective
    obj2 = @(pars) norm(dTheta(Rsrho(Rdatas(x_par(4:N+3),x_par(N+4:N*2+3))),...
        pars(1),x_par(2),pars(3)) - ...
        Rvelsth(Rvels(x_par(4:N+3),x_par(N+4:N*2+3))))^2;
    [x_par2,err2] = fmincon(obj2,x_par,[],[],[],[],lb,ub,[],options);
    
    steps = steps + 1;
end

disp('Minimized errors:');
err1
err2

% Export found parameters
disp('Parameters found:');
params = [];
params.rho0 = x_par2(2);
params.M = x_par2(1);
params.R = x_par2(3);
params.a = x_par2(4:N+3);
params.x0 = x_par2(N+4:N*2+3)

end

