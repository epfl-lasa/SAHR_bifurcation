% Learn bounded energy function using potential energy (for ND)

clear all;
close all;

addpath('data');
addpath('potential');

%% Load data

% 2D:
% load('ellipse.mat');
% load('circle0_new.mat');
% load('attractor.mat');
% load('circle_rho3_M2_Rm1.mat');

% 3D:
load('circle_rho3_M2_Rm1_R2.mat');

X = Xstored;

%% Data preparation

T = 1000;                   % number of samples per trajectory
dt = 0.01;                  % time between samples
m = size(X,1)/T;            % number of trajectories
N = size(X,2)/2;            % number of dimensions

% Get positions and velocities
Xdata = X(:,1:N);
Xvel = X(:,N+1:N*2);

% Get (hyper)spherical coordinates and velocities
% if N == 2
%     Rdata = [sqrt(Xdata(:,1).^2+Xdata(:,2).^2), atan2(Xdata(:,2),Xdata(:,1))];
%     Rvel = [(Xdata(:,1).*Xvel(:,1)+Xdata(:,2).*Xvel(:,2))./...
%         sqrt(Xdata(:,1).^2 + Xdata(:,2).^2),...
%         Xdata(:,1).*Xvel(:,2)+Xdata(:,2).*Xvel(:,1)./...
%         (Xdata(:,1).^2 + Xdata(:,2).^2)];
% end
Rdata = cart2hyper(Xdata);
Rvel = cart2hypervelocities(Xdata,Xvel,Rdata);


%%
% Plot motion
figure; hold on;
if N == 2
    subplot(2,2,1); hold on; grid on; title('Polar coordinates');
    xlabel('\rho'); ylabel('\theta');
    subplot(2,2,2); hold on; grid on; title('Polar velocities');
    xlabel('d\rho'); ylabel('d\theta_1');
    subplot(2,2,3); hold on; grid on; title('Cartesian coordinates');
    xlabel('x_1'); ylabel('x_2');
    subplot(2,2,4); hold on; grid on; title('Cartesian velocities');
    xlabel('dx_1'); ylabel('dx_2');
else
    subplot(2,2,1); hold on; grid on;
    title('Spherical coordinates');
    xlabel('\rho'); ylabel('\theta_1'); zlabel('\theta_2');
    subplot(2,2,2); hold on; grid on;
    title('Spherical velocities');
    xlabel('d\rho'); ylabel('d\theta_1'); zlabel('d\theta_2');
    subplot(2,2,3); hold on; grid on; title('Cartesian coordinates');
    xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
    subplot(2,2,4); hold on; grid on; title('Cartesian velocities');
    xlabel('dx_1'); ylabel('dx_2'); zlabel('dx_3');
end

for i=1:m
    if N == 2
        subplot(2,2,1); % Polar coordinates
        plot(Rdata((i-1)*T+1:i*T,1),Rdata((i-1)*T+1:i*T,2),'*');
        
        subplot(2,2,2); % Polar velocities
        plot(Rvel((i-1)*T+1:i*T,1),Rvel((i-1)*T+1:i*T,2),'*');
        
        subplot(2,2,3); % Cartesian coordinates
        plot(Xdata((i-1)*T+1:i*T,1),Xdata((i-1)*T+1:i*T,2),'*');
        
        subplot(2,2,4); % Cartesian velocities
        plot(Xvel((i-1)*T+1:i*T,1),Xvel((i-1)*T+1:i*T,2),'*');
    else
        subplot(2,2,1); % Spherical coordinates
        plot3(Rdata((i-1)*T+1:i*T,1),Rdata((i-1)*T+1:i*T,2),...
            Rdata((i-1)*T+1:i*T,3),'*')
        
        subplot(2,2,2); % Spherical velocities
        plot3(Rvel((i-1)*T+1:i*T,1),Rvel((i-1)*T+1:i*T,2),...
            Rvel((i-1)*T+1:i*T,3),'*');
        
        subplot(2,2,3); % Cartesian coordinates
        plot3(Xdata((i-1)*T+1:i*T,1),Xdata((i-1)*T+1:i*T,2),...
            Xdata((i-1)*T+1:i*T,3),'*');
        
        subplot(2,2,4); % Cartesian velocities
        plot3(Xvel((i-1)*T+1:i*T,1),Xvel((i-1)*T+1:i*T,2),...
            Xvel((i-1)*T+1:i*T,3),'*');
    end
end


%% Potential, energy and DS

U = @(r,M,rho0,R) M*(r(:,1) - rho0).^2;
dU = @(r,M,rho0,R) 2.*M.*(r(:,1) - rho0);
dRho = @(r,M,rho0,R) - sqrt(2.*M) .* (r(:,1) - rho0);
dTheta = @(r,M,rho0,R) R .* exp(-dU(r(:,1),M,rho0,R).^2);

E = @(r,M,rho0,R) 1/2 * (dRho(r,M,rho0,R).^2 + dTheta(r,M,rho0,R).^2) - U(r,M,rho0,R);

%% Learn parameters M, rho0 and R_i

% Constraints (number of R_i varies depending on N)
lb = [0.01 0 -100*ones(1,N-1)];
ub = [100*ones(1,2) 100*ones(1,N-1)];

% Objective
obj1 = @(pars) norm(dRho(Rdata(:,1),pars(1),pars(2),pars(3)) - Rvel(:,1));
% obj = @(pars) norm(dTheta(Rdata(:,1),pars(1),pars(2),pars(3)) - Rvel(:,2));
% norm(E(Rdata(:,1),pars(1),pars(2),pars(3)) - 1/2 * Rvel(:,2).^2);

% Optimization
par0 = [randn(1,2) randn(1,N-1)];
options = optimoptions(@fmincon,'MaxIterations',5000,...%'Display','iter',...
    'MaxFunctionEvaluations',10000,'Algorithm','sqp');
[x_par,fval1] = fmincon(obj1,par0,[],[],[],[],lb,ub,[],options);
obj2 = @(pars) norm(vecnorm(dTheta(Rdata(:,1),x_par(1),x_par(2),pars(3:N+1)) - Rvel(:,2:end),2,2));
[x_par2,fval2] = fmincon(obj2,x_par,[],[],[],[],lb,ub,[],options);
% [x_par,err] = fminunc(obj,par0);

disp('Minima found:');
fval1
fval2
disp('Value of parameters found (M, rho0 and R_i)');
x_par
x_par2

%% Interactive plot of trajectories with found parameters

M = x_par(1);
rho0 = x_par(2);
R = x_par2(3:end);

% if N == 2
%     r = @(X_plot) [sqrt(X_plot(1).^2+X_plot(2).^2), atan2(X_plot(2),X_plot(1))];
%     dr = @(r) [dRho(r(1),M,rho0,R), dTheta(r(1),M,rho0,R)];
%     dx = @(r,dr) [dr(1).*cos(r(2))-r(1).*dr(2).*sin(r(2)),...
%         dr(1).*sin(r(2))+r(1).*dr(2).*cos(r(2))];
%     y = @(X_plot) dx(r(X_plot),dr(r(X_plot)));      % dynamics to be plotted
% else
    r = @(X_plot) cart2hyper(X_plot);%[sqrt(sum(X_plot(1:3).^2,2)), acos(X_plot(1)./...
        %sqrt(sum(X_plot(1:3).^2,2))), acos(X_plot(2)./sqrt(sum(X_plot(2:3).^2,2)))];
    dr = @(r) [dRho(r(:,1),M,rho0,R), dTheta(r(:,1),M,rho0,R)];
    y = @(X_plot) hyper2cartvelocities(r(X_plot),dr(r(X_plot))); % dynamics to be plotted
% end

% Plot dynamics flow
figure; title('Dynamical flow in cartesian coordinates');
yl = [-rho0-2 rho0+2];
xl = [-rho0-2 rho0+2];
if N > 2
    zl = [-rho0-2 rho0+2];
end

if N == 2
    [Xs,Ys] = meshgrid(linspace(xl(1),xl(2),20),linspace(yl(1),yl(2),20));
    X_plot = [Xs(:), Ys(:)];
else
    [Xs,Ys,Zs] = meshgrid(linspace(xl(1),xl(2),5),linspace(yl(1),yl(2),5),...
        linspace(zl(1),zl(2),5));
    X_plot = [Xs(:), Ys(:), Zs(:)];
end
Y = zeros(size(X_plot));
for i= 1:size(X_plot,1)
    Y(i,:) = y(X_plot(i,:));
end
if N ==2
    streamslice(Xs,Ys,reshape(Y(:,1),20,20),reshape(Y(:,2),20,20),'method','cubic');
else
    streamslice(Xs,Ys,Zs,reshape(Y(:,1),5,5,5),reshape(Y(:,2),5,5,5),...
        reshape(Y(:,3),5,5,5),'method','cubic');
end

% Plot dynamics flow in polar/spherical coordinates
figure; title('Dynamical flow in polar/spherical coordinates');
xlabel('\rho'); ylabel('\theta_1');
yl = [-3.2 3.2];
xl = [-rho0-2 rho0+2];
if N > 2
    zl = [-3.2 3.2];
    zlabel('\theta_2');
    ngrid = 5;
end

if N == 2
    [Xs,Ys] = meshgrid(linspace(xl(1),xl(2),20),linspace(yl(1),yl(2),20));
    X_plot = [Xs(:), Ys(:)];
else
    [Xs,Ys,Zs] = meshgrid(linspace(xl(1),xl(2),ngrid),linspace(yl(1),yl(2),ngrid),...
        linspace(zl(1),zl(2),ngrid));
    X_plot = [Xs(:), Ys(:), Zs(:)];
end
Y = zeros(size(X_plot));
for i= 1:size(X_plot,1)
    Y(i,:) = [dRho(X_plot(i,:),M,rho0,R), dTheta(X_plot(i,:),M,rho0,R)];
end
if N ==2
    streamslice(Xs,Ys,reshape(Y(:,1),20,20),reshape(Y(:,2),20,20),'method','cubic');
else
    streamslice(reshape(Y(:,1),ngrid,ngrid,ngrid),reshape(Y(:,2),ngrid,ngrid,ngrid),...
        reshape(Y(:,3),ngrid,ngrid,ngrid),Xs,Ys,Zs,'method','cubic');
end


pause(1);

% Interactive plot
f = figure(3);
ax = axes('Parent',f,'position',[0.1 0.13  0.85 0.8]);
%ax.YLim = [-3 3];
%ax.XLim = [-3 3];
grid on; zoom on;
title('Press on axes to see dynamics for 1000 steps');
xlabel('x_1');
ylabel('x_2');

button = 1;
while sum(button) <=1   % read ginputs until a mouse right-button occurs
    [x1,x2,button] = ginput(1);
    if button == 1
        hold on; plot(x1,x2,'b.');
        X = [x1,x2];
        if N > 2
            X = [X,zeros(1,N-2)];
        end
        for i = 1:T
            %dx = y(X1);
            %X = X + dx(1:2)*0.01;
            Rad = r(X) + dr(r(X)) * 0.01;
            X = hyper2cart(Rad);
            plot(X(1),X(2),'b.');
        end
    end
end


