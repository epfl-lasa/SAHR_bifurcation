%% Circular and parabolic motion around a star 

clear all;
close all;

%% Plot motion

figure; hold on;
% Polar coordinates: r = rho and theta
r = [15 0];
dr = [1 1];
ecc = 0.3;                            % eccentricity (constant)
mu = 1;                             % mass*gravitational constant
subplot(2,2,1); hold on; grid on; plot(r(1),r(2),'b*'); title('Polar coordinates');
subplot(2,2,2); hold on; grid on; plot(dr(1),dr(2),'b*'); title('Polar velocities');
subplot(2,2,3); hold on; grid on; plot(r(1)*cos(r(2)),r(1)*sin(r(2)),'b*'); title('Cartesian coordinates');
subplot(2,2,4); hold on; grid on; plot(dr(1)*cos(r(2))-r(1)*dr(2)*sin(r(2)),dr(1)*sin(r(2))+r(1)*dr(2)*cos(r(2)),'b*');
title('Cartesian velocities');

for i=1:1000
    h = r(1)*dr(2)-r(2)*dr(1);      % angular momentum per unit mass (scalar for 2d vectors)
    dr = [ecc*mu / h * sin(r(2)), mu / (h*r(1)) * (1 + ecc*cos(r(2)))];
    r = r + dr*0.01;
    subplot(2,2,1); hold on; grid on; plot(r(1),r(2),'r*');
    subplot(2,2,2); hold on; grid on; plot(dr(1),dr(2),'r*');
    subplot(2,2,3); hold on; grid on; plot(r(1)*cos(r(2)),r(1)*sin(r(2)),'r*');
    subplot(2,2,4); hold on; grid on; plot(dr(1)*cos(r(2))-r(1)*dr(2)*sin(r(2)),dr(1)*sin(r(2))+r(1)*dr(2)*cos(r(2)),'r*');
end

%% Load data

% load('ellipse_inside.mat');
% load('ellipse.mat');
% load('circle_new.mat');
load('circle.mat');
% load('attractor.mat');

X = Xstored;

%% Data preparation

T = 1000;                   % number of samples per trajectory
dt = 0.01;                  % time between samples
M = size(X,1)/T;            % number of trajectories
N = size(X,2)/2;            % number of dimensions

% Get positions and velocities
Xdata = X(:,1:N);
Xvel = X(:,N+1:N*2);

% Plot trajectories
figure; hold on;
for i=1:M
    plot(Xdata((i-1)*1000+1:i*1000,1),Xdata((i-1)*1000+1:i*1000,2),'*');
end
title('Trajectories'); grid on;
% Plot velocites
figure; hold on;
for i=1:M
    plot(Xvel((i-1)*1000+1:i*1000,1),Xvel((i-1)*1000+1:i*1000,2),'*');
end
title('Velocities'); grid on;


%% Learn parameters for dynamics: mu, ecc and rho/h

% f = @(x, v, mu, ecc, rho) norm(vecnorm(v,2,2).^2 - 2*mu/rho + (1 - ecc.^2)*mu^2 /...
%     x(:,1)*v(:,2)-x(:,2).*v(:,1));
f = @(x, v, mu, ecc, h) norm(vecnorm(v,2,2).^2 - 2*mu/sqrt(x(1)^2+x(2)^2) + (1 - ecc.^2)*mu^2 / h);
obj = @(pars) f(Xdata,Xvel,pars(1),pars(2),pars(3));

% Constraints
lb = [0 0 0];
ub = [inf 1 inf];

% Optimization
par0 = [1 0 1];
options = optimoptions(@fmincon,'MaxIterations',1500,'ConstraintTolerance',1e-4,...
    'MaxFunctionEvaluations',10000);%,'Display','iter');
[x_par,err] = fmincon(obj,par0,[],[],[],[],lb,ub,[],options);

%% Plot trajectories with found dynamics

figure; hold on;
% Polar coordinates: r = rho and theta
ecc = x_par(2);                            % eccentricity (constant)
mu = x_par(1);                             % mass*gravitational constant
h = x_par(3);

color_plot = ['g','r','b'];
for j=1:M
    subplot(2,2,1); hold on; grid on; 
    plot(Xdata((j-1)*1000+1:j*1000,1),Xdata((j-1)*1000+1:j*1000,2),'.','color',color_plot(j)); title('Original trajectories');
    subplot(2,2,2); hold on; grid on;
    plot(Xvel((j-1)*1000+1:j*1000,1),Xvel((j-1)*1000+1:j*1000,2),'.','color',color_plot(j)); title('Original velocities');
end
    
for j=1:M
    x = Xdata((j-1)*1000+1,:);             % initial position
    dx = Xvel((j-1)*1000+1,:);             % initial velocity
    subplot(2,2,3); hold on; grid on; plot(x(1),x(2),'.','color',color_plot(j)); title('Learned trajectories');
    subplot(2,2,4); hold on; grid on; plot(dx(1),dx(2),'.','color',color_plot(j)); title('Learned velocities');
    
    r = [sqrt(x(1)^2+x(2)^2), atan2(x(2),x(1))];
    dr = [(x(1)*dx(1) + x(2)*dx(2))/sqrt(x(1)^2+x(2)^2), (x(1)*dx(2)-x(2)*dx(1))/(x(1)^2+x(2)^2)];
    
    for i=1:T
    %h = r(1)*dr(2)-r(2)*dr(1);      % angular momentum per unit mass (scalar for 2d vectors)
    dr = -[ecc*mu / h * sin(r(2)), mu / (h*r(1)) * (1 + ecc*cos(r(2)))];
    r = r + dr*0.1;
    subplot(2,2,3); hold on; grid on; plot(r(1)*cos(r(2)),r(1)*sin(r(2)),'.','color',color_plot(j));
    subplot(2,2,4); hold on; grid on;
    plot(dr(1)*cos(r(2))-r(1)*dr(2)*sin(r(2)),dr(1)*sin(r(2))+r(1)*dr(2)*cos(r(2)),'.','color',color_plot(j));
    end
end

%% Plot learned field

figure; hold on; grid on; title('Learned field');
xlabel('x','FontSize',16);
ylabel('y','FontSize',16);
ecc = x_par(2);
mu = x_par(1);
h = x_par(3);

% Plot trajectories
hplot = plot(Xdata(:,1),Xdata(:,2),'*');

% Plot gradient
xl = xlim;
yl = ylim;
xlim manual
ylim manual
[Xs,Ys] = meshgrid(linspace(xl(1),xl(2),20),linspace(yl(1),yl(2),20));
X_plot = [Xs(:), Ys(:)];

r = [sqrt(X_plot(:,1).^2+X_plot(:,2).^2), atan2(X_plot(:,2),X_plot(:,1))];
dr = -[ecc*mu / h * sin(r(:,2)), (mu ./ (h*r(:,1))) .* (1 + ecc*cos(r(:,2)))];
y = [dr(:,1).*cos(r(:,2))-r(:,1).*dr(:,2).*sin(r(:,2)),...
    dr(:,1).*sin(r(:,2))+r(:,1).*dr(:,2).*cos(r(:,2))];

% r = @(X_plot) [sqrt(X_plot(1).^2+X_plot(2).^2), atan2(X_plot(2),X_plot(1))];
% dr = @(r)[ecc*mu / h * sin(r(2)), (mu ./ (h*r(1))) .* (1 + ecc*cos(r(2)))];
% dx = @(r,dr)[dr(1).*cos(r(2))-r(1).*dr(2).*sin(r(2)),...
%     dr(1).*sin(r(2))+r(1).*dr(2).*cos(r(2))];
% f = @(X_plot) dx(r(X_plot),dr(r(X_plot)));
% y = zeros(size(X_plot,1),2);
% for i = 1:size(X_plot,1)
%     y(i,:) = f(X_plot(i,:));
% end

quiver(X_plot(:,1),X_plot(:,2),y(:,1),y(:,2));

%% Interactive plot

r = @(X_plot) [sqrt(X_plot(1).^2+X_plot(2).^2), atan2(X_plot(2),X_plot(1))];
dr = @(r)[ecc*mu / h * sin(r(2)), (mu ./ (h*r(1))) .* (1 + ecc*cos(r(2)))];
dx = @(r,dr) -[dr(1).*cos(r(2))-r(1).*dr(2).*sin(r(2)),...
    dr(1).*sin(r(2))+r(1).*dr(2).*cos(r(2))];
y = @(X_plot) dx(r(X_plot),dr(r(X_plot)));      % dynamics to be plotted

f = figure;
ax = axes('Parent',f,'position',[0.1 0.13  0.85 0.8]);
%ax.YLim = [-3 3];
%ax.XLim = [-3 3];
grid on;
title('Press on axes to see dynamics of 1000 steps');
xlabel('x_1');
ylabel('x_2');

button = 1;
while sum(button) <=1   % read ginputs until a mouse right-button occurs
    [x1,x2,button] = ginput(1);
    if button == 1
        hold on; plot(x1,x2,'b.');
        X = [x1,x2];
        for i = 1:1000
            dx = y(X);
            X = X + dx*0.01;
            plot(X(1),X(2),'b.');
        end
    end
end


%% OLD CODE
% r = 2;
% p_star = [0;0];
% mu = 1;

% To have elliptical/circular orbit, total energy is negative
% E = v^2 / 2 - mu / r < 0
% v^2 < 2*mu / r -> v < sqrt(2*mu / r)

% Angular speed : d_theta = sqrt(mu / r^3)
% Transverse acceleration : d_d_r = 
