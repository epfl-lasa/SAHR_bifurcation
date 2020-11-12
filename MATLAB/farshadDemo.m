%% *Demo -- Optimization*
% This demo shows the steps of the optimization for the project on:
% 
% *"Learning dynamical systems with bifurcations"*
%%
clear all;
close all;
%% Add dependencies and folders to path
%%
addpath(genpath('.'));
%% Load and prepare data
% You can load data from a .csv file or from a .mat file:
%%


load('datatest12.mat', 'traj')
initial_parameters = [];
initial_parameters.M = 10;
initial_parameters.R = 1; 
dirsgn = .1;
% data files in 3D with 'time' as first column
    % Select the maximum length of the trajectory
    len = 6000;
    % Select how many samples to remove at beginning
    st = 100;
    rat = ceil(max(len/traj.size, 1));
    yfar1 = interp(traj.data(st:end-st,1),rat);
    yfar2 = interp(traj.data(st:end-st,2),rat);
    tfar1= interp(traj.time(st:end-st,1),rat);
    if size(yfar1,1) >= len
        yfar1 = yfar1(1:len,:);
        yfar2 = yfar2(1:len,:);
        tfar1 = tfar1(1:len,:);
    end
    tfar1 = linspace(tfar1(1),tfar1(end),size(tfar1,1));
%% 
% Now we prepare the data and convert it to spherical coordinates:


type = 3;
X = [yfar1,yfar2];
smoothing = 0;
time = tfar1;
if (isfield(traj,'size'))
    T = size(yfar1,1);
else
    T = [];
end

% Get data with preferred structure
[Xdata,Xvel,Rdata,Rvel,dt,T,N,m,begin] = prepareData(type,X,time,smoothing,T);
%% Plot data
%%
plotData(Xdata,Xvel,Rdata,Rvel,T,m,'cartp');
legend('Data trajectory','Location','northeast');
%% Find $\omega$ limit set
% Initialize and perform Expectation Maximization on a Gaussian Mixture Model 
% with 1 or 2 Gaussians to find the model of the limit set:
%%
j=1;
[Priors, Mu, Sigma] = EM_init_kmeans([Xdata';Xvel'], j);
[Priors, Mu, Sigma] = EM([Xdata';Xvel'], Priors, Mu, Sigma);

%% If wanted, perfom dimensionality reduction
% Decide if you want to perform dimensionality reduction:
%%
% prompt = 'Enter 1 to perform dimensionality reduction and 0 to skip it:\n';
% dimred = input(prompt); 
dimred = 1;
%% 
% If wanted, perform dimensionality reduction (PCA) and plot projected data:

if (j == 2)     % if there are more clusters, choose which one to perform eigenvalue decomposition on
    prompt = 'Choose which gaussian to keep (1 for 1st, 2 for 2nd):\n';
    k = input(prompt);
else
    k = 1;
end
if (dimred == 1)
    % Get rotation matrix from PCA performed on covariance matrix of gaussian:
    [Rrot,~] = eig(Sigma(1:N,1:N,k));
    Rrot = Rrot(:,N:-1:1);
    
    % Plot projected (rotated) data -- takes a lot of time! If you trust it, set "if false"
    if false
        figure; hold on; grid on;
        subplot(1,2,1); hold on; grid on;
        title('Original data'); xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
        subplot(1,2,2); hold on; grid on;
        title('Projected data'); xlabel('e_1'); ylabel('e_2'); zlabel('e_3');
        
        for i = 1:sum(T)
            Xplot = (Rrot \ (Xdata(i,:)' - Mu(1:N,k)))';
            if N == 2
                subplot(1,2,1); plot(Xdata(i,1), Xdata(i,2), 'r.'); grid on; hold on;
                subplot(1,2,2); plot(Xplot(1), Xplot(2), 'r.'); grid on; hold on;
            else
                subplot(1,2,1); view(3); plot3(Xdata(i,1), Xdata(i,2), Xdata(i,3), 'r.'); grid on; hold on;
                subplot(1,2,2); view(3); plot3(Xplot(1), Xplot(2), Xplot(3), 'r.'); grid on; hold on;
            end
        end
    end
else
    % If you are not performing dimensionality reduction
    Rrot = eye(N);
end

% Find Euler angle (or rotation angle if N = 2)
if(N == 3)
    theta0 = rotm2eul(Rrot)
elseif(N == 2)
    theta0 = acos(Rrot(1,1))
end

% Get rotated data and save original data
Xdata_ = Xdata;
%% Optimization
% Select initial values of paramaters and optimize:
%%
Xdata = Xdata_;
Xdata = (Rrot \ (Xdata' - Mu(1:N,k)))';


% initial_parameters = [];
% Set initial rho0 to variance of Gaussian model
initial_parameters.rho0 = 3*mean(diag(Sigma(1:N,1:N,1)));
% Set initial values of other parameters
initial_parameters.x0 = [0 0]; 
% initial_parameters.M = 20;
% Select which parameters to optimize in first objective (M, rho0, (always 0 for R,) a, x0):
initial_parameters.first = [0 1 0 1 1];
% Select which parameters to optimize in second objective (M, (always 0 for rho0,) R, a, x0):
initial_parameters.second = [0 0 1 0 0];        % e.g. [1 1 1 1 0] ecludes x0

%%%%%% OPTIMIZATION FUNCTION: %%%%%%
[params] = optimizePars(initial_parameters,Xdata,dt,begin,1);
%% Get parameters learned
%%
rho0 = params.rho0;
M = params.M;
R = params.R;
a = params.a;
% Add the mean of the Gaussian model to x0
if isfield(initial_parameters,'x0') && isempty(initial_parameters.x0)
    x0 = -Mu(1:N,k)';
else
    x0 = (Rrot * params.x0' - Mu(1:N,k))';
end
params.x0 = x0;
params.Rrot = Rrot;
disp(params);
%% Plot learned dynamics with original data and new trajectories
%%
% Get unrotated data
Xdata = Xdata_;

% Plot original data
figure; hold on; grid on;
plot(Xdata(:,1),Xdata(:,2),':','Color','r','LineWidth',2); hold on;

% Plot streamlines / arrows to show dynamics
 yl = ylim;
 xl = xlim;

ylim(yl);
xlim(xl);
resol = 500;
[Xs,Ys] = meshgrid(linspace(xl(1),xl(2),resol),linspace(yl(1),yl(2),resol));
X_plot = [Xs(:), Ys(:)];

Y = zeros(size(X_plot));
for i= 1:size(X_plot,1)
    [r,dr] = DS(X_plot(i,:),params);
    dr = dirsgn*dr;
    Y(i,1:N) = sph2cartvelocities(r,dr);
end

streamslice(Xs,Ys,reshape(Y(:,1),resol,resol),...
reshape(Y(:,2),resol,resol),'method','cubic');
ax = gca;
ax.FontSize = 14; 
ylim(yl);
xlim(xl);
ylabel('$\xi_2$','Interpreter','latex','FontSize',18,'FontWeight','bold');
xlabel('$\xi_1$','Interpreter','latex','FontSize',18,'FontWeight','bold');
% Test dynamics for T time steps
X0 = 1.5*Xdata(1,:);
X_s = []; Xvel_s = [];
for j = 1:size(X0,1)
    X = X0(j,:);
    for i = 1:T(1)
        X_prev = X;
        %%%%%% FUNCTION TO CALCULATE POLAR/SPHERICAL VELOCITIES: %%%%%%
        [r,dr] = DS(X,params);
        %%%%%% INTEGRATE THE DS TO GET NEXT POLAR/SPHERICAL POSITION: %%%%%%
        next_r = r + dr*dt;
        % Get next cartesian position
        X = (Rrot*(hyper2cart(next_r)./a)')' - x0;
        X_s = [X_s; X];
        Xvel_s = [Xvel_s; sph2cartvelocities(r,dr)];
        if N == 2
            plot(X(1),X(2),'k.','LineWidth',2,'MarkerSize',10); hold on; grid on;
        else
            plot3(X(1),X(2),X(3),'k.'); hold on; grid on;
        end
    end
end
