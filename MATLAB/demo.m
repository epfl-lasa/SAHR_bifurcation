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
disp('Choose a value for loading files.');
disp('1         Loads file with time as first column (data_simulation, data_icub, data_kin).')
disp('2         Loads file with time as last column (data_polishing).');
disp('3         Loads files of kinesthetic data (data_LfD).');
disp('4         Draw 2D data.');
disp('default   Loads some toy data.');
ld = input('Your choice:\n');
if(ld == 1)
    % data files in 3D with 'time' as first column
    [data,time] = loadData([],4,1);     % leave it empty to select file at runtime
    % Select the maximum length of the trajectory
    len = 5000;
    % Select how many samples to remove at beginning
    st = 35;
    if size(data,1) >= len+st
        traj.data = data(st:len+st,:);
        traj.time = time(st:len+st,:);
    else
        traj.data = data(st:end,:);
        traj.time = time(st:end,:);
    end
    traj.size = size(traj.data,1);
elseif(ld == 2)
    % data files in 3D with 'time' as last column
    [data,time] = loadData([],4,0);     % leave it empty to select file at runtime
    traj.data = data(1000:4000,:);
    traj.time = time(1000:4000,:);
    traj.size = size(traj.data,1);
elseif(ld == 3)
    % data for Learning from Demonstration
    load('circular_wiping_dataset.mat');
    j = input('Select trajectory number, 1 or 3 (defaults to 1):\n');
    if j~=1 && j~=3
        j = 1;
    end
    X = proc_data{j}.X(1:3,any(True_states{j}==[1,2],2))';
    X = X(1:end-50,:);
    time = proc_data{j}.dt;
    type = 4;
    smoothing = 30;
    T = [];
elseif(ld == 4) % draw your own dataset!
    disp('Draw one (or more) trajectories, save the data to workspace when done.');
    h = drawData();
    uiwait(h);
    if(~exist('traj','var'))
        error('Data was not saved to the workspace.');
    end
else    % Loads some toy data instead
    load('circle_rho2_M3_Rm1_th0_th0_th0.mat');
end
%% 
% Now we prepare the data and convert it to spherical coordinates:

if (exist('Xstored','var'))
    type = 1;
    X = Xstored;
    time = [];
    smoothing = 0;
    T = 1000;
elseif (exist('trial','var'))
    type = 2;
    X = [trial.position_real.x, trial.position_real.y, trial.position_real.z];
    time = trial.position_real.time;
    smoothing = 10;
    T = [];
elseif (exist('traj','var'))
    type = 3;
    X = traj.data;
    smoothing = 0;
    time = traj.time;
    if (isfield(traj,'size'))
        T = traj.size;
    else
        T = [];
    end
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
j = input('Choose number of Gaussian models (defaults to 1):\n');
if j~=1 && j~=2
    j = 1
end
[Priors, Mu, Sigma] = EM_init_kmeans([Xdata';Xvel'], j);
[Priors, Mu, Sigma] = EM([Xdata';Xvel'], Priors, Mu, Sigma);
%% 
% Plot the cluster found as the $\omega$ limit set:

figure; grid on; hold on; view(3);
title('Original data with \omega limit set found');
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
if j == 2
    plotGMM(Mu(1:3,1), Sigma(1:3,1:3,1), [1 0.5 0.5], 4);
    plotGMM(Mu(1:3,2), Sigma(1:3,1:3,2), [1 0.7 0.5], 4);
    plot3(Xdata(:,1), Xdata(:,2), Xdata(:,3), 'r');
    legend('1st Gaussian model','2nd Gaussian model','Data trajectories','Location','northeast');
else
    plotGMM(Mu(1:3,:), Sigma(1:3,1:3,:), [1 0.5 0.5], 4);
    plot3(Xdata(:,1), Xdata(:,2), Xdata(:,3), 'r');
    legend('Gaussian model','Data trajectories','Location','northeast');
end
%% If wanted, perfom dimensionality reduction
% Decide if you want to perform dimensionality reduction:
%%
prompt = 'Enter 1 to perform dimensionality reduction and 0 to skip it:\n';
dimred = input(prompt);
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
    if true
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

initial_parameters = [];
% Set initial rho0 to variance of Gaussian model
initial_parameters.rho0 = 3*mean(diag(Sigma(1:N,1:N,1)));
% Set initial values of other parameters
% initial_parameters.M = 2;
% Select which parameters to optimize in first objective (M, rho0, (always 0 for R,) a, x0):
initial_parameters.first = [1 1 0 1 0];
% Select which parameters to optimize in second objective (M, (always 0 for rho0,) R, a, x0):
initial_parameters.second = [1 0 1 0 0];        % e.g. [1 1 1 1 0] ecludes x0

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
figure; title('Dynamical flow in cartesian coordinates'); hold on; grid on;
if N == 2
    plot(Xdata(:,1),Xdata(:,2),'r.'); hold on;
else
    view(3);
    plot3(Xdata(1:T(1),1),Xdata(1:T(1),2),Xdata(1:T(1),3),'r'); hold on;
    for i = 2:m
        plot3(Xdata(sum(T(1:i-1))+1:sum(T(1:i)),1),...
            Xdata(sum(T(1:i-1))+1:sum(T(1:i)),2),...
            Xdata(sum(T(1:i-1))+1:sum(T(1:i)),3),'r'); hold on;
    end
end

% Plot streamlines / arrows to show dynamics
ylabel('x_2'); yl = ylim;
xlabel('x_1'); xl = xlim;
if N > 2
    zlabel('x_3'); zl = zlim;
    ngrid = 7;
end

if N == 2
    [Xs,Ys] = meshgrid(linspace(xl(1),xl(2),20),linspace(yl(1),yl(2),20));
    X_plot = [Xs(:), Ys(:)];
else
    [Xs,Ys,Zs] = meshgrid(linspace(xl(1),xl(2),ngrid),...
        linspace(yl(1),yl(2),ngrid),linspace(zl(1),zl(2),ngrid));
    X_plot = [Xs(:), Ys(:), Zs(:)];
end
Y = zeros(size(X_plot));
for i= 1:size(X_plot,1)
    [r,dr] = DS(X_plot(i,:),params);
    Y(i,1:N) = sph2cartvelocities(r,dr);
end

if N == 2
    streamslice(Xs,Ys,reshape(Y(:,1),20,20),...
        reshape(Y(:,2),20,20),'method','cubic');
else
    quiver3(X_plot(:,1),X_plot(:,2),X_plot(:,3),Y(:,1),Y(:,2),Y(:,3),'color','black');
end

% Test dynamics for T time steps
X0 = Xdata(1,:);
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
            plot(X(1),X(2),'k.'); hold on; grid on;
        else
            plot3(X(1),X(2),X(3),'k.'); hold on; grid on;
        end
    end
end
%% Evaluation
%%
normalized = 1;
rmserr = RMSErr(Xdata,Xvel,params,dt,normalized);
disp('RMSE (normalized):');
disp(rmserr);

coserr = cosSim(Xdata,Xvel,params,dt);
disp('Cosine similarity:');
disp(coserr);

% % Error for full reproduction:
% rmserr_DS = 1/(T) * sum(vecnorm((Xvel - Xvel_s),2,2))./ max(max(Xvel) - min(Xvel));
% disp('RMSE for DS full (normalized):');
% disp(rmserr_DS);
% for i=2:T; err(i-1)=(Xvel_s(i,:) * Xvel(i,:)') / (norm(Xvel(i,:))*norm(Xvel_s(i,:))); end
% coserr_DS = 1/(T) * sum(abs(1 - err));
% disp('Cosine similarity for DS full:');
% disp(coserr_DS);


% Save values if needed (optional -- uncomment)
%save("eval.mat",'rmserr','coserr');
%% Testing reproduction with DMP learning (optional -- uncomment)
%%
% first = 1;        % PDMP only expects periodic data, here you can remove approaching phase
% last = T;
% trj = [smooth(Xdata(first:last,1),30),smooth(Xdata(first:last,2),30),smooth(Xdata(first:last,3),30)];
% timeDMP = time(first:last) - time(first);
% 
% %%%%%%% Run now the script 'test_case_pdmp.m' in 'evaluation/DMP/koda' !!!! %%%%%%%%
% 
% Xvel_DMP = diff(TRJ)/dt;
% rmserr_DMP = 1/(last-first) * sum(vecnorm((Xvel(first+1:last,:) - Xvel_DMP),2,2))./ max(max(Xvel(first+1:last,:)) - min(Xvel(first+1:last,:)));
% disp('RMSE for DMP (normalized):');
% disp(rmserr_DMP);
% for i=2:last-first; err(i-1)=(Xvel_DMP(i,:) * Xvel(i+first-1,:)') / (norm(Xvel(i+first-1,:))*norm(Xvel_DMP(i,:))); end
% coserr_DMP = 1/(last-first) * sum(abs(1 - err));
% disp('Cosine similarity for DMP:');
% disp(coserr_DMP);
%% 
% Other version of DMP:

% st = 301;
% global rcps;
% XvelSm = [smooth(Xvel(st:end,1),20), smooth(Xvel(st:end,2),20), smooth(Xvel(st:end,3),20)];
% if length(time) == T+1
%     time = time(1:end-1);
% elseif length(time) == 1
%     time = [dt:dt:T*dt]';
% end
% [XDMP,XvDMP] = LearningDMPComparison(Xdata(st:end,:),XvelSm,dt,time(st:end,:),0);
% % figure; plot(Xdata(st:end,:)); hold on; grid on; plot(XDMP,'--');
% % title('Original data and reproduced data with DMP');
% % 
% % rmserr_DMP = 1/(T-st+1) * sum(vecnorm((Xvel(st:end,:) - XvDMP),2,2))./ max(max(Xvel(st:end,:)) - min(Xvel(st:end,:)));
% % disp('RMSE for DMP (normalized):');
% % disp(rmserr_DMP);
% % for i=2:T-st+1; err(i-1)=(XvDMP(i,:) * Xvel(i+st-1,:)') / (norm(Xvel(i+st-1,:))*norm(XvDMP(i,:))); end
% % coserr_DMP = 1/(T-st+1) * sum(abs(1 - err));
% % disp('Cosine similarity for DMP:');
% % disp(coserr_DMP);
% 
% Xvel_DMP = zeros(T-st+1,N);
% for ID = 1:N
%     rcp('change',ID,'y',Xdata(st,ID));
%     rcp('change',ID,'yd',Xvel(st,ID));
%     for i=1:T-st+1
%         [y,yd,ydd]=rcp('run',ID,rcps(ID).tau,dt);
%         P(i+1,:)   = [rcps(ID).p rcps(ID).pd];
%         Z(i+1,:)   = [rcps(ID).z rcps(ID).zd];
%         Y(i+1,:)   = [y yd ydd];
%         PSI(i+1,:) = rcps(ID).psi';
%         W(i+1,:,:) = rcps(ID).w;
%         Xvel_DMP(i,ID) = yd;
%     end
% end
% figure; title('Velocity original (-) vs learned with DMP (--)');
% for c=1:N
%     col = zeros(1,3);
%     col(c) = 1;
%     plot(Xvel(st:end,c),'Color',col); hold on; grid on; plot(Xvel_DMP(:,c),'LineStyle','--','Color',col);
% end
% 
% rmserr_DMP = 1/(T-st+1) * sum(vecnorm((Xvel(st:end,:) - Xvel_DMP),2,2))./ max(max(Xvel(st:end,:)) - min(Xvel(st:end,:)));
% disp('RMSE for DMP (normalized):');
% disp(rmserr_DMP);
% for i=2:T-st+1; err(i-1)=(Xvel_DMP(i,:) * Xvel(i+st-1,:)') / (norm(Xvel(i+st-1,:))*norm(Xvel_DMP(i,:))); end
% coserr_DMP = 1/(T-st+1) * sum(abs(1 - err));
% disp('Cosine similarity for DMP:');
% disp(coserr_DMP);
%% Export parameters to .mat file
%%
% save("params.mat",'params');
%% Export parameters to .txt file (optional -- uncomment)
%%
% saveToTxt(params);
%% Export parameters to ROS configuration file (optional -- uncomment)
%%
% saveToCfg(params);