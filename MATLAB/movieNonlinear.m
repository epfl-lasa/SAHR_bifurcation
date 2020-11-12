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
nameVideo = 'nonlinear_data_12.mp4';
Xint = [1.05, 1.15, .95, 1.3, 1.1, 1.10, 1.1, 1.15;...
        .9,   1.05, 1.2,  .8, 1.0, 1.29, .71, 1.10];
%% Load and prepare data
% You can load data from a .csv file or from a .mat file:
%%


load('datatest12.mat', 'traj')
initial_parameters = [];
initial_parameters.M = 15;
initial_parameters.R = 1; 
dirsgn = 1;
no_gauss = 15;
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
%% Nonlinear Limit Cycle
ZData = Xdata;

thData = zeros(size(ZData,1),1);
rsData = zeros(size(ZData,1),1);
for cnt=1:size(ZData,1)
    rr = cart2hyper(ZData(cnt,:).*a);
    [theta,~]= cart2pol(ZData(cnt,1)*a(1),ZData(cnt,2)*a(2));
    thData(cnt) = theta;
    
%     alp = (a(2)* abs(ZData(cnt,2))/ rho0) + (a(1)* abs(ZData(cnt,1))/ rho0);
%     alp = sqrt((a(2)* abs(ZData(cnt,2))/ rho0)^2 + (a(1)* abs(ZData(cnt,1))/ rho0)^2);
    r0 = rho0;
    rsData(cnt) = rr(:,1) / r0;
end
% figure;
% scatter(thData, rData);

[t_sort,t_order] = sort(thData);

rdata = [t_sort,rsData(t_order,1)];
% ----
window_size = 60;
bins = linspace( min(t_sort),max(t_sort), window_size);
mu_bin = zeros(length(bins)-1,2);
sig_bin = zeros(length(bins)-1,2);
count_bin = zeros(length(bins)-1,1);
for ic = 1:length(bins)-1
    logicalindex = (t_sort >bins(ic)) & (t_sort < bins(ic+1));
    indx = find(logicalindex);
    bin_data = [t_sort(indx,1),rdata(indx,2)];
    bin_slice(ic).data = bin_data;
    count_bin(ic) = length(bin_data);
    mu_bin(ic,:) = mean(bin_data);
    sig_bin(ic,:) = std(bin_data);
end
for iter=1:10
    meancount = ceil(.9*mean(count_bin));
    for jc = 1:length(bin_slice)
        if (count_bin(jc) < meancount)
            sigma_ = .1*[min(sig_bin(jc,1),.01) 0;0 min(sig_bin(jc,2),.01)];
            newpoints = mvnrnd(mu_bin(jc,:),sigma_*sigma_,meancount - count_bin(jc));
%             randindex = ceil(length(bin_slice(jc).data)*rand(meancount - count_bin(jc),1) );
%             newpoints =  bin_slice(jc).data(randindex,:);
            bin_slice(jc).data = [bin_slice(jc).data;newpoints];
            count_bin(jc) = count_bin(jc) + length(newpoints);
            mu_bin(jc,:) = mean(bin_slice(jc).data);
            sig_bin(jc,:) = std(bin_slice(jc).data);
        end
    end
end

rsdata = [];
for kc = 1:length(bin_slice)
    rsdata =[rsdata;bin_slice(kc).data];
end

[tt_sort,tt_order] = sort(rsdata(:,1));
rrdata = [tt_sort,rsdata(tt_order,2)];
% ---

addfirst = (rrdata(end-500:end,:)'+ [rrdata(1,1);0]-[rrdata(end,1);0])';
addlast = (rrdata(1:500,:)'- [rrdata(1,1);0]+[rrdata(end,1);0])';
Rdata = [addfirst;rrdata ; addlast];
% rho_polyModel = polyfit(Rdata(:,1),Rdata(:,2),20);

options_gmm = statset('Display','final','MaxIter',1500,'TolFun',1e-8);

% AIC = zeros(25,1);
% for kk=1:25
% rho_GMModel = fitgmdist(Rdata,kk,'Options',options_gmm);
% AIC(kk) = rho_GMModel.NegativeLogLikelihood;
% end

rho_GMModel = fitgmdist(Rdata,no_gauss,'Options',options_gmm);
gmPDF = @(x,y)reshape(pdf(rho_GMModel,[x(:) y(:)]),size(x));

figure;
scatter(Rdata(:,1), Rdata(:,2),5,'MarkerEdgeColor','none',...
    'MarkerFaceColor','r','LineWidth',.5);
hold on;
fcontour(gmPDF,[-4 4 .3 1.5],'LevelList',.1:.2:1.5,'LineWidth', 2);
grid on;
ax = gca;
ax.FontSize = 14; 
ylabel('$\bf{g}(\theta)$','Interpreter','latex','FontSize',18,'FontWeight','bold');
xlabel('$\theta$','Interpreter','latex','FontSize',18,'FontWeight','bold');
ylim([.3 1.7]);
xlim([-4. 4]);
% plot(Rdata(:,1),polyval(rho_polyModel,Rdata(:,1)));
hold off;
%% Plot learned dynamics with original data and new trajectories
%%
% Get unrotated data
Xdata = Xdata_;

% Plot original data
figure; hold on; grid on;

plot(Xdata(:,1),Xdata(:,2),'r.'); hold on;

% Plot streamlines / arrows to show dynamics
 yl = ylim;
 xl = xlim;

 xl = xl ;
 yl = yl ;
ylim(yl);
xlim(xl);
resol = 500;
[Xs,Ys] = meshgrid(linspace(xl(1),xl(2),resol),linspace(yl(1),yl(2),resol));
X_plot = [Xs(:), Ys(:)];

% Y = zeros(size(X_plot));
% for i= 1:size(X_plot,1)
%     [r,dr] = DS(X_plot(i,:),params);
%     dr = dirsgn*dr;
%     Y(i,1:N) = sph2cartvelocities(r,dr);
% end
% 
% streamslice(Xs,Ys,reshape(Y(:,1),resol,resol),...
% reshape(Y(:,2),resol,resol),'method','cubic');
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
%========================
% hold on;
% fig2 = figure;
% plot(Xdata(:,1),Xdata(:,2),'r.'); hold on;
% Ygmm = zeros(size(X_plot));
% for i= 1:size(X_plot,1)  
%     fprintf('in process:% .2f \n',100*i/size(X_plot,1));
%     [r_gmm,dr_gmm] = DSGMM(X_plot(i,:),params,rho_GMModel);
%     dr_gmm = dirsgn*dr_gmm;
%     Ygmm(i,1:N) = sph2cartvelocities(r_gmm,dr_gmm);
% end
% 
% streamslice(Xs,Ys,reshape(Ygmm(:,1),resol,resol),...
% reshape(Ygmm(:,2),resol,resol),'method','cubic');
% ax = gca;
% ax.FontSize = 14; 
% ylim(yl);
% xlim(xl);
% ylabel('$\xi_2$','Interpreter','latex','FontSize',18,'FontWeight','bold');
% xlabel('$\xi_1$','Interpreter','latex','FontSize',18,'FontWeight','bold');
% hold on;
% % grid on;
% % Test dynamics for T time steps
% X0 = 1.5*Xdata(1,:);
% X_s = []; Xvel_s = [];
% for j = 1:size(X0,1)
%     X = X0(j,:);
%     for i = 1:T(1)
%         X_prev = X;
%         %%%%%% FUNCTION TO CALCULATE POLAR/SPHERICAL VELOCITIES: %%%%%%
%         [r,dr] = DSGMM(X,params,rho_GMModel);
% %         [r,dr] = DSPoly(X,params,rho_polyModel);
%         %%%%%% INTEGRATE THE DS TO GET NEXT POLAR/SPHERICAL POSITION: %%%%%%
%         next_r = r + dr*dt;
%         % Get next cartesian position
%         X = (Rrot*(hyper2cart(next_r)./a)')' - x0;
%         X_s = [X_s; X];
%         Xvel_s = [Xvel_s; sph2cartvelocities(r,dr)];
%         if N == 2
%             plot(X(1),X(2),'k.','LineWidth',2,'MarkerSize',10); hold on; grid on;
%         else
%             plot3(X(1),X(2),X(3),'k.'); hold on; grid on;
%         end
%     end
% end

%% get Movie:

fig3 = figure;
h(1) = plot(Xdata(:,1),Xdata(:,2),'r.'); hold on;
ax = gca;
ax.FontSize = 14; 
ylim(yl);
xlim(xl);
ylabel('$\xi_2$','Interpreter','latex','FontSize',18,'FontWeight','bold');
xlabel('$\xi_1$','Interpreter','latex','FontSize',18,'FontWeight','bold');
hold on;

    colorlist = (1/256)*[218,112,214;...
                160 82 45;...
                255,140,0;...
                34,139,34;...
                255,215,0;...
                25,25,112;...
                32,178,170;...
                0,0,0];
     
    traj =[];
    for j =1:size(Xint,2)
        traj(j).X = Xint(:,j)';
        traj(j).Xs = traj(j).X;
    end
    
    cout = 0;
    for i = 1:dt:10
        cout = cout +1;
        delete(h);
        hold on;
        for j =1:size(Xint,2)
            [r,dr] = DSGMM(traj(j).X,params,rho_GMModel);
            next_r = r + dr*dt;
        % Get next cartesian position
            traj(j).X = (Rrot*(hyper2cart(next_r)./a)')' - x0;
        end
       for j =1:size(Xint,2)
            h(2*j)=plot(traj(j).Xs(:,1),traj(j).Xs(:,2),'Color',colorlist(j,:),'LineWidth',3);
       end
        for j =1:size(Xint,2)
            h(2*j+1)=scatter(traj(j).X(1),traj(j).X(2),80,'filled','MarkerFaceColor',colorlist(j,:)); grid on;
            traj(j).Xs = [traj(j).Xs;traj(j).X];
        end
        hold off;
    
        Fr(cout) = getframe(gcf);
    
        pause(0.001);
    end
% end
delete(h);
close(gcf)
video = VideoWriter(nameVideo);
video.FrameRate = 30;
open(video)
writeVideo(video,Fr);
close(video)
