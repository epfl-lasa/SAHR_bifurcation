%% Circular and parabolic motion around a star 

clear all;
close all;

%% Plot motion

% Polar coordinates: r = rho and theta
r = [2 0.1 0.4];             % initial rho and theta
M = 2;                   % 'mass' of planet
rho0 = 0;                  % radius of limit cycle
R = [-1 0.1];                  % sign of R changes direction of rotation
N = 3;

%% Plot motion

figure; hold on; grid on;
subplot(2,2,1); hold on; grid on; plot(r(1),r(2),'b*'); title('Polar coordinates');
subplot(2,2,2); hold on; grid on; title('Polar velocities');
subplot(2,2,3); hold on; grid on; 
plot(r(1)*cos(r(2)),r(1)*sin(r(2)),'b*');
title('Cartesian coordinates');
subplot(2,2,4); hold on; grid on; title('Cartesian velocities');

X1 = zeros(1000,N);
Xvel1 = zeros(1000,N);
X1(1,:) = hyper2cart(r);%[r(1)*cos(r(2)),r(1)*sin(r(2))];
R1 = zeros(1000,N); R1(1,:) = r;
Rvel1 = zeros(1000,N);
for i=1:1000
    dU = M*(r(1) - rho0);
    dr = [-2/M*dU, R .* exp(-dU.^2)];
    r = r + dr*0.01;
    if i<1000
        R1(i+1,:) = r;
    end
    Rvel1(i,:) = dr;
    subplot(2,2,1); hold on; grid on; plot(r(1),r(2),'r*');
    subplot(2,2,2); hold on; grid on; 
    if (i==1)
        plot(dr(1),dr(2),'b*');
    else
        plot(dr(1),dr(2),'r*');
    end
    subplot(2,2,3); hold on; grid on;
    %plot(r(1)*cos(r(2)),r(1)*sin(r(2)),'r*');
    if i < 1000
        X1(i+1,:)=hyper2cart(r);%[r(1)*cos(r(2)),r(1)*sin(r(2))];
    end
    Xvel1(i,:) = hyper2cartvelocities(r,dr);
    plot(X1(:,1),X1(:,2),'b*');
        %[dr(1)*cos(r(2))-r(1)*dr(2)*sin(r(2)),dr(1)*sin(r(2))+r(1)*dr(2)*cos(r(2))];
    subplot(2,2,4); hold on; grid on;
    plot(Xvel1(:,1),Xvel1(:,2),'b*');
%     if (i==1)
%         %plot(dr(1)*cos(r(2))-r(1)*dr(2)*sin(r(2)),dr(1)*sin(r(2))+r(1)*dr(2)*cos(r(2)),'b*');
%     else
%         plot(dr(1)*cos(r(2))-r(1)*dr(2)*sin(r(2)),dr(1)*sin(r(2))+r(1)*dr(2)*cos(r(2)),'r*');
%     end
end

% xl = xlim;
% yl = ylim;
% [Xs,Ys] = meshgrid(linspace(xl(1),xl(2),20),linspace(yl(1),yl(2),20));
% X_plot = [Xs(:), Ys(:)];
% r = @(X_plot) [sqrt(X_plot(1).^2+X_plot(2).^2), atan2(X_plot(2),X_plot(1))];
% dU = @(r) M*(r - rho0);
% dr = @(r) [-sqrt(2./M).*dU(r), R * exp(-dU(r).^2)];
% dx = @(r,dr) [dr(1).*cos(r(2))-r(1).*dr(2).*sin(r(2)),...
%     dr(1).*sin(r(2))+r(1).*dr(2).*cos(r(2))];
% y = @(X_plot) dx(r(X_plot),dr(r(X_plot)));
% Y = zeros(size(X_plot));
% for i= 1:size(X_plot,1)
%     Y(i,:) = y(X_plot(i,:));
% end
% quiver(X_plot(:,1),X_plot(:,2),Y(:,1),Y(:,2));
% streamslice(Xs,Ys,reshape(Y(:,1),20,20),reshape(Y(:,2),20,20),'method','cubic');

%% Plot U(r)

U = @(r) M*(r - rho0).^2; %- 50./(r + rho0);
%U = @(r) (0.2*(r<rho0) + (r>=rho0)) .* M.*(r - rho0).^2;

rho = linspace(0,10);
figure; hold on; title(['Potential energy with M = ' num2str(M) ' and \rho_0 = ' num2str(rho0)]);
plot(rho, U(rho)); grid on; hold on;
xlabel('\rho');
ylabel('U(\rho)');

[U0,i] = min(U(rho));
plot(rho(i),U0,'*');

legend('Potential energy','Minimum energy at \rho_0');

%% Plot dRho and dTheta:

dU = @(r) 2.*M.*(r - rho0);
dTheta = @(r) R * exp(-dU(r).^2);
%dTheta = @(r) R.*(1-exp(-M)).*exp(-dU(r));
dRho = @(r) - sqrt(1./(2.*M)) .* dU(r);%-dU(r);

rho = linspace(0,10);
figure; hold on; title(['Centripetal velocity with M = ' num2str(M) ' and \rho_0 = ' num2str(rho0)]);
plot(rho, dRho(rho)); grid on; hold on;
xlabel('\rho');
ylabel('d(\rho)');
figure; hold on; title(['Tangential velocity with M = ' num2str(M) ' and \rho_0 = ' num2str(rho0)]);
plot(rho, dTheta(rho)); grid on; hold on;
xlabel('\rho');
ylabel('d(\theta)');

%% Plot total energy

E = @(r) 1/2 * (dRho(r).^2 + dTheta(r).^2) - U(r);

rho = linspace(0,10);
figure; hold on; title(['Total energy with M = ' num2str(M) ' and \rho_0 = ' num2str(rho0)]);
plot(rho, E(rho)); grid on; hold on;
xlabel('\rho');
ylabel('E(\rho)');


%% Interactive plot

r = @(X_plot) [sqrt(X_plot(1).^2+X_plot(2).^2), atan2(X_plot(2),X_plot(1))];
dU = @(r) M*(r(1) - rho0);
%dU = @(r) (0.5*(r(1)<rho0) + (r(1)>=rho0)) .* M .*(r(1) - rho0);
dr = @(r) [-sqrt(1./(2.*M)).*dU(r), R * exp(-dU(r).^2)];%[-dU(r), R * (1 - exp(-M)) * exp(-dU(r))];
dx = @(r,dr) [dr(1).*cos(r(2))-r(1).*dr(2).*sin(r(2)),...
    dr(1).*sin(r(2))+r(1).*dr(2).*cos(r(2))];
y = @(X_plot) dx(r(X_plot),dr(r(X_plot)));      % dynamics to be plotted

f = figure;
ax = axes('Parent',f,'position',[0.1 0.35  0.85 0.6]);
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
        for i = 1:1000
            dx = y(X);
            X = X + dx*0.01;
            plot(X(1),X(2),'b.');
        end
    end
    
    % add edit for position (polar)
    % add sliders for rho0, M, R and steps
end

