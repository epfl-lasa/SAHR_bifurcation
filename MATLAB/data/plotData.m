function [] = plotData(Xdata,Xvel,Rdata,Rvel,T,m,type)
%PLOTDATA Plots data trajectories in cartesian coordinates from (Xdata,
%Xvel) and in polar/spherical coordinates for (Rdata,Rvel).
% INPUT:    Xdata: MxN (datapoints x dimensions) data in cartesian coord
%           Xvel: MxN velocities for Xdata
%           Rdata: MxN datapoints in polar/spherical coordinates
%           Rvel: MxN velocities for Rdata
%           T: vector of sizes of each of the m trajectories
%           m: number of trajectories
%           type: decides what to plot; can be 'all', 'cart' or 'sph'

[N,~] = size(Xdata);

if strcmp(type,'all')
    p1 = 2;
    p2 = 2;
elseif strcmp(type,'cartp')
    p1 = 1;
    p2 = 1;
else
    p1 = 1;
    p2 = 2;
end
p = 1;

figure; hold on;
if N == 2
    if strcmp(type,'all') || strcmp(type,'sph')
        subplot(p1,p2,p); hold on; grid on; title('Polar coordinates');
        xlabel('\rho'); ylabel('\theta');
        subplot(p1,p2,p+1); hold on; grid on; title('Polar velocities');
        xlabel('d\rho'); ylabel('d\theta_1');
        p = p + 2;
    end
    if ~strcmp(type,'sph')
        subplot(p1,p2,p);hold on; grid on; title('Cartesian coordinates');
        xlabel('x_1'); ylabel('x_2');
        if ~strcmp(type,'cartp')
            subplot(p1,p2,p+1); hold on; grid on; title('Cartesian velocities');
            xlabel('dx_1'); ylabel('dx_2');
        end
    end
else
    if strcmp(type,'all') || strcmp(type,'sph')
        subplot(p1,p2,p); hold on; grid on; view(3);
        title('Spherical coordinates');
        xlabel('\rho'); ylabel('\theta_1'); zlabel('\theta_2');
        subplot(p1,p2,p+1); hold on; grid on; view(3);
        title('Spherical velocities');
        xlabel('d\rho'); ylabel('d\theta_1'); zlabel('d\theta_2');
        p = p + 2;
    end
    if ~strcmp(type,'sph')
        subplot(p1,p2,p); hold on; grid on; view(3);
        title('Cartesian coordinates');
        xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
        if ~strcmp(type,'cartp')
            subplot(p1,p2,p+1); hold on; grid on; view(3);
            title('Cartesian velocities');
            xlabel('dx_1'); ylabel('dx_2'); zlabel('dx_3');
        end
    end
end

% Plot of data in original and polar/spherical space
start = 0;
p = 1;
for i=1:m
    if N == 2
        if strcmp(type,'all') || strcmp(type,'sph')
            subplot(p1,p2,p); % Polar coordinates
            plot(Rdata(start+1:start+T(i),1),Rdata(start+1:start+T(i),2),'r');
            
            subplot(p1,p2,p+1); % Polar velocities
            plot(Rvel(start+1:start+T(i),1),Rvel(start+1:start+T(i),2),'r');
            
            p = p + 2;
        end
        
        if ~strcmp(type,'sph')
            subplot(p1,p2,p); % Cartesian coordinates
            plot(Xdata(start+1:start+T(i),1),Xdata(start+1:start+T(i),2),'r');
            
            if ~strcmp(type,'cartp')
                subplot(p1,p2,p+1); % Cartesian velocities
                plot(Xvel(start+1:start+T(i),1),Xvel(start+1:start+T(i),2),'r');
            end
        end
    else
        if strcmp(type,'all') || strcmp(type,'sph')
            subplot(p1,p2,p); % Spherical coordinates
            plot3(Rdata(start+1:start+T(i),1),Rdata(start+1:start+T(i),2),...
                Rdata(start+1:start+T(i),3),'r');
            
            subplot(p1,p2,p+1); % Spherical velocities
            plot3(Rvel(start+1:start+T(i),1),Rvel(start+1:start+T(i),2),...
                Rvel(start+1:start+T(i),3),'r');
            
            p = p + 2;
        end
        
        if ~strcmp(type,'sph')
            subplot(p1,p2,p); % Cartesian coordinates
            plot3(Xdata(start+1:start+T(i),1),Xdata(start+1:start+T(i),2),...
                Xdata(start+1:start+T(i),3),'r');
            
            if ~strcmp(type,'cartp')
                subplot(p1,p2,p+1); % Cartesian velocities
                plot3(Xvel(start+1:start+T(i),1),Xvel(start+1:start+T(i),2),...
                    Xvel(start+1:start+T(i),3),'r');
            end
        end
    end
    start = start + T(i);
end

end

