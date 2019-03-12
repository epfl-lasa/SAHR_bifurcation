function [] = plotData(Xdata,Xvel,Rdata,Rvel,T,m)
%PLOTDATA Plots the data example

[N,~] = size(Xdata);

figure; hold on;
    if N == 2
        subplot(2,2,1); hold on; grid on; title('Polar coordinates');
        xlabel('\rho'); ylabel('\theta');
        subplot(2,2,2); hold on; grid on; title('Polar velocities');
        xlabel('d\rho'); ylabel('d\theta_1');
        subplot(2,2,3); hold on; grid on; title('Cartesian coordinates');
        xlabel('x'); ylabel('dx');
        subplot(2,2,4); hold on; grid on; title('Cartesian velocities');
        xlabel('dx'); ylabel('d(dx)');
    else
        subplot(2,2,1); hold on; grid on; view(3);
        title('Spherical coordinates');
        xlabel('\rho'); ylabel('\theta_1'); zlabel('\theta_2');
        subplot(2,2,2); hold on; grid on; view(3);
        title('Spherical velocities');
        xlabel('d\rho'); ylabel('d\theta_1'); zlabel('d\theta_2');
        subplot(2,2,3); hold on; grid on; view(3);
        title('Cartesian coordinates');
        xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
        subplot(2,2,4); hold on; grid on; view(3);
        title('Cartesian velocities');
        xlabel('dx_1'); ylabel('dx_2'); zlabel('dx_3');
    end
    
    % Plot of data in original and polar/spherical space
    start = 0;
    for i=1:m
        if N == 2
            subplot(2,2,1); % Polar coordinates
            plot(Rdata(start+1:start+T(i),1),Rdata(start+1:start+T(i),2));
            
            subplot(2,2,2); % Polar velocities
            plot(Rvel(start+1:start+T(i),1),Rvel(start+1:start+T(i),2));
            
            subplot(2,2,3); % Cartesian coordinates
            plot(Xdata(start+1:start+T(i),1),Xdata(start+1:start+T(i),2));
            
            subplot(2,2,4); % Cartesian velocities
            plot(Xvel(start+1:start+T(i),1),Xvel(start+1:start+T(i),2));
        else
            subplot(2,2,1); % Spherical coordinates
            plot3(Rdata(start+1:start+T(i),1),Rdata(start+1:start+T(i),2),...
                Rdata(start+1:start+T(i),3));
            
            subplot(2,2,2); % Spherical velocities
            plot3(Rvel(start+1:start+T(i),1),Rvel(start+1:start+T(i),2),...
                Rvel(start+1:start+T(i),3));
            
            subplot(2,2,3); % Cartesian coordinates
            plot3(Xdata(start+1:start+T(i),1),Xdata(start+1:start+T(i),2),...
                Xdata(start+1:start+T(i),3));
            
            subplot(2,2,4); % Cartesian velocities
            plot3(Xvel(start+1:start+T(i),1),Xvel(start+1:start+T(i),2),...
                Xvel(start+1:start+T(i),3));
        end
        start = start + T(i);
    end
    
end

