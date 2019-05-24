function [r,dr] = DS(x,params,Rrot)
%DS Calculates velocity dr for x, using parameters in params
% INPUT:
%   x: current position, vecotr with 1xN dimensions (either 2 or 3)
%   params: parameters rho0,M,R,a,x0,theta0 for the DS
% OUTPUT:
%   r: current position in polar/spherical coordinates
%   dr: velocities given by DS

if ~isempty(params)
    
    N = size(x,2);
    
    rho0 = params.rho0;
    M = params.M;
    R = params.R;
%     if N == 3
%         Rrot = eul2rotm(params.theta0);
%     else
%         Rrot = [cos(params.theta0), sin(params.theta0);
%             -sin(params.theta0), cos(params.theta0)];
%     end
    a = Rrot; for i = 1:N; a(i,i) = a(i,i) / params.a(i); end
    x0 = params.x0;
    
    r = cart2hyper((a\(x+x0)')');
    if N == 3
        dr = [-sqrt(M.*2) .* (r(:,1) - rho0), -sqrt(M.*2) .* r(:,2),...
            R .* exp(-4.*M^2.*(r(:,1) - rho0).^2)];
    else
        dr = [-sqrt(M.*2) .* (r(:,1) - rho0), R .* exp(-4.*M^2.*(r(:,1) - rho0).^2)];
    end

else
    
    r = cart2hyper(x);
    dr = [];
    disp('Please provide parameters for the DS to calculate velocities');
    
end

end

