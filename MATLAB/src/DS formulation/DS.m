function [r,dr] = DS(x,params)
%DS Calculates velocity dr for x, using parameters in params
% INPUT:
%   x: current position, vecotr with 1xN dimensions (either 2 or 3)
%   params: parameters rho0,M,R,a,x0,Rrot for the DS
% OUTPUT:
%   r: current position in polar/spherical coordinates
%   dr: velocities given by DS

if ~isempty(params)
    
    N = size(x,2);
    
    rho0 = params.rho0;
    M = params.M;
    R = params.R;
    a = params.a;
    x0 = params.x0;
    Rrot = params.Rrot;
    
    r = cart2hyper(a.*(Rrot\(x+x0)')');
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

