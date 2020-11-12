function [r,dr] = DSPoly(x,params,rho_poly)
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
    xt = a.*(Rrot\(x+x0)')';
    [theta,~]= cart2pol(xt(1),xt(2));
    
    r_scale = polyval(rho_poly,theta);
    der_poly = polyder(rho_poly);
    rdot_scale = polyval(der_poly,theta);
    
    if N == 3
        dr = [-sqrt(M.*2) .* (r(:,1) - rh0), -sqrt(M.*2) .* r(:,2),...
            R .* exp(-4.*M^2.*(r(:,1) - rho0).^2)];
    else
        theta_dot = R .* exp(-4.*M^2.*((r(:,1)/r_scale) - rho0).^2);
        Dbr = -sqrt(M.*2) .* ((r(:,1)/r_scale) - rho0);
        dR = r_scale*Dbr + rdot_scale*theta_dot*(r(:,1)/r_scale);
        dr = [dR, theta_dot];
    end

else
    
    r = cart2hyper(x);
    dr = [];
    disp('Please provide parameters for the DS to calculate velocities');
    
end

end

