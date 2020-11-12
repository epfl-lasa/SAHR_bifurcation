function [r,dr] = DSGMM(x,params,rho_gmm)
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
    
    hi = zeros(1,rho_gmm.NumComponents);
    ki = zeros(1,rho_gmm.NumComponents);
    Ri = zeros(1,rho_gmm.NumComponents);
    gi = zeros(1,rho_gmm.NumComponents);
    r_scale = 0;
    rdot_scale = 0;
    for i =1:rho_gmm.NumComponents
        hi(i) = rho_gmm.ComponentProportion(i) * normpdf(theta,rho_gmm.mu(i,1),sqrt(rho_gmm.Sigma(1,1,i)));
        Ri(i) = -1* (theta - rho_gmm.mu(i,1))/rho_gmm.Sigma(1,1,i);
        ki(i) = Ri(i)*hi(i);
        gi(i) = rho_gmm.mu(i,2) - rho_gmm.Sigma(2,1,i) * Ri(i);
        r_scale = r_scale + hi(i)*gi(i);
    end
    alpha = sum(hi);
    r_scale = r_scale / alpha;
    gamma = sum(ki);
    for i =1:rho_gmm.NumComponents
        rdot_scale = rdot_scale + hi(i)* ( (Ri(i) - gamma/alpha)*gi(i) + rho_gmm.Sigma(2,1,i)/rho_gmm.Sigma(1,1,i));
    end
    
    if N == 3
        dr = [-sqrt(M.*2) .* (r(:,1) - r_scale*rho0), -sqrt(M.*2) .* r(:,2),...
            R .* exp(-4.*M^2.*(r(:,1) - r_scale*rho0).^2)];
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

