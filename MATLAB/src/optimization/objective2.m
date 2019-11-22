function [obj2] = objective2(parsOpt,parsFixed,Xdata,second,N,opts)
%OBJECTIVE2 Function for the second objective function. Allows to select
%which paramaters to optimize through the input "second".
%   parsOpt:    parameters M (size 1), rho0 (size 1), R (size 1), a (size N),
%               x0 (size N) concatenated in one vector; to be optimized.
%   parsFixed:  parameters M (size 1), rho0 (size 1), R (size 1), a (size N),
%               x0 (size N) concatenated in one vector; to be used "as is".
%   second: vector of 5 booleans values selecting whether to optimize the 
%           corresponding parameter (in order: M, rho0, R, a, x0).
%   N:      size of the data
%   opts:       other parameters passed to the function

% Function handle for the conversion from cartesian to polar/spherical 
% coordinates and velocities, depending on whether a and x0 should be opt:
if(~second(4) && ~ second(5))
    Rdatas = @(a,x0) cart2hyper(parsFixed(4:N+3).*(Xdata+parsFixed(N+4:N*2+3)));
    Rvels = @(a,x0) cart2sphvelocities(parsFixed(4:N+3).*(Xdata+parsFixed(N+4:N*2+3)),...
        subvelocities(parsFixed(4:N+3).*(Xdata+parsFixed(N+4:N*2+3)),opts.dt,opts.begin),...
        Rdatas(parsFixed(4:N+3),parsFixed(N+4:N*2+3)),opts.begin);
elseif(~second(4))
    Rdatas = @(a,x0) cart2hyper(parsFixed(4:N+3).*(Xdata+x0));
    Rvels = @(a,x0) cart2sphvelocities(parsFixed(4:N+3).*(Xdata+x0),...
        subvelocities(parsFixed(4:N+3).*(Xdata+x0),opts.dt,opts.begin),...
        Rdatas(parsFixed(4:N+3),x0),opts.begin);
elseif(~second(5))
    Rdatas = @(a,x0) cart2hyper(a.*(Xdata+parsFixed(N+4:N*2+3)));
    Rvels = @(a,x0) cart2sphvelocities(a.*(Xdata+parsFixed(N+4:N*2+3)),...
        subvelocities(a.*(Xdata+parsFixed(N+4:N*2+3)),opts.dt,opts.begin),...
        Rdatas(a,parsFixed(N+4:N*2+3)),opts.begin);
else
    Rdatas = @(a,x0) cart2hyper(a.*(Xdata+x0));
    Rvels = @(a,x0) cart2sphvelocities(a.*(Xdata+x0),...
        subvelocities(a.*(Xdata+x0),opts.dt,opts.begin),...
        Rdatas(a,x0),opts.begin);
end
Rvelsth = @(Rvel) Rvel(:,end);

% Function handle for differential eq in theta (polar/azimuth angle)
dTheta = @(r,M,rho0,R) R .* exp(-(2.*M.*(r(:,1) - rho0)).^2);

% Function handle for loss, depending on which parameter has to be
% optimized (rho0 never optimized):
if(~second(1) && ~second(3))
    disp('Warning: R not optimized!');
    obj2 = norm(dTheta(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsFixed(1),parsFixed(2),parsFixed(3)) - ...
            Rvelsth(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
elseif(~second(1))
    obj2 = norm(dTheta(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsFixed(1),parsFixed(2),parsOpt(3)) - ...
            Rvelsth(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
elseif(~second(3))
    disp('Warning: R not optimized!');
    obj2 = norm(dTheta(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsOpt(1),parsFixed(2),parsFixed(3)) - ...
            Rvelsth(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
else
    obj2 = norm(dTheta(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsOpt(1),parsFixed(2),parsOpt(3)) - ...
            Rvelsth(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
    
end

