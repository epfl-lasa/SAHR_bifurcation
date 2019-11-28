function [obj1] = objective1(parsOpt,parsFixed,Xdata,first,N,opts)
%OBJECTIVE1 Function for the first objective function. Allows to select which
%paramaters to optimize through the input "first".
%   parsOpt:    parameters M (size 1), rho0 (size 1), R (size 1), a (size N),
%               x0 (size N) concatenated in one vector; to be optimized.
%   parsFixed:  parameters M (size 1), rho0 (size 1), R (size 1), a (size N),
%               x0 (size N) concatenated in one vector; to be used "as is".
%   first:      vector of 5 booleans values selecting whether to optimize the 
%               corresponding parameter (in order: M, rho0, R, a, x0).
%   N:          size of the data
%   opts:       other parameters passed to the function

% Function handle for the conversion from cartesian to polar/spherical 
% coordinates and velocities, depending on whether a and x0 should be opt:
if(~first(4) && ~ first(5))
    Rdatas = @(a,x0) cart2hyper(parsFixed(4:N+3).*(Xdata+parsFixed(N+4:N*2+3)));
    Rvels = @(a,x0) cart2sphvelocities(parsFixed(4:N+3).*(Xdata+parsFixed(N+4:N*2+3)),...
        subvelocities(parsFixed(4:N+3).*(Xdata+parsFixed(N+4:N*2+3)),opts.dt,opts.begin),...
        Rdatas(parsFixed(4:N+3),parsFixed(N+4:N*2+3)),opts.begin);
elseif(~first(4))
    Rdatas = @(a,x0) cart2hyper(parsFixed(4:N+3).*(Xdata+x0));
    Rvels = @(a,x0) cart2sphvelocities(parsFixed(4:N+3).*(Xdata+x0),...
        subvelocities(parsFixed(4:N+3).*(Xdata+x0),opts.dt,opts.begin),...
        Rdatas(parsFixed(4:N+3),x0),opts.begin);
elseif(~first(5))
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
Rvelsrho = @(Rvel) Rvel(:,1);

% Function handle for differential eq in rho (polar/spherical radius)
dRho = @(r,M,rho0,R) - sqrt(M.*2) .* (r(:,1) - rho0);

% Function handle for loss, depending on which parameter has to be
% optimized (R never optimized):
if(~first(1) && ~first(2))
    disp('Warning: rho0 not optimized!');
    obj1 = norm(dRho(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsFixed(1),parsFixed(2),parsFixed(3)) - ...
            Rvelsrho(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
elseif(~first(1))
    obj1 = norm(dRho(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsFixed(1),parsOpt(2),parsFixed(3)) - ...
            Rvelsrho(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
elseif(~first(2))
    disp('Warning: rho0 not optimized!');
    obj1 = norm(dRho(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsOpt(1),parsFixed(2),parsFixed(3)) - ...
            Rvelsrho(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
else
    obj1 = norm(dRho(Rdatas(parsOpt(4:N+3),parsOpt(N+4:N*2+3)),...
            parsOpt(1),parsOpt(2),parsFixed(3)) - ...
            Rvelsrho(Rvels(parsOpt(4:N+3),parsOpt(N+4:N*2+3))))^2;
end

end

