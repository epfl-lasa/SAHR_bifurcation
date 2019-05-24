function [error] = RMSErr(data,vel,params,Rrot,dt,normalized)
%RMSErr Prediction Root Mean Square Error to evualuate fit of learned DS
%   INPUT: 
%       data, vel : MxN (#points x #dimensions) data trajectories and
%           velocities
%       params : structure containing learned DS parameters
%       Rrot : rotation matrix
%       dt : timestep size
%   OUTPUT:
%       error : RMS Error for fit

[M,N] = size(data);
dx = zeros(M,N);
for i=1:M
    [r,dr] = DS(data(i,:),params,Rrot);
    x = (Rrot * (hyper2cart(r + dr*dt) ./ params.a)')' - params.x0;
    dx(i,:) = (x - data(i,:)) ./ dt ;
end

if normalized == 1
    error = 1/M * sum(vecnorm((vel - dx),2,2))./ max(max(vel) - min(vel));
else
    error = 1/M * sum(vecnorm(vel - dx,2,2));
end

end

