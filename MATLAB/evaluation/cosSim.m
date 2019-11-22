function [error] = cosSim(data,vel,params,dt)
%COSSIM Prediction Cosine Similarity to evualuate fit of learned DS
%   INPUT: 
%       data, vel : MxN (#points x #dimensions) data trajectories and
%           velocities
%       params : structure containing learned DS parameters
%       Rrot : rotation matrix
%       dt : timestep size
%   OUTPUT:
%       error : Cosine Similarity Error for fit

[M,~] = size(data);
err = zeros(M,1);

for i=1:M
    [r,dr] = DS(data(i,:),params);
    x = (params.Rrot * (hyper2cart(r + dr*dt) ./ params.a)')' - params.x0;
    dx = (x - data(i,:)) ./ dt;
%     dx = sph2cartvelocities(r,dr);
    err(i) = (dx * vel(i,:)') / (norm(vel(i,:))*norm(dx));
end

error = 1/M * sum(abs(1 - err));

end

