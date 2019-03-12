function [dx] = subvelocities(x,dt,T)
%CARTVELOCITIES Finds *** velocities from *** coordinates
% Inputs:
%   x : MxN     cartesian/spherical coordinates (for M datapoints in N dim)
% Output:
%   dx : MxN    cartesian/spherical velocities

dx = zeros(size(x));
[M,~] = size(x);

if M < 2
    disp("Error: function requires at least 2 datapoints!");
else
    for i = 1:M
        if any(i == T-1) || i == M
            dx(i,:) = dx(i-1,:);
        else
            dx(i,:) = (x(i+1,:)-x(i,:)) ./ dt;
        end
    end
end
