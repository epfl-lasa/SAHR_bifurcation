function [dx] = sph2cartvelocities(r,dr)
%SPH2CARTVELOCITIES Converts spherical velocities to cartesian
% Inputs:
%   r : MxN     spherical coordinates (for M datapoints in N dim)
%   dr : MxN    spherical velocities
% Output:
%   dx : MxN    cartesian velocities

dx = zeros(size(r));
[~,N] = size(r);

if N == 2
    dx(:,1) = cos(r(:,2)).*dr(:,1) - r(:,1).*sin(r(:,2)).*dr(:,2);
    dx(:,2) = sin(r(:,2)).*dr(:,1) + r(:,1).*cos(r(:,2)).*dr(:,2);
else
    dx(:,1) = cos(r(:,2)).*cos(r(:,3)).*dr(:,1) - ...
        r(:,1).*sin(r(:,2)).*cos(r(:,3)).*dr(:,2) - ...
        r(:,1).*cos(r(:,2)).*sin(r(:,3)).*dr(:,3);
    
    dx(:,2) = cos(r(:,2)).*sin(r(:,3)).*dr(:,1) - ...
        r(:,1).*sin(r(:,2)).*sin(r(:,3)).*dr(:,2) + ...
        r(:,1).*cos(r(:,2)).*cos(r(:,3)).*dr(:,3);
    
    dx(:,3) = sin(r(:,2)).*dr(:,1) + ...
        r(:,1).*cos(r(:,2)).*dr(:,2);
end

end

