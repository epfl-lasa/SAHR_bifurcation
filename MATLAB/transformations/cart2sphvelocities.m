function [dr] = cart2sphvelocities(x,dx,r,T)
%CART2SPHVELOCITIES Transforms cartesian velocities to polar/spherical
% Inputs:
%   x : MxN     cartesian coordinates (for M datapoints in N dim)
%   dx : MxN    cartesian velocities
%   r : MxN     polar/spherical coordinates
%   T : 1xm     initial point of each of m trajectories in x
% Output:
%   dr : MxN    polar/spherical velocities

dr = zeros(size(x));
[~,N] = size(x);

dr(:,1) = sum(x.*dx,2) ./ sqrt(sum(x.^2,2));
if N == 3
% dr(:,2) = r(:,1) .* (dx(:,3).*sqrt(sum(x(:,1:2).^2,2)) - x(:,3).*(sum(x(:,1:2).*dx(:,1:2),2))) ./ ...
%     (sqrt(sum(x(:,1:2).^2,2)).*(sum(x.^2,2)));
% dr(:,2) = (x(:,3).*(sum(x(:,1:3).*dx(:,1:3),2)) - dx(:,3).*sum(x(:,1:3).^2,2)) ./ ...
%     (sqrt(sum(x(:,1:2).^2,2)).*sqrt(sum(x(:,1:3).^2,2)));
dr(:,2) = -(x(:,3).*sum(x(:,1:2).*dx(:,1:2),2) - dx(:,3).*sum(x(:,1:2).^2,2)) ./ ...
    (sum(x(:,1:3).^2,2) .* sqrt(sum(x(:,1:2).^2,2)));%r(:,1) .* (dx(:,3).*r(:,1) - x(:,3).*dr(:,1)) ./ ...
    %(r(:,1) .* sqrt(1 - (x(:,3).^2 ./ r(:,1).^2)));
    change = 0;
    for i = 1:size(x,1)
        if any(i == T)
            if ((r(i,3) < 0) || (abs(r(i,3)) == pi))
                change = 1;
            else
                change = 0;
            end
        else
            if sign(x(i-1,1))~=sign(x(i,1)) && (sign(x(i-1,2))~=sign(x(i,2)) || x(i,2) == 0)
                if change == 1
                    change = 0;
                else
                    change = 1;
                end
            end
        end
        if change == 1
            dr(i,2) = -dr(i,2);
        end
    end
dr(:,3) = (dx(:,2).*(x(:,1)) - x(:,2).*dx(:,1)) ./ ...
    sum(x(:,1:2).^2,2);%r(:,1) .* sin(r(:,2)) .* (dx(:,2).*(x(:,1)) - x(:,2).*dx(:,1)) ./ ...
    %sum(x(:,1:2).^2,2);
else
dr(:,2) = (dx(:,2).*x(:,1) - x(:,2).*dx(:,1)) ./ ...
    sum(x(:,1:2).^2,2);%r(:,1) .* (dx(:,2).*x(:,1) - x(:,2).*dx(:,1)) ./ ...
    %sum(x(:,1:2).^2,2);
end

end