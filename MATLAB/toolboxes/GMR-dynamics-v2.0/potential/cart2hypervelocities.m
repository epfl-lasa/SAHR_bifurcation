function [dr] = cart2hypervelocities(x,dx,r)
%HYPER2CART Transforms cartesian velocities to hyperspherical
% Inputs:
%   x : MxN     cartesian coordinates (for M datapoints in N dim)
%   dx : MxN    cartesian velocities
%   r : MxN     hyperspherical coordinates
% Output:
%   dr : MxN    hyperspherical velocities

dr = zeros(size(x));
[~,N] = size(x);

dr(:,1) = sum(x.*dx,2) ./ sqrt(sum(x.^2,2));
for i = 2:N-1
    dr(:,i) = (x(:,i-1).*(sum(x(i-1:N).*dx,2)) - dx(:,i-1).*sum(x(:,i-1:N).^2,2)) ./ ...
        (sqrt(sum(x(:,i-1:N).^2,2)).*sqrt(sum(x(:,i:N).^2,2)));
end
dr(:,N) = sqrt(sum(x.^2,2)) .* prod(sin(r(:,2:N-1)),2) .* (dx(:,N).*x(:,N-1) - ...
    x(:,N).*dx(:,N-1)) ./ sum(x(:,N-1:N).^2,2);

end

