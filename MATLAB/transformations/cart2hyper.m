function [r] = cart2hyper(x)
%HYPER2CART Converts cartesian coordinates to hyperspherical
% Inputs:
%   x : MxN     cartesian coordinates (for M datapoints in N dim)
% Output:
%   r : MxN     hyperspherical coordinates

r = zeros(size(x));
[~,N] = size(x);

r(:,1) = sqrt(sum(x.^2,2));
for i = 1:N-2
   r(:,i+1) = asin(x(:,N-i+1)./sqrt(sum(x(:,1:N-i+1).^2,2)));
end
r(:,N) = atan2(x(:,2),x(:,1));

end

