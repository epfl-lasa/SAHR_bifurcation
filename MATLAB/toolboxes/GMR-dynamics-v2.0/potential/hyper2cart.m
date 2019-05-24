function [x] = hyper2cart(r)
%HYPER2CART Transforms hyperspherical coordinates to cartesian
% Inputs:
%   r : MxN     hyperspherical coordinates (for M datapoints in N dim)
% Output:
%   x : MxN    cartesian coordinates

x = zeros(size(r));
[~,N] = size(r);

x(:,1) = r(:,1) .* cos(r(:,2)) .* sign(r(:,2));
for i = 2:N-1
   x(:,i) = r(:,1) .* cos(r(:,i+1)) .* sign(r(:,i+1)) .* prod(sin(r(:,2:i)),2);
end
x(:,N) = r(:,1) .* prod(sin(r(:,2:N)),2);

end

