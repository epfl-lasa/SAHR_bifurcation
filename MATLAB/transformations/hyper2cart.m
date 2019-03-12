function [x] = hyper2cart(r)
%HYPER2CART Transforms hyperspherical coordinates to cartesian
% Inputs:
%   r : MxN    hyperspherical coordinates (for M datapoints in N dim)
% Output:
%   x : MxN    cartesian coordinates

x = zeros(size(r));
[~,N] = size(r);

x(:,1) = r(:,1) .* prod(cos(r(:,2:N)),2);
for i = 2:N-1
   x(:,i) = r(:,1) .* prod(cos(r(:,2:N-i+1)),2) .* sin(r(:,N-i+2));
end
x(:,N) = r(:,1) .* sin(r(:,2));

end

