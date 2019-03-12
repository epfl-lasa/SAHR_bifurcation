function [r] = cart2hyper(x)
%HYPER2CART Transforms cartesian coordinates to hyperspherical
% Inputs:
%   x : MxN     cartesian coordinates (for M datapoints in N dim)
% Output:
%   r : MxN     hyperspherical coordinates

r = zeros(size(x));
[~,N] = size(x);

r(:,1) = sqrt(sum(x.^2,2));
for i = 1:N-1
   r(:,i+1) = atan2(x(:,N-i+1),sqrt(sum(x(:,1:N-i).^2,2)));
end

% for i = 2:N-1
%     r(:,i) = acos(x(:,i-1)./sqrt(sum(x(:,i-1:N).^2,2)));
% end
% r(:,N) = sign(x(:,N)) .* acos(x(:,N-1)./sqrt(sum(x(:,N-1:N).^2,2)));

end

