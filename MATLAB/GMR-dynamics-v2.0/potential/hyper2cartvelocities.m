function [dx] = hyper2cartvelocities(r,dr)
%HYPER2CARTVELOCITIES Transforms hyperspherical velocities to cartesian
% Inputs:
%   r : MxN     hyperspherical coordinates (for M datapoints in N dim)
%   dr : MxN    hyperspherical velocities
% Output:
%   dx : MxN    cartesian velocities

[M,N] = size(r);
dx = zeros(M,N);
tmp = zeros(1,N);

% First line of matrix
tmp(1,1:2) = [cos(r(2)), -r(1) * sin(r(2))];
dx(:,1) = dr * tmp';
tmp = zeros(1,N);
% Middle lines of matrix
for i = 2:N-1
    for j = 1:i+1
        if j == 1
            tmp(1,j) = cos(r(i+1)) * prod(sin(r(2:i)));
        elseif j == i+1
            tmp(1,j) = -r(1) * prod(sin(r(2:i+1)));
        else
            tmp(1,j) = r(1) * cos(r(j)) * cos(r(i+1));
            if j > 2
                tmp(1,j) = tmp(1,j) * prod(sin(r(2:j-1)));
            end
            if j < i
                tmp(1,j) = tmp(1,j) * prod(sin(r(j+1:i)));
            end
        end
    end
    dx(:,i) = dr * tmp';
    tmp = zeros(1,N);
end
% Last line of matrix
for j = 1:N
    if j == 1
        tmp(1,j) = prod(sin(r(2:N)));
    else
        tmp(1,j) = r(1) * cos(r(j));
        if j > 2
            tmp(1,j) = tmp(1,j) * prod(sin(r(2:j-1)));
        end
        if j < N
            tmp(1,j) = tmp(1,j) * prod(sin(r(j+1:N)));
        end
    end
end
dx(:,N) = dr * tmp';

end

