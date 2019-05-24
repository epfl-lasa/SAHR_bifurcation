function [noise] = getNoiseRange(X,window)
%GETNOISERANGE Finds the range of the noise of the limit cycle data in X

[M,N] = size(X);

wind = window;

x0 = zeros(M-wind*2,N);
for i = wind+1:M-wind
    x0(i-wind,:) = mean(X(i-wind:i+wind,:));
end
figure; plot(x0);

noise = 0;

end

