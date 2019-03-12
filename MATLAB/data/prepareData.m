function [Xdata,Xvel,Rdata,Rvel,dt,T,N,m,begin] = prepareData(type,data,time,smoothing,T)
%PREPAREDATA Get loaded data and prepare it for optimization and plotting.
%   type: type of saved data
%   data: data points
%   (time: timestamp of data points)
%   smoothing: if 0: no smoothing; else: value to smooth for

if (type == 1)
    dt = 0.01;
    m = size(data,1)/T;
    T = repmat(T,1,m);
    N = size(data,2)/2;
    
    % Get cartesian positions
    Xdata = data(:,1:N);
    
    % Find initial point of each trajectory
    begin = ones(1,m);
    for i = 1:m-1
        begin(1,i+1) = sum(T(1:i))+1;
    end
    % Get cartesian velocities
    Xvel = subvelocities(Xdata,dt,begin);
    
elseif(type == 2)
    Xdata = data;
    m = 1;
    dt = mean(diff(time));
    [T,N] = size(Xdata);
    
    % Smooth data if needed
    if smoothing > 0
        for i = 1:N
            Xdata(:,i) = smooth(Xdata(:,i), smoothing);
        end
    end
    
    % Get cartesian velocities and adjust Xdata
    Xvel = diff(Xdata) ./ dt;
    Xdata = Xdata(1:end-1,:);
    T = T-1;
    begin = 1;
    
elseif(type == 3)
    if (exist('time','var'))
        dt = mean(diff(time));
    else
        dt = 1/50;                          % with camera frequency of 50Hz
    end
    m = size(T,2);
    N = size(data,2);
    % Smooth data if needed
    if smoothing > 0
        start = 1;
        for i = 1:m
            for j = 1:N
                data(start:sum(T(1:i))+i-1,j) = ...
                    smooth(data(start:sum(T(1:i))+i-1,j),smoothing);
            end
            start = sum(T(1:i-1))+i+1;
        end
    end
    
    % Get positions and velocities
    Xdata = zeros(sum(T(1:m)),N);
    Xvel = zeros(sum(T(1:m)),N);
    start = 1;
    for i = 1:m
        Xdata(start:sum(T(1:i)),:) = data(start+i:sum(T(1:i))+i,:);
        Xvel(start:sum(T(1:i)),:) = diff(data(start+i:sum(T(1:i))+i,:)) ./ dt;
        start = sum(T(1:i-1))+1;
    end
    
    % Find initial point of each trajectory
    begin = ones(1,m);
    for i = 1:m-1
       begin(1,i+1) = sum(T(1:i))+1;
    end
    % Find 
end

% Get polar/spherical positions and velocities
Rdata = cart2hyper(Xdata);
Rvel = cart2sphvelocities(Xdata,Xvel,Rdata,begin);

end

