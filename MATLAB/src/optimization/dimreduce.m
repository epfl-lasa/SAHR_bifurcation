function [data2,transf] = dimreduce(data,method,opts)
%DIMREDUCE Ask for data.m input and reduces it using various techniques:
%INPUT
%   data : N-dimensional original data
%   method : string to select method; 'PCA', 'kPCA', or 'ICA'
%   opts : used to store variables such as flag for plots, kPCA kernel
%   width, etc.
%OUTPUT
%   data2 : N-dimensional projected data
%   transf : transformation matrix   
%DEPENDENCE:    drtoolbox (pca, kernel_pca)

transf = [];

if(~exist('method','var') || isempty(method))
    method = 'PCA';
end

% If data not passed to the function, ask for data input (only .mat files displayed)
if(exist('data','var') && ~isempty(data))
    N = size(data,2);
else
    disp('Please select data file');
    uiopen('*.mat');
    if(exist('Xstored','var'))
        N = size(Xstored,2)/2;
        data = Xstored(:,1:N);
    elseif(exist('trial','var'))
        data = [trial.position_real.x, trial.position_real.y, trial.position_real.z];
        N = size(data,2);
    elseif(exist('traj','var'))
        data = traj.data;
        N = size(data,2);
    else
        data = [];
        disp('Format not recognized; please select another data file.');
        return
    end
end
    
% Performs algorithm and returns transformation matrix
if(strcmp(method,'PCA'))
    % From matlab:
%     [transf,data2,eigenvalues] = pca(data);
%     disp(eigenvalues);
    % From drtoolbox:
    if(isfield(opts,'mean'))
        [data2,transf] = pca(data,N,opts.mean);
    else
        [data2,transf] = pca(data,N);
    end
elseif(strcmp(method,'kPCA'))
    if(isfield(opts,'kwidth'))
        sigma = opts.sigma;
    else
        sigma = 1;
    end
    [data2, transf] = kernel_pca(data, N, true, 'gauss', sigma);
elseif(strcmp(method,'ICA'))
    mdl = rica(data,N);
    data2 = transform(mdl,data);
    transf = mdl.TransformWeights;
else
    disp('Method not available; choose between "PCA", "kPCA" or "ICA".');
    return
end

% Plot original and projected data
if(~isfield(opts,'plots'))
   opts.plots = 1;
   disp("Plots displayed automatically; to avoid plots, use opts.plots = 0.");
end
if(opts.plots == 1)
    figure; hold on;
    subplot(1,2,1); hold on; grid on; title('Original data');
    xlabel('x_1'); ylabel('x_2');
    if N == 2
        plot(data(:,1),data(:,2),'.');
    else
        zlabel('x_3'); view(3);
        plot3(data(:,1),data(:,2),data(:,3),'.');
    end
    subplot(1,2,2); hold on; grid on; title('Projected data');
    xlabel('e_1'); ylabel('e_2');
    if N == 2
        plot(data2(:,1),data2(:,2),'.');
    else
        zlabel('e_3'); view(3);
        plot3(data2(:,1),data2(:,2),data2(:,3),'.');
    end
end

end

