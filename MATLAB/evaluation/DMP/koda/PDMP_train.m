function DMP = PDMP_train(y, time, DMP, lambda, delta, display_result)
%% Encodes input trajectory y with a periodic DMP
%
% INPUTS:
%   y              ... trajectory data
%   time           ... array of sample times or time between two samples
%   DMP            ... initial DMP parameters (optionally N, a_z, a_x)
%   lambda         ... forgetting factor for recursive regression (usually between
%                       0.97 and 0.995 according to Matlab), default 0.995
%   delta          ... initialization constant for covariance matrix, default 100
%   display_result ... show the graphs for frequency and signal learning
%
%   DMP.Omega      ... if set to 0, PDMP_train learns frequency of the input 
%                      signal with adaptive frequency oscillators
%
% OUTPUT: DMP parameters
%   w      ... DMP weight vector
%   c      ... centers of kernel functions
%   h      ... widths of kernel functions
%   Omega  ... signal frequency
%   goal   ... center of periodic motion
%

%% check input parameters, some are optional
if ~exist('DMP')
  DMP.N = 25;
elseif ~isfield(DMP, 'N')
  DMP.N = 25; 	   % number of basis functions to encode the path
end
if ~isfield(DMP, 'Omega')
  DMP.Omega = 0.0; % signal frequency
end
if ~isfield(DMP, 'a_z')
  DMP.a_z = 48;    % DMP stifness
end
if ~isfield(DMP, 'b_z')
  DMP.b_z = DMP.a_z/4;
end
if ~isfield(DMP, 'a_x')
  DMP.a_x = 2;
end
if ~exist('lambda')
  lambda = 0.995;
end
if ~exist('delta')
  delta = 100;
end
if ~exist('display_result')
  display_result = 0;
end

%% compute derivatives
[NT, NS] = size(y);
if length(time) == 1
  time = linspace(0, (size(y,1)-1)*time, size(y,1));
end
for dof = 1:NS
  dy(:,dof) = gradient(y(:,dof), time);
  ddy(:,dof) = gradient(dy(:,dof), time);
end

%% define the rest of the DMP parameters
DMP.goal = mean(y, 1);    % alternative goal: zeros(1,NS);
DMP.y0 = y(1,:);          % initial position
DMP.dy0 = dy(1,:);        % initial velocity

% initial covariance matrix for recursive regression, usually large
Pk = delta * eye(DMP.N);
% initial weights
DMP.w = zeros(DMP.N, NS);

% define Gausian kernel functions
DMP.c = linspace(pi/DMP.N, 2*pi - pi/DMP.N, DMP.N);
DMP.h = 2.5 * DMP.N * ones(1, DMP.N);

if DMP.Omega == 0
  % initializes Adaptive Frequency Oscilators for learning
  afs = AFS_init(10, NS, 2, 20);
  afs.Learn = 1;                 % Set to update frequency
else
  afs.Learn = 0;
end

if display_result
  Omega = [];
  Weights = [];
end
 
%% fit all points of the trajectory
for t = 1:NT
  if afs.Learn == 1
    if t == 1
      dt = time(2) - time(1);
    else
      dt = time(t) - time(t-1);
    end
    afs = AFS_integrate(afs, y(t,:), dt); % integrate
    DMP.Omega = min(afs.Omega)/(2*pi);
  end
  [DMP, Pk] = DMP_fit(y(t,:), dy(t,:), ddy(t,:), time(t), DMP, Pk, lambda);
  if display_result
    Omega = [Omega, DMP.Omega]; Weights = [Weights, norm(DMP.w)];
  end
end

if display_result
  figure(2);
  subplot(2,1,1)
  plot(time, Omega)
  title('Estimated frequency');
  subplot(2,1,2)
  plot(time, Weights)
  title('Norm of estimated weights');
end

%% Local function: DMP_fit
%
function [DMP, Pk] = DMP_fit(y, dy, ddy, t, DMP, Pk, lambda)
%% ---------------------------------------------------------------------
% Fits weights DMP.w to signal values y, dy, ddy using recursive
% regression with a forgetting factor
%
% INPUT:
%   y, yd, ydd, t: position, velocity, acceleration and time for one
%                  trajectory sample
%   lambda:        forgetting factor
%
% INPUT and OUTPUT values:
%   DMP: DMP values before and after one recursive regression learning step
%   PK:  current covariance matrix of recursive regression
%

  NS = size(DMP.w,2);
  % current phase
  fi = (2*pi) * DMP.Omega * t;
  % values of kernel functions at current phase
  psi = exp(DMP.h .* (cos(fi - DMP.c) - 1))';
  xx = psi / sum(psi);
  Lk = (Pk*xx) / (lambda + xx'*Pk*xx);
  for k = 1:NS
    % target for fitting
    ft = 1/((2*pi) * DMP.Omega)^2 * ddy(k) - ...
         DMP.a_z * (DMP.b_z * (DMP.goal(k)- y(k)) - dy(k) / (2*pi * DMP.Omega));
    % signal estimation error
    e = ft - xx'*DMP.w(:,k);
    % recursive regression step for weights
    DMP.w(:,k) = DMP.w(:,k) + e * Lk;
    % DMP.w(:,k) = DMP.w(:,k) + e*P(:,:,k)*xx;
  end
  % recursive regression step for covariance matrix
  Pk = (Pk - Lk*xx'*Pk) / lambda;

%% Local function AFS_integrate
%
function [ AFS ] = AFS_integrate(AFS, y, dt)
%%  ---------------------------------------------------------------------
%  Adaptive oscillator combined with the adaptive Fourier series
%  for frequency and phase estimation from an input singal.
%
%  Date:  January 2015
%  Authors: Tadej Petric
%
%  REFERENCE:
%  Petric, T., Gams, A., Ijspeert, A. J., & Zlajpah, L. (2011). On-line frequency adaptation and
%  movement imitation for rhythmic robotic tasks. The International Journal of Robotics Research,
%  30(14), 1775?1788. 
% ---------------------------------------------------------------------

ni = AFS.ni;
K = AFS.K;
Phi = AFS.Phi;
Omega = AFS.Omega;

% Integrate Fourier series coeficients using Euler's method
for j = 1:AFS.DOF
  if AFS.Learn == 1;        % check if learning is active
    e = y(j) - AFS.y_fb(j); % feedback error
  else
    e = 0;
  end

  % First the constant term of the Fourier series
  dalpha = e * ni;
  AFS.alpha(1,j) = AFS.alpha(1,j) + dalpha * dt;
  AFS.y_fb(j) = AFS.alpha(1,j);
  
  for i = 2:AFS.N
    dalpha = e * ni * cos((i-1)*Phi(j));
    AFS.alpha(i,j) = AFS.alpha(i,j) + dalpha * dt;

    dbeta = e * ni * sin((i-1)*Phi(j));
    AFS.beta(i,j) = AFS.beta(i,j) + dbeta * dt;

    AFS.beta(2,j) = 0; % this is requred otherwice the phase is not clearly defined!!!

    AFS.y_fb(j) = AFS.y_fb(j) + AFS.alpha(i,j) * cos((i-1)*Phi(j)) + ...
                                AFS.beta(i,j) * sin((i-1)*Phi(j));
  end

  dPhi = Omega(j) - K * e * sin(Phi(j));
  dOmega = - K * e * sin(Phi(j));

  % Euler integration - new frequency and phase 
  AFS.Phi(j) = Phi(j) + dPhi * dt; 
  AFS.Omega(j) = Omega(j) + dOmega * dt; 
end

%% Local function AFS_init
%
function [ AFS ] = AFS_init(N, DOF, ni, K)
%%  ---------------------------------------------------------------------
%  Initialization of an adaptive oscillator combined with the adaptive
%  Fourier series for frequency and phase estimation from an input signal.
%
%  Date  January 2015
%  Author: Tadej Petric
%
%
%  INPUTS and OUTPUTS:
%    N   - The approximation error depends only on the size of the Fourier series. Default is 10. 
%    DOF - Number of Degrees of Freedom. Each DOF works as a stand alone adaptive oscillator.
%    ni  - Gain for the Fourier series parameter adaption, default 2. 
%    K   - Gain for the frequency adaptation, default 20.
%
%  Remarks: Default values work best with signal between -1 and 1.
%
%  Additional details for parameter settings are in: 
%    - Gams, A., Ijspeert, A. J., Schaal, S., & Lenar?i?, J. (2009). On-line learning and modulation
%      of periodic movements with nonlinear dynamical systems. Autonomous Robots, 27(1), 3?23. 
%    - Petric, T., Gams, A., Ijspeert, A. J., & Zlajpah, L. (2011). On-line frequency adaptation and
%      movement imitation for rhythmic robotic tasks. The International Journal of Robotics Research,
%      30(14), 1775?1788. 
% ---------------------------------------------------------------------

if nargin == 0 % Default setings
  AFS.N = 10;
  AFS.DOF = 1;
  AFS.ni = 2;
  AFS.K = 20;
else
  AFS.N = N;
  AFS.DOF = DOF;
  AFS.ni = ni;
  AFS.K = K;
end

AFS.Omega = 2 * pi * ones(1, AFS.DOF);
AFS.Phi = zeros(1, AFS.DOF);

AFS.y_fb = zeros(1, AFS.DOF);

AFS.alpha = zeros(AFS.N, AFS.DOF);
AFS.beta = zeros(AFS.N, AFS.DOF);

AFS.Learn = 0;