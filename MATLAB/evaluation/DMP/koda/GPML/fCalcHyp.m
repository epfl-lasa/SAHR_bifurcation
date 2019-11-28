function Hyp=fCalcHyp(dmW,dmQ)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Calculate hyper parameters
%%% Hyp=fCalcHyp(dmW,dmQ)
%%%
%%% dmW(dof, nbrWeights, nbrDemos) - matrix of demo weights
%%% dmW(dim of Q, nbrDemos) - matrix of demo querries
%%%%%


%% INIT
path(genpath('GPML'), path)

%% SET UP GENE
D=size(dmQ,1); %dimensions of querry
n=size(dmQ,2); %number of demos

%%%     COVARIANCE
% set up simple covariance functions
cn  = {'covNoise'}; sn = .1;  hypn = log(sn);  % one hyperparameter
cc  = {@covConst};   sf = 2;  hypc = log(sf); % function handles OK
cl  = {@covLIN};              hypl = []; % linear is parameter-free
%     cla = {'covLINard'}; L = rand(D,1).*[1 1]'; hypla = log(L);  % linear (ARD)
cla = {'covLINard'}; L = rand(D,1); hypla = log(L);  % linear (ARD)
cli = {'covLINiso'}; l = rand(1);   hypli = log(l);    % linear iso
clo = {@covLINone}; ell = .9; hyplo = log(ell);  % linear with bias
cp  = {@covPoly,3}; c = 2; hypp = log([c;sf]);   % third order poly
cga = {@covSEard};   hypga = log([L;sf]);       % Gaussian with ARD
cgi = {'covSEiso'};  hypgi = log([ell;sf]);    % isotropic Gaussian
cgu = {'covSEisoU'}; hypgu = log(ell);   % isotropic Gauss no scale
cra = {'covRQard'}; al = 2; hypra = log([L;sf;al]); % ration. quad.
cri = {@covRQiso};          hypri = log([ell;sf;al]);   % isotropic
cma = {@covMaternard,5};  hypma = log([ell;sf]); % Matern class d=5
cmi = {'covMaterniso',3}; hypmi = log([ell;sf]); % Matern class d=3
cnn = {'covNNone'}; hypnn = log([L;sf]);           % neural network
cpe = {'covPeriodic'}; p = 2; hyppe = log([ell;p;sf]);   % periodic
cpn = {'covPeriodicNoDC'}; p = 2; hyppe = log([ell;p;sf]); % w/o DC
cpc = {'covCos'}; p = 2; hypcpc = log([p;sf]);         % cosine cov
% cca = {'covPPard',3}; hypcc = hypm; % compact support poly degree 3
% cci = {'covPPiso',2}; hypcc = hypm; % compact support poly degree 2
cgb = {@covGaboriso}; ell = 1; p = 1.2; hypgb=log([ell;p]); % Gabor
% csm = {@covSM,4}; hypsm = log([w;m(:);v(:)]);    % Spectral Mixture

% set up composite i.e. meta covariance functions
csc = {'covScale',{cgu}};    hypsc = [log(3); hypgu];  % scale by 9
csu = {'covSum',{cn,cc,cl}}; hypsu = [hypn; hypc; hypl];      % sum
cpr = {@covProd,{cn,cc}};   hyppr = [hypn; hypc];       % product
mask = [0,1,0]; %   binary mask excluding all but the 2nd component
cma = {'covMask',{mask,cgi{:}}}; hypma = hypgi;
% isotropic periodic rational quadratic
cpi = {'covPERiso',{@covRQiso}};
% periodic Matern with ARD
cpa = {'covPERard',{@covMaternard,3}};
% additive based on SEiso using unary and pairwise interactions
cad = {'covADD',{[1,2],'covSEiso'}};


%%%     MEAN
% set up simple mean functions
m0 = {'meanZero'};  hyp0 = [];      % no hyperparameters are needed
m1 = {'meanOne'};   hyp1 = [];      % no hyperparameters are needed
mc = {@meanConst};  hypc = 2;  % also function handles are possible
ml = {@meanLinear}; hypl = [2;3];              % m(x) = 2*x1 + 3*x2
mp = {@meanPoly,2}; hypp = [1;1;2;3];  % m(x) = x1+x2+2*x1^2+3*x2^2

% set up composite mean functions
msc = {'meanScale',{m1}};      hypsc = [3; hyp1];      % scale by 3
msu = {'meanSum',{m0,mc,ml}};  hypsu = [hyp0; hypc; hypl];    % sum
mpr = {@meanProd,{mc,ml}};     hyppr = [hypc; hypl];      % product
mpo = {'meanPow',3,msu};       hyppo = hypsu;         % third power
mask = [false,true];     % mask excluding all but the 2nd component
mma = {'meanMask',mask,ml}; hypma = hypl(mask);


%%%     LIKELIHOOD
f = randn(n,1);       % create random latent function values
% set up simple classification likelihood functions
yc = sign(f);
lc0 = {'likErf'};     hypc0 = [];   % no hyperparameters are needed
lc1 = {@likLogistic}; hypc1 = [];    % also function handles are OK
lc2 = {'likUni'};     hypc2 = [];
lc3 = {'likMix',{'likUni',@likErf}}; hypc3 = log([1;2]); %mixture

% set up simple regression likelihood functions
yr = f + randn(n,1)/20;
sn = 0.1;                                % noise standard deviation
lr0 = {'likGauss'};   hypr0 = log(sn);
lr1 = {'likLaplace'}; hypr1 = log(sn);
lr2 = {'likSech2'};   hypr2 = log(sn);
nu = 4;                              % number of degrees of freedom
lr3 = {'likT'};       hypr3 = [log(nu-1); log(sn)];
% lr4 = {'likMix',{lr0,lr1}}; hypr4 = [log([1,2]);hypr0;hypr1];

a = 1; % set up warped Gaussian with g(y) = y + a*sign(y).*y.^2
lr5 = {'likGaussWarp',['poly2']}; hypr5 = log([a;sn]);
lr6 = {'likGumbel','+'}; hypr6 = log(sn);

% set up Poisson regression
yp = fix(abs(f)) + 1;
lp0 = {@likPoisson,'logistic'}; hypp0 = [];
lp1 = {@likPoisson,'exp'};      hypp1 = [];

% set up other GLM likelihoods for positive or interval regression
lg1 = {@likGamma,'logistic'}; al = 2;    hyp.lik = log(al);
lg2 = {@likInvGauss,'exp'};   lam = 1.1; hyp.lik = log(lam);
lg3 = {@likBeta,'expexp'};    phi = 2.1; hyp.lik = log(phi);
lg4 = {@likBeta,'logit'};     phi = 4.7; hyp.lik = log(phi);

%%%overwrite strat Hyp to a good init value i.e. guess it
% hypn=log(.05); hypga=[log([.05 .5]) log(1.2)]';
%
% dof9; nW8; infEP
% hypn=log(.05); hypga=[log([.1 .5]) log(1.2)]';
% hypn=log(.05); hypga=[log([rand(1)/10 rand(1)*10]) log(1.2)]';
% hypn=log(.07); hypga=[log([.04 10]) log(1.2)]';
%
% hypc=log(.0000001);
% hypc=log(1);
% hypr0=log(.07);

%% CALC HYP PARAMS

GPsett.CF = {'covSum',{cn,cga}}; initHyp.cov = [hypn; hypga];
GPsett.MF = mc; initHyp.mean = hypc;
%meanfunc = mp; initHyp.mean = hypp;
GPsett.LF = lr0; initHyp.lik = hypr0;

nDof=size(dmW,1);
nW=size(dmW,2);

warning off
disp('>>> CALCULATING HYP PARAMS >>>')
for cDof=1:nDof;
    for cW=1:nW;

        disp(['> Current Dof:',num2str(cDof),' and Weight:',num2str(cW),' ...'])
        Hyp{cDof}{cW}=minimize(initHyp, @gp, -1000, @infExact, GPsett.MF, GPsett.CF, GPsett.LF, dmQ', squeeze(dmW(cDof,cW,:)));
        
        Hyp{cDof}{cW}.GPsett=GPsett;
        
    end
end

disp('>>> DONE!! ')



end