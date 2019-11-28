
addpath('GPML')
% clear
% load baza


nDm=11; %number of demos, i.e., examples
nDof=1; %number of DOFs
nW=25;
dQ=1; %dim of querries


dmW=zeros(nDof,nW+1,nDm);
dmQ=zeros(dQ,nDm);
% query = [1,2,3,4];
query = goals;

%%%prep dmW and dmQ
for n=1:nDm
    dmW(:,:,n)=[www{n}(:,1)' DMP_database{n}.dy0];
    dmQ(n)=query(n);
end


%%% calculate hyper parameters
Hyp=fCalcHyp(dmW,dmQ);

% nwQ = 2.5; % new query
nwW=fCalcNewW(dmW,dmQ,Hyp,nwQ);

%%

figure; hold on
for i = 1:nDm
    plot(www{i}(:,1));
end
plot(nwW,'linewidth',2)