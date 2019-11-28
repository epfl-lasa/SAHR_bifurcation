function [nwW]=correct_weight(dmW,dmQ,nwW,nwQ,weight)

Hyp1=fCalcHyp_separate(dmW,dmQ,weight);

nwW1 = fCalcNewW_separate(dmW,dmQ,Hyp1,nwQ,weight);

nwW(weight) = nwW1(weight);

plot(nwW','r--')
