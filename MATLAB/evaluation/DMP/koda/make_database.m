% generate base
clear; close all


%% trajectories
dt = 0.002;
t_end = 2;
time = linspace(0, t_end, t_end / dt + 1);
iter = 0;
for i = 0:0.2:2
    iter = iter+1;
    trj{iter}(:,1) = -cos(2*pi*time)+time*i;
end

%% plot trajectories
% figure; hold on
% for iter = 1:11
%     plot(trj{iter})
% end

%% make nice database entries for generalization
for iter = 1:11
    qPath{iter}(:,1)=time;
    qPath{iter}(:,2)=trj{iter};
    qPath{iter}(:,3)=gradient(trj{iter}, time);
    qPath{iter}(:,4)=gradient(qPath{iter}(:,3), time);
end
cc=linspace(0,1,25);
h=0.51;
goals = -1:0.4:3;

for jj= 1:length(qPath)
    final(jj,:)=qPath{jj}(end,[2]);
end

for i = 1
    sp{i} = spline(goals', final(:,i));
end
sp{i+1} = spline(goals', ones(1,11)*t_end);


already_exec = 0;
return
