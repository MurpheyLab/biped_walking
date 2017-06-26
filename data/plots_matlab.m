clear all

global param 
global master

%Read and format states and time
states = csvread('states.csv');
time = states(:,1);
J = states(:,end);
states(:,1) = [];
states(:,end) = [];
%Read and format actions
master.all_actions = csvread('actions.csv');
master.all_actions(:,end) = [];

% Load all settings in struct "param" 
param = settings();

figure();
plot(time, J, '.', 'MarkerSize', 10)

%Plots
figure,
plot(time,states(:,1:2)) %hold on, plot(time, pi/8*ones(length(time), 1))

% figure,plot(time,states(:,4))
%  figure,plot(states(:,1),states(:,2))
% figure,plot(diff(states(:,5)*60)), hold on, plot(diff(states(:,4)*60))

%Animate result
figure();
hold on
for i=1:length(time)
    clf

    animate2(time(i), states(i,:))
    pause(1/80)
%      writeVideo(writerObj,getframe(gcf));
    
end

plot_control();