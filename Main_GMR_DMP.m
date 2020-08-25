%% This main funciton is the top one 
% excuting this funciton, we can launce GMR to trait trajectories of
% character s and get the GMM model and GMR model
% In addition, F refers to the expected results of GMR
% Then, we can use DMPs to learn character S' s x position and y_position
% and get a similar character 
%% launch GMR main functin
clc
clf
clear all
MainFunctionGMR

%% design new main function DMP for using Y (trajectroy estimated by GMR)
global DMPs;
Num = size(F,2);
% obtain velocity and acceleration from position
TrajectoryForDMPs = zeros(Numsamples,3,Num);

indices = 1:Numsamples;
t = indices * dt;

for i= 1:Num
    TrajectoryForDMPs(1:Numsamples,1,i) = F(1:Numsamples,i);
    TrajectoryForDMPs(1:end-1,2,i) = diff(TrajectoryForDMPs(:,1,i),1)/dt;
    TrajectoryForDMPs(end,2) = TrajectoryForDMPs(end-1,2,i);
	TrajectoryForDMPs(1:end-1,3,i) = diff(TrajectoryForDMPs(:,2,i),1)/dt;
    TrajectoryForDMPs(end,2,i) = TrajectoryForDMPs(end-1,2,i);
end

initial_goal = F(Numsamples,:);
new_goal = initial_goal + [2,1];
%temporal scale factor
tau = 1;
% space scale factor
scale = 1;
%start point for new trajectory
initial_start = F(1,:);
new_start = initial_start;
T =(Numsamples)*dt;
n_rfs = 50;
%control the order of canonical system 
%flag = 1 -> second order canonical system
%flag = 0 -> first order canonical system
flag = 0;

%Modifying the first DMP
Discret_Movement_Primitive('clear',1);
disp('DMPs is empty now');
Discret_Movement_Primitive('init',1,'Desired_trajectory',n_rfs,T,dt,initial_goal(1),tau);
disp('The first dmp initialized');
Discret_Movement_Primitive('init',2,'Desired_trajectory',n_rfs,T,dt,initial_goal(2),tau);
disp('The second dmp initialized');

%batch fit w
Initial_Trajectory = Discret_Movement_Primitive('train',1,TrajectoryForDMPs(:,:,1),flag);
disp("The first dmp's parameters is successfully trained");

Initial_Trajectory = Discret_Movement_Primitive('train',2,TrajectoryForDMPs(:,:,2),flag);
disp("The second dmp's parameters is successfully trained");

%modify parameter (new_start = 起始点位置、速度、加速度) 最终位置goal (系统收敛速度控制因子tau) scale
Discret_Movement_Primitive('reset state',1,new_start(1),new_goal(1),scale);
disp('reset dmp1 state for new goal1');
Discret_Movement_Primitive('reset state',2,new_start(2),new_goal(2),scale);
disp('reset dmp2 state for new goal2');

%fit new trajectory 
Discret_Movement_Primitive('fit',1,tau);
disp('new goal1 information getted');
Discret_Movement_Primitive('fit',2,tau);
disp('new goal2 information getted');

disp('plot fitting figures');
% plot position, velocity, acceleration vs. target
figure('NumberTitle', 'off', 'Name','DMP results with GMR','position',[1000,200,800,300])
subplot(2,2,1);hold on;
plot(t,[DMPs(1).Y(:,1),TrajectoryForDMPs(:,1,1)]);
title('X_{Position}');
aa=axis;
axis([min(t) max(t) aa(3:4)+[-1,1]]);

subplot(2,2,3);hold on;
plot(t,[DMPs(2).Y(:,1),TrajectoryForDMPs(:,1,2)]);
title('Y_{Position}');
aa=axis;
axis([min(t) max(t) aa(3:4)+[-0.5,0.5]]);

subplot(2,2,[2,4]);hold on;
plot([DMPs(1).Y(:,1),TrajectoryForDMPs(:,1,1)],[DMPs(2).Y(:,1),TrajectoryForDMPs(:,1,2)]);
title('2D-Trajectory');
aa=axis;
axis([aa(1:2)-[1,-1] aa(3:4)+[-0.5,0.5]])
hold off;
drawnow;