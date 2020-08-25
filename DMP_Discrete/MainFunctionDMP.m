%% This main function uses Desired_Trajectory1.m to create a ideal trajectory 
%and create new trajectory based on standard DMP algo

global DMPs
% goal position
initial_goal = 1;
new_goal = 5;
%temporal scale factor
tau = 1;
%space scale factor
scale = 1;
%start point for new trajectory
new_start = [1.5,62.8314,-2.8575];
%time
T = 2;
dt = 0.01;
n_rfs = 50;
%control the order of canonical system 
%flag = 1 -> second order canonical system
%flag = 0 -> first order canonical system
flag = 0;

%Modifying the first DMP
ID = 1;
Discret_Movement_Primitive('clear',ID);
disp('action1: DMPs is empty now');
Discret_Movement_Primitive('init',ID,'Desired_trajectory',n_rfs,T,dt,initial_goal,tau);
disp('action2: new dmp initialized');

%parameter initialization
Trajectory = zeros(T/dt,3);

%T contains information of desired trajectory's position, velocity and acceleration
Trajectory = Desired_Trajectory1(T/dt,dt);
disp('action3: get the original information of desired trajectory');

%batch fit w
Initial_Trajectory = Discret_Movement_Primitive('train',ID,Trajectory,flag);
disp("action4: dmp's parameters is successfully trained");

%modify parameter
Discret_Movement_Primitive('reset state',ID,new_start,new_goal,scale);
disp('action5: reset dmp state for new goal');

%fit new trajectory 
Discret_Movement_Primitive('fit',ID,tau);
disp('action6: new goal information getted');

disp('action7£ºplot fitting figures');
% plotting
time = (0:dt:T-dt)';

% plot position, velocity, acceleration vs. target
figure('NumberTitle', 'off', 'Name','DMP results with RBF','position',[1000,200,800,300])
subplot(311);
plot(time,[DMPs(ID).Y(:,1),Trajectory(:,1)]);
title('Position');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(312);
plot(time,[DMPs(ID).Y(:,2),Trajectory(:,2)]);
title('Velocity');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(313);
plot(time,[DMPs(ID).Y(:,3),Trajectory(:,3)]);
title('Accerleration');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

drawnow;






