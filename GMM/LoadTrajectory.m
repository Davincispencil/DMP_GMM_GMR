%% This function load S.mat files which contains several trajectories of character S
load S.mat
NumTrajectories = 15;
Numsamples = 200;
Dimension = 2;
X_TrajectoryMatrix = zeros(Numsamples*NumTrajectories,Dimension);
Y_TrajectoryMatrix = zeros(Numsamples*NumTrajectories,Dimension);

indices = 1:Numsamples;
dt = 0.007;
t = indices*dt;

for i=1:NumTrajectories
    Position = demos{i}.pos;
    X_TrajectoryMatrix((i-1)*Numsamples+indices,2) = Position(1,:);
    X_TrajectoryMatrix((i-1)*Numsamples+indices,1) = t;
    
    Y_TrajectoryMatrix((i-1)*Numsamples+indices,2) = Position(2,:);
    Y_TrajectoryMatrix((i-1)*Numsamples+indices,1) = t;
end


figure('NumberTitle', 'off', 'Name','Initial Trajectory information','position',[100,700,800,300])
subplot(2,2,[2 4]);hold on ;
for i=1:NumTrajectories
    Position = demos{i}.pos;
    plot(Position(1,:),Position(2,:), 'x', 'markerSize', 4, 'color', [.3 .3 .3]);hold on;
    xlabel('X_{position}','fontsize',16);ylabel('Y_{position}','fontsize',16)
end 
subplot(2,2,1);hold on;
plot(t,Position(1,:),'x', 'markerSize', 4, 'color', [.3 .3 .3]);hold on;
xlabel('t','fontsize',16);ylabel('X_{position}','fontsize',16)

subplot(2,2,3);hold on;
plot(t,Position(2,:),'x', 'markerSize', 4, 'color', [.3 .3 .3]);hold on;
xlabel('t','fontsize',16);ylabel('Y_{position}','fontsize',16)
hold off
drawnow;

