function T = Desired_Trajectory1(length,dt)
%this function is used to build an initial trajectory.
%here we give an example trajectory
trajectory = @(t)(10*sin(2*pi*t)*exp(-t^2)+1);
T = zeros(length,3);
for i= 0:1:length-1
    T(i+1,1) = trajectory(i*dt);
end

T(1:end-1,2) = diff(T(:,1),1)/dt;
T(end,2) = T(end-1,2);
T(1:end-1,3) = diff(T(:,2),1)/dt;
T(end,2) = T(end-1,2);

