function [varargout] = Discret_Movement_Primitive(action,varargin)
%% DMPs is designed to be a global structure which contains several dmp
%here in this document,we only added 1 dmp, which means only one trajectory
%was presented and learned.
% ID -> the IDth DMP component
% n_rfs -> the number of RBF nodes
% T -> time 
% dt -> step time
% initial_goal -> the goal position of initial trajectory
% tau -> temporal scale factor
% s -> space scale factor
% alpha_z, beta_z, alpha_x, alpha_v, beta_v are constants
% x¡¢v are phase factors
% w -> weights
% phi -> outputs of RBF nodes
% f -> acceleration vetor to be fitted
% st2 sxtd are two variables for calculating weights
% Y refers to new trajectory
%%
global DMPs
switch lower(action)
    %% clear all dmps existed
    case 'clear'
       ID     = varargin{1};
     if exist('dmps')
         if length(DMPs) >= ID
            DMPs(ID) = [];
         end
     end
   
    case 'init'
        ID = varargin{1};
        DMPs(ID).name = varargin{2};
        DMPs(ID).n_rfs = varargin{3};
        DMPs(ID).T = varargin{4};
        DMPs(ID).dt = varargin{5};
        DMPs(ID).initial_goal = varargin{6};
        DMPs(ID).tau = varargin{7};
        DMPs(ID).s = 1;
        
        % the time constants for chosen for critical damping
        DMPs(ID).alpha_z = 25;
        DMPs(ID).beta_z  = DMPs(ID).alpha_z/4;
        DMPs(ID).alpha_x = DMPs(ID).alpha_z/3;
        DMPs(ID).alpha_v = DMPs(ID).alpha_z;
        DMPs(ID).beta_v  = DMPs(ID).beta_z;
%         Sample_number =DMPs(ID).T/DMPs(ID).dt;
        Sample_number = 200;
        DMPs(ID).x = zeros(Sample_number,1);
        DMPs(ID).v = zeros(Sample_number,1);
        DMPs(ID).c = zeros(DMPs(ID).n_rfs,1);
        DMPs(ID).D =  DMPs(ID).c ;
        DMPs(ID).w =  DMPs(ID).c ;
        DMPs(ID).phi =  zeros(Sample_number,DMPs(ID).n_rfs);
        DMPs(ID).f = zeros(Sample_number,1);
        DMPs(ID).sx2 =  DMPs(ID).c ;
        DMPs(ID).sxtd =  DMPs(ID).c ; 
        DMPs(ID).Y = zeros(Sample_number,3);
    
    case 'train'    
        ID = varargin{1};
        Trajectory = varargin{2};
        DMPs(ID).flag = varargin{3}; 
        %decide the value of c based on the number of radial basis function
        t = (0:1/(DMPs(ID).n_rfs-1):1)'*  DMPs(ID).T ;
        if (DMPs(ID).flag == 1)
            % anaytical solutions x(t) = 1-(1+alpha/2*t)*exp(-alpha/2*t)
            DMPs(ID).c = (1+DMPs(ID).alpha_z/2*t).*exp(-DMPs(ID).alpha_z/2*t);
            %calcute x v when we use second order canonical system
            DMPs(ID).x(1) = 1;
            DMPs(ID).v(1) = 0;
            for i=2:length(DMPs(ID).x)
               vd = DMPs(ID).alpha_v * (DMPs(ID).beta_v *(0-DMPs(ID).x(i-1))-DMPs(ID).v(i-1));
               xd = DMPs(ID).v(i-1);
               
               DMPs(ID).v(i) = DMPs(ID).v(i-1) + vd * DMPs(ID).dt;
               DMPs(ID).x(i) = DMPs(ID).x(i-1) + xd * DMPs(ID).dt;              
            end
            
        elseif(DMPs(ID).flag == 0)
            % anaytical solutions x(t) = exp(-alpha*t)
            DMPs(ID).c = exp(-DMPs(ID).alpha_x*t);
            %calcute x when we use first order canonical system
            DMPs(ID).x(1) = 1;
            for i= 2:length(DMPs(ID).x)
                dx = -DMPs(ID).alpha_x * DMPs(ID).x(i-1);
                DMPs(ID).x(i) = DMPs(ID).x(i-1) + dx * DMPs(ID).dt;
            end
            
        else
            disp('flag is not well designed!')
        end
        
        %decide the value of width for radial basis function
        DMPs(ID).D       = (diff(DMPs(ID).c)*.8).^2;
        DMPs(ID).D       = 1./[DMPs(ID).D;DMPs(ID).D(end)];
        %calculate the value of phi in forms of matrix
        DMPs(ID).phi = exp(-0.5*((DMPs(ID).x*ones(1,length(DMPs(ID).c))-ones(length(DMPs(ID).x),1)*DMPs(ID).c').^2).*(ones(length(DMPs(ID).x),1)*DMPs(ID).D'));
        
        Fd  = Trajectory(:,3)-DMPs(ID).alpha_z*(DMPs(ID).beta_z*(DMPs(ID).initial_goal-Trajectory(:,1))-Trajectory(:,2));
        %get the value of fd sx2 sxtd and w
        if (DMPs(ID).flag == 1)
            DMPs(ID).sx2 = sum(DMPs(ID).v.*DMPs(ID).phi.*Fd,1)';
            DMPs(ID).sxtd = sum((DMPs(ID).v).^2 .* DMPs(ID).phi,1)';
            DMPs(ID).w = DMPs(ID).sx2 ./ (DMPs(ID).sxtd + 0.000001);
            
            f = DMPs(ID).phi * DMPs(ID).w ./ sum(DMPs(ID).phi,2);
            f = f .* DMPs(ID).v; 
        else
            DMPs(ID).sx2 = sum(DMPs(ID).x .*Fd .*DMPs(ID).phi,1)';
            DMPs(ID).sxtd = sum((DMPs(ID).x).^2 .* DMPs(ID).phi,1)';
            DMPs(ID).w = DMPs(ID).sx2 ./ (DMPs(ID).sxtd + 0.000001);
            
            f = DMPs(ID).phi * DMPs(ID).w ./ sum(DMPs(ID).phi,2);
            f = f .* DMPs(ID).x; 
            
        end
        
        %formulate the same trajectory to the original one
        Y     = zeros(size(Trajectory));
        Y(1,:) = Trajectory(1,:);
        for i=2:length(DMPs(ID).x)
        dz = DMPs(ID).alpha_z * (DMPs(ID).beta_z * (DMPs(ID).initial_goal - Y(i-1,1))-Y(i-1,2))+f(i-1);
        dy = Y(i-1,2);
        Y(i,1) = dy * DMPs(ID).dt + Y(i-1,1); 
        Y(i,2) = dz * DMPs(ID).dt + Y(i-1,2);
        Y(i,3) = dz;
        end
    varargout(1) = {Y};
  
    case 'reset state'
        ID = varargin{1};
        DMPs(ID).new_start = varargin{2};
        DMPs(ID).new_goal = varargin{3};
        DMPs(ID).s = varargin{4};
    
    case 'fit'
        ID = varargin{1};
        DMPs(ID).tau = varargin{2};
        %get the value of fd sx2 sxtd and w
        if (DMPs(ID).flag == 1)
            f = DMPs(ID).phi * DMPs(ID).w ./ sum(DMPs(ID).phi,2);
            f = f .* DMPs(ID).v * DMPs(ID).s;
        else
            f = DMPs(ID).phi * DMPs(ID).w ./ sum(DMPs(ID).phi,2);
            f = f .* DMPs(ID).x * DMPs(ID).s;
        end
        
        %fit new trajectory 
        DMPs(ID).Y(1,:) = DMPs(ID).new_start;
        for i=2:length(DMPs(ID).x)
        dz = DMPs(ID).alpha_z * (DMPs(ID).beta_z * (DMPs(ID).new_goal - DMPs(ID).Y(i-1,1))-DMPs(ID).Y(i-1,2))+f(i-1);
        dy = DMPs(ID).Y(i-1,2);
        
        DMPs(ID).Y(i,1) = dy * DMPs(ID).dt * DMPs(ID).tau + DMPs(ID).Y(i-1,1); 
        DMPs(ID).Y(i,2) = dz * DMPs(ID).dt * DMPs(ID).tau + DMPs(ID).Y(i-1,2);
        DMPs(ID).Y(i,3) = dz * DMPs(ID).tau;
        end
    varargout(1) = {DMPs(ID).Y};
end