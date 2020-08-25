function [varargout] = Gaussian_Mixture_Regression(alpha,means,sigma,n_input,n_output,input,Numsamples)            
%% This function is used to tranform GMM to GMR
global GMM
nbData = length(input);
nbStates = size(sigma,3);

for m = 1:nbData
    for i=1:nbStates
        beta(m,i) = alpha(i).*Normal_Distribution(means(i,n_input), sigma(n_input,n_input,i),input(m));
    end
end
beta = beta./repmat(sum(beta,2)+realmin,1,nbStates);

for j=1:nbStates
   Y_mean(:,:,j) = repmat(means(j,n_output)',1,nbData) + sigma(n_output,n_input,j)*inv(sigma(n_input,n_input,j)) * (input'-repmat(means(j,n_input)',1,nbData));
end
gamma_tmp = reshape(beta,[1 size(beta)]);
y_tmp2 = repmat(gamma_tmp,[length(n_output) 1 1]) .* Y_mean;
Y = sum(y_tmp2,3)';

for j=1:nbStates
  Sigma_y_tmp(:,:,1,j) = sigma(n_output,n_output,j) - (sigma(n_output,n_input,j)*inv(sigma(n_input,n_input,j))*sigma(n_input,n_output,j));
end
beta_tmp = reshape(beta,[1 1 size(beta)]);
Sigma_y_tmp2 = repmat(beta_tmp.*beta_tmp, [length(n_output) length(n_output) 1 1]) .* repmat(Sigma_y_tmp,[1 1 nbData 1]);
Sigma_Y = sum(Sigma_y_tmp2,4);

varargout(1) = {Y};
varargout(2) = {Sigma_Y};
%% Plot GMR
Numsamples = Numsamples;
time = linspace(min(GMM.dataset(:,1)),max(GMM.dataset(:,1)),Numsamples);
figure('NumberTitle', 'off', 'Name','GMR Expected Trajectory','position',[1000,700,800,300])
subplot(2,2,1)
for j=1:Numsamples
    ymax(j) = Y(j,1) + sqrtm(3.*Sigma_Y(1,1,j));
    ymin(j) = Y(j,1) - sqrtm(3.*Sigma_Y(1,1,j));
end
patch([time time(end:-1:1)], [ymax(1:end) ymin(end:-1:1)], [0.6 0.6 1], 'LineStyle', 'none');hold on
plot(time, Y(1:Numsamples,1), '-', 'lineWidth', 3, 'color', [0 0 0.8]); hold on;
xlabel('t','fontsize',16); ylabel('X_{position}','fontsize',16);

subplot(2,2,3)
for j=1:Numsamples
    ymax(j) = Y(j,2) + sqrtm(3.*Sigma_Y(2,2,j));
    ymin(j) = Y(j,2) - sqrtm(3.*Sigma_Y(2,2,j));
end
patch([time time(end:-1:1)], [ymax(1:end) ymin(end:-1:1)], [0.6 0.6 1], 'LineStyle', 'none');hold on
plot(time, Y(1:Numsamples,2), '-', 'lineWidth', 3, 'color', [0 0 0.8]); hold on;
xlabel('t','fontsize',16); ylabel('Y_{position}','fontsize',16);

subplot(2,2,[2,4])
nbDrawingSeg = 40;
t = linspace(-pi, pi, nbDrawingSeg)';
for j=1:Numsamples
    stdev = sqrtm(3.0.*Sigma_Y(:,:,j));
    X = [cos(t) sin(t)] * real(stdev) + repmat(Y(j,:),nbDrawingSeg,1);
    patch(X(:,1), X(:,2), [.6 .6 1], 'LineStyle', 'none');hold on
end
plot(Y(1:Numsamples,1), Y(1:Numsamples,2), '-', 'lineWidth', 3, 'color', [0 0 .8] );
xlabel('X_{position}','fontsize',16); ylabel('Y_{position}','fontsize',16);
hold off
drawnow;
end 
