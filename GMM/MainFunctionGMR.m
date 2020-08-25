%% This document demonstrate how to build GMM and tranforme it to GMR
% note that the result of GMM is significantly influences by initial
% parameters,here I use Kmeans algo to get initial means and vars;
% alpha vector is given mannually
clc;
clf;
clear all;
global GMM;
%number of GMM component
K = 10;
iteration = 100;

LoadTrajectory

%dataset
dataset = [X_TrajectoryMatrix Y_TrajectoryMatrix(:,2)];

%Record dimension info
[NumData, Dimension] = size(dataset);

%% kmeans for means
init_vars = zeros(Dimension,Dimension,K);
[labels,init_means] = kmeans(dataset,K);
for k=1:K
    init_vars(:,:,k) = cov(dataset(labels==k,:));
end
init_alpha = ones(K,1)/K;

%% Train and plot GMM 
Gaussian_Mixture_Model('initialization',dataset,init_means,init_vars,init_alpha,K)
disp('GMM initialized');

[means,sigma,alpha] = Gaussian_Mixture_Model('Train',iteration);
disp('GMM trained')

Gaussian_Mixture_Model('plot')
disp('GMM encoding result has shown!')
%% Traing and plot GMR 
[F,Sigma_F]= Gaussian_Mixture_Regression(alpha,means,sigma,[1],[2,3],dataset(:,1),Numsamples);
disp('GMR trained')
disp('GMR result has shown!')