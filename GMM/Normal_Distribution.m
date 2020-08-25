function out = Normal_Distribution(means,vars,x)
%% This function definite the Gaussian Distribution Function for high dimension
[Dimension1,Dimension2] = size(vars);
if Dimension1 ~= Dimension2
    disp('Matrix size does not fit!');
end

out = exp(-(x-means)*inv(vars)*(x-means)'/2)/(power(2*pi,Dimension1/2)*det(vars)^(0.5));
